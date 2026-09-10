/**
 * *********************************************************
 *
 * @file: wamv_lqr_controller.cpp
 * @brief: Contains the LQR controller for WAMV USV
 * @author: Yang Haodong
 * @date: 2025-04-14
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "controller/lqr_controller/otter_lqr_controller.h"

using namespace rmp::common::util;
using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::trajectory_planner;

namespace rmp
{
namespace controller
{
int LQRControllerOtter::state_dim_ = 6;
int LQRControllerOtter::control_dim_ = 2;

/**
 * @brief Construct a new LQRControllerOtter object
 */
LQRControllerOtter::LQRControllerOtter() : Controller()
{
}

/**
 * @brief Construct a new LQRControllerOtter object with specific parameters
 * @param control_frequency   the control frequency in Hz
 * @param usv_model           the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros         a pointer to the costmap for collision checking and environmental awareness
 * @param tf                  a pointer to the TF2 buffer for managing transformations
 * @param goal_dist_tolerance the tolerance for the goal distance
 * @param lookahead_time      the lookahead time for trajectory prediction
 * @param max_iterations      the maximum number of iterations for solving the LQR problem
 * @param discrete_dt         the time step used for discretizing system dynamics
 * @param solve_eps           the convergence threshold for solving the LQR problem
 * @param Q_diag              the diagonal elements of the Q matrix (state cost matrix)
 * @param R_diag              the diagonal elements of the R matrix (control cost matrix)
 */
LQRControllerOtter::LQRControllerOtter(double control_frequency, rmp::common::structure::USV_MODEL usv_model,
                                     nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                                     double goal_dist_tolerance, double lookahead_time, int max_iterations,
                                     double discrete_dt, double solve_eps, const std::vector<double>& Q_diag,
                                     std::vector<double>& R_diag)
  : Controller(control_frequency, usv_model, costmap_ros, tf)
  , goal_dist_tolerance_(goal_dist_tolerance)
  , lookahead_time_(lookahead_time)
  , max_iterations_(max_iterations)
  , discrete_dt_(discrete_dt)
  , solve_eps_(solve_eps)
{
  initialized_ = true;
  Q_ = Eigen::MatrixXd::Zero(Q_diag.size(), Q_diag.size());
  R_ = Eigen::MatrixXd::Zero(R_diag.size(), R_diag.size());
  for (int i = 0; i < Q_diag.size(); ++i)
  {
    Q_(i, i) = Q_diag[i];
  }
  for (int i = 0; i < R_diag.size(); ++i)
  {
    R_(i, i) = R_diag[i];
  }

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("controller_lqr_otter");
  lookahead_point_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 1);
}
/**
 * @brief Computes the control command for the vehicle
 * @param ctrl_cmd the control command to be populated
 * @return true if the control command is successfully computed, false otherwise
 */
bool LQRControllerOtter::computeControlCommand(ControlCommand& ctrl_cmd)
{
  double dist_to_goal = std::hypot(usv_state_.x - goal_.x(), usv_state_.y - goal_.y());
  if (dist_to_goal < goal_dist_tolerance_)
  {
    is_goal_reached_ = true;
    ctrl_cmd.setThrustCmdLeft(0.0);
    ctrl_cmd.setThrustCmdLateral(0.0);
    ctrl_cmd.setThrustCmdRight(0.0);
    return true;
  }

  // lookahead
  TrajectoryCell desired_pt;
  local_plan_.getReferencWithTime(lookahead_time_, desired_pt);

  // control process
  Eigen::MatrixXd K;
  if (_solveLQRProblem(desired_pt, K))
  {
    Eigen::Matrix2d R;
    Eigen::Vector2d vel_n, vel_b;
    R << std::cos(usv_state_.psi), -std::sin(usv_state_.psi), std::sin(usv_state_.psi), std::cos(usv_state_.psi);
    vel_n << usv_state_.x_dot, usv_state_.y_dot;
    vel_b = R.transpose() * vel_n;

    Eigen::VectorXd curr_state(state_dim_);
    Eigen::VectorXd desired_state(state_dim_);
    curr_state << usv_state_.x, usv_state_.y, usv_state_.psi, vel_b.norm(), 0.0, usv_state_.psi_dot;
    desired_state << desired_pt.x, desired_pt.y, desired_pt.theta, desired_pt.velocity, 0.0, 0.0;
    Eigen::VectorXd d_state = curr_state - desired_state;
    d_state(2) = pi2pi(d_state(2));

    Eigen::Vector2d tau = K * d_state;
    ctrl_cmd.setThrustCmdLeft(tau[0]);
    ctrl_cmd.setThrustCmdRight(tau[1]);
    regularControlCommand(ctrl_cmd);

    // visualization
    const auto& visualizer = VisualizerPtr::Instance();
    Point3d vis_pose(desired_pt.x, desired_pt.y, desired_pt.theta);
    visualizer->publishPoses(vis_pose, lookahead_point_pub_, usv_model_.ns + "/map", usv_model_.ns + "_lookahead_point",
                             Visualizer::DARK_BLUE, 3.0, 0.5, 0.5);

    return true;
  }

  return false;
}

/**
 * @brief Solves the LQR optimization problem to compute the feedback gain matrix
 * @param desired_pt the desired trajectory point
 * @param K          the resulting feedback gain matrix
 * @return true if the problem is successfully solved, false otherwise
 */
bool LQRControllerOtter::_solveLQRProblem(const TrajectoryCell& desired_pt, Eigen::MatrixXd& K)
{
  // 1) set dynamics
  double psi_ref = desired_pt.theta;
  double u_ref = desired_pt.velocity;
  double v_ref = 0.0;
  double r_ref = 0.0;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  A(0, 2) = -u_ref * std::sin(psi_ref) - v_ref * std::cos(psi_ref);
  A(0, 3) = std::cos(psi_ref);
  A(0, 4) = -std::sin(psi_ref);
  A(1, 2) = u_ref * std::cos(psi_ref) - v_ref * std::sin(psi_ref);
  A(1, 3) = std::sin(psi_ref);
  A(1, 4) = std::cos(psi_ref);
  A(2, 5) = 1.0;
  A(3, 3) = usv_model_.x_u / (usv_model_.mass + usv_model_.x_dot_u);
  A(3, 5) = (usv_model_.mass - usv_model_.y_dot_v) / (usv_model_.mass + usv_model_.x_dot_u);
  A(4, 4) = usv_model_.y_v / (usv_model_.mass + usv_model_.y_dot_v);
  A(4, 5) = -(usv_model_.mass - usv_model_.x_dot_u) / (usv_model_.mass + usv_model_.y_dot_v);
  A(5, 3) = (usv_model_.y_dot_v - usv_model_.x_dot_u) / (usv_model_.i_z + usv_model_.n_dot_r) * v_ref;
  A(5, 4) = (usv_model_.y_dot_v - usv_model_.x_dot_u) / (usv_model_.i_z + usv_model_.n_dot_r) * u_ref;
  A(5, 5) = usv_model_.n_r / (usv_model_.i_z + usv_model_.n_dot_r);
  A *= discrete_dt_;
  A += Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(state_dim_, control_dim_);
  B(3, 0) = 1.0 / (usv_model_.mass + usv_model_.x_dot_u);
  B(3, 1) = 1.0 / (usv_model_.mass + usv_model_.x_dot_u);
  B(5, 0) = -usv_model_.D / (usv_model_.i_z + usv_model_.n_dot_r);
  B(5, 1) = usv_model_.D / (usv_model_.i_z + usv_model_.n_dot_r);
  B *= usv_model_.max_force * discrete_dt_;

  // 2) solving the discrete iterative Riccati Equation
  bool is_solve_success = false;
  Eigen::MatrixXd P = Q_, P_next;
  for (int i = 0; i < max_iterations_; ++i)
  {
    P_next = Q_ + A.transpose() * P * A -
             A.transpose() * P * B * (R_ + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    if ((P_next - P).norm() < solve_eps_)
    {
      is_solve_success = true;
      break;
    }
    P = P_next;
  }

  if (!is_solve_success)
  {
    R_WARN << "LQR controller (Otter) solved failed.";
    return false;
  }

  // 3) calculate feedback gain
  K = -(R_ + B.transpose() * P_next * B).inverse() * B.transpose() * P_next * A;
  return true;
}

}  // namespace controller
}  // namespace rmp
