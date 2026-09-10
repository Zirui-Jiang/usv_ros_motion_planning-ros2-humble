/**
 * *********************************************************
 *
 * @file: otter_pid_controller.cpp
 * @brief: Contains the pid controller for Otter USV
 * @author: Yang Haodong
 * @date: 2025-04-08
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
#include "controller/pid_controller/otter_pid_controller.h"

using namespace rmp::common::util;
using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::trajectory_planner;

namespace rmp
{
namespace controller
{
/**
 * @brief Construct a new PIDControllerOtter object
 */
PIDControllerOtter::PIDControllerOtter() : Controller()
{
}
/**
 * @brief Construct a new PIDControllerOtter object with specific parameters
 * @param control_frequency   the control frequency in Hz
 * @param usv_model           the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros         a pointer to the costmap for collision checking and environmental awareness
 * @param tf                  a pointer to the TF2 buffer for managing transformations
 * @param goal_dist_tolerance the tolerance for the goal distance
 * @param lookahead_time      the lookahead time for trajectory prediction
 * @param k_p_speed           the proportional coefficient for speed control
 * @param k_i_speed           the integral coefficient for speed control
 * @param k_d_speed           the derivative coefficient for speed control
 * @param k_p_course          the proportional coefficient for course control
 * @param k_i_course          the integral coefficient for course control
 * @param k_d_course          the derivative coefficient for course control
 */
PIDControllerOtter::PIDControllerOtter(double control_frequency, rmp::common::structure::USV_MODEL usv_model,
                                       nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                                       double goal_dist_tolerance, double lookahead_time, double k_p_speed,
                                       double k_i_speed, double k_d_speed, double k_p_course, double k_i_course,
                                       double k_d_course)
  : Controller(control_frequency, usv_model, costmap_ros, tf)
  , goal_dist_tolerance_(goal_dist_tolerance)
  , lookahead_time_(lookahead_time)
  , k_p_speed_(k_p_speed)
  , k_i_speed_(k_i_speed)
  , k_d_speed_(k_d_speed)
  , k_p_course_(k_p_course)
  , k_i_course_(k_i_course)
  , k_d_course_(k_d_course)
{
  initialized_ = true;
  allocation_matrix_ << 1.0, 1.0, -usv_model.D, usv_model.D;
  allocation_matrix_inv_ = allocation_matrix_.inverse();

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("controller_pid_otter");
  lookahead_point_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 1);
}

/**
 * @brief Computes the control command for the vehicle
 * @param ctrl_cmd the control command to be populated
 * @return true if the control command is successfully computed, false otherwise
 */
bool PIDControllerOtter::computeControlCommand(ControlCommand& ctrl_cmd)
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
  Eigen::Vector2d tau;
  tau << _calculateSurgeForce(desired_pt.velocity), _calculateYawMoment(desired_pt.theta);
  Eigen::Vector2d thruster = allocation_matrix_inv_ * tau;
  ctrl_cmd.setThrustCmdLeft(thruster[0]);
  ctrl_cmd.setThrustCmdRight(thruster[1]);
  regularControlCommand(ctrl_cmd);

  // visualization
  const auto& visualizer = VisualizerPtr::Instance();
  Point3d vis_pose(desired_pt.x, desired_pt.y, desired_pt.theta);
  visualizer->publishPoses(vis_pose, lookahead_point_pub_, usv_model_.ns + "/map", usv_model_.ns + "_lookahead_point",
                           Visualizer::DARK_BLUE, 3.0, 0.5, 0.5);

  return true;
}

/**
 * @brief Calculates the surge force based on the desired speed
 * @param desired_speed the desired speed
 * @return the calculated surge force
 */
double PIDControllerOtter::_calculateSurgeForce(double desired_speed)
{
  // Translate the global odometry velocity to body coordinate system for actual tracking.
  const double psi = usv_state_.psi;
  Eigen::Matrix2d R;
  Eigen::Vector2d vel_n, vel_b;
  R << std::cos(psi), -std::sin(psi), std::sin(psi), std::cos(psi);
  vel_n << usv_state_.x_dot, usv_state_.y_dot;
  vel_b = R.transpose() * vel_n;

  // PID process
  speed_error_ = desired_speed - vel_b.norm();
  double p_term = k_p_speed_ * speed_error_;
  i_speed_term_ += k_i_speed_ * speed_error_ * control_dt_;
  d_speed_term_ = k_d_speed_ * (speed_error_ - last_speed_error_) / control_dt_;
  last_speed_error_ = speed_error_;
  return p_term + i_speed_term_ + d_speed_term_;
}

/**
 * @brief Calculates the yaw moment based on the desired heading
 * @param desired_heading the desired heading in radians
 * @return the calculated yaw moment
 */
double PIDControllerOtter::_calculateYawMoment(double desired_heading)
{
  course_error_ = pi2pi(desired_heading - usv_state_.psi);
  double p_term = k_p_course_ * course_error_;
  i_course_term_ += k_i_course_ * course_error_ * control_dt_;
  d_course_term_ = k_d_course_ * (course_error_ - last_course_error_) / control_dt_;
  last_course_error_ = course_error_;
  return pi2pi(p_term + i_course_term_ + d_course_term_);
}

}  // namespace controller
}  // namespace rmp
