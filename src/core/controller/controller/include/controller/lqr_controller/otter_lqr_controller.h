/**
 * *********************************************************
 *
 * @file: wamv_lqr_controller.h
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
#ifndef RMP_CONTROLLER_OTTER_LQR_CONTROLLER_H_
#define RMP_CONTROLLER_OTTER_LQR_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>

#include "common/geometry/point.h"
#include "controller/controller.h"

namespace rmp
{
namespace controller
{
class LQRControllerOtter : public Controller
{
public:
  /**
   * @brief Construct a new LQRControllerOtter object
   */
  LQRControllerOtter();

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
  LQRControllerOtter(double control_frequency, rmp::common::structure::USV_MODEL usv_model,
                    nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf, double goal_dist_tolerance,
                    double lookahead_time, int max_iterations, double discrete_dt, double solve_eps,
                    const std::vector<double>& Q_diag, std::vector<double>& R_diag);

  /**
   * @brief Destroy the LQRControllerOtter object
   */
  ~LQRControllerOtter() = default;

  /**
   * @brief Computes the control command for the vehicle
   * @param ctrl_cmd the control command to be populated
   * @return true if the control command is successfully computed, false otherwise
   */
  bool computeControlCommand(ControlCommand& ctrl_cmd);

private:
  /**
   * @brief Solves the LQR optimization problem to compute the feedback gain matrix
   * @param desired_pt the desired trajectory point
   * @param K          the resulting feedback gain matrix
   * @return true if the problem is successfully solved, false otherwise
   */
  bool _solveLQRProblem(const rmp::trajectory_planner::TrajectoryCell& desired_pt, Eigen::MatrixXd& K);

private:
  double goal_dist_tolerance_;          // Tolerance for the goal distance
  double lookahead_time_;               // Lookahead time for trajectory prediction
  int max_iterations_;                  // Maximum number of iterations for solving the LQR problem
  double discrete_dt_;                  // Time step for system discretization
  double solve_eps_;                    // Convergence threshold for solving the LQR problem
  Eigen::MatrixXd Q_;                   // State cost matrix for the LQR problem
  Eigen::MatrixXd R_;                   // Control cost matrix for the LQR problem
  static int state_dim_;                // Dimension of the system state
  static int control_dim_;              // Dimension of the control input
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_pub_;  // Publisher for the lookahead point used in visualization
};
}  // namespace controller
}  // namespace rmp

#endif