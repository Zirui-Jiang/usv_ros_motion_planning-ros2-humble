/**
 * *********************************************************
 *
 * @file: otter_pid_controller.h
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
#ifndef RMP_CONTROLLER_OTTER_PID_CONTROLLER_H_
#define RMP_CONTROLLER_OTTER_PID_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>

#include "common/geometry/point.h"
#include "controller/controller.h"

namespace rmp
{
namespace controller
{
class PIDControllerOtter : public Controller
{
public:
  /**
   * @brief Construct a new PIDControllerOtter object
   */
  PIDControllerOtter();
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
  PIDControllerOtter(double control_frequency, rmp::common::structure::USV_MODEL usv_model,
                     nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf, double goal_dist_tolerance,
                     double lookahead_time, double k_p_speed, double k_i_speed, double k_d_speed, double k_p_course,
                     double k_i_course, double k_d_course);

  /**
   * @brief Destroy the PIDControllerOtter object
   */
  ~PIDControllerOtter() = default;

  /**
   * @brief Computes the control command for the vehicle
   * @param ctrl_cmd the control command to be populated
   * @return true if the control command is successfully computed, false otherwise
   */
  bool computeControlCommand(ControlCommand& ctrl_cmd);

private:
  /**
   * @brief Calculates the surge force based on the desired speed
   * @param desired_speed the desired speed
   * @return the calculated surge force
   */
  double _calculateSurgeForce(double desired_speed);
  /**
   * @brief Calculates the yaw moment based on the desired heading
   * @param desired_heading the desired heading in radians
   * @return the calculated yaw moment
   */
  double _calculateYawMoment(double desired_heading);

private:
  double goal_dist_tolerance_;                   // Tolerance for the goal distance
  double lookahead_time_;                        // lookahead time gain
  double k_p_speed_, k_i_speed_, k_d_speed_;     // speed error pid coefficient
  double k_p_course_, k_i_course_, k_d_course_;  // course error pid coefficient

  double speed_error_{ 0.0f }, last_speed_error_{ 0.0f };
  double course_error_{ 0.0f }, last_course_error_{ 0.0f };
  double i_speed_term_{ 0.0f }, d_speed_term_{ 0.0f };
  double i_course_term_{ 0.0f }, d_course_term_{ 0.0f };

  Eigen::Matrix2d allocation_matrix_;      // Allocation matrix for thrust distribution
  Eigen::Matrix2d allocation_matrix_inv_;  // Inverse of the allocation matrix
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_pub_;  // lookahead point publisher
};
}  // namespace controller
}  // namespace rmp

#endif