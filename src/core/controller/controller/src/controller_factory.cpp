/**
 * *********************************************************
 *
 * @file: controller_factory.cpp
 * @brief: Create the controller with specifical parameters
 * @author: Yang Haodong
 * @date: 2025-02-16
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/structure/usv_model.h"
#include "common/util/log.h"
#include "controller/controller_factory.h"

// otter
#include "controller/pid_controller/otter_pid_controller.h"
#include "controller/lqr_controller/otter_lqr_controller.h"

using namespace rmp::common::structure;

namespace rmp
{
namespace controller
{
/**
 * @brief Create and configure planner
 * @param nh ROS node
 * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool ControllerFactory::createController(rclcpp::Node::SharedPtr nh, USV_MODEL usv_model,
                                         nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                                         ControllerProps& controller_props)
{
  // controller related parameters
  const std::string controller_namespace = "usv_simulation.controller.";
  double controller_frequncy = nh->declare_parameter<double>(controller_namespace + "controller_frequncy", 10.0);
  std::string controller_name = nh->declare_parameter<std::string>(controller_namespace + "controller_name", "pid");

  if (usv_model.name == "otter")
  {
    if (controller_name == "pid")
    {
      double goal_distance_tolerance, lookahead_time;
      double k_p_speed, k_i_speed, k_d_speed;
      double k_p_course, k_i_course, k_d_course;

      goal_distance_tolerance = nh->declare_parameter<double>(controller_namespace + "pid_controller.goal_distance_tolerance", 1.0);
      lookahead_time = nh->declare_parameter<double>(controller_namespace + "pid_controller.lookahead_time", 3.0);
      k_p_speed = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_p_speed", 0.0);
      k_i_speed = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_i_speed", 0.0);
      k_d_speed = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_d_speed", 0.0);
      k_p_course = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_p_course", 0.0);
      k_i_course = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_i_course", 0.0);
      k_d_course = nh->declare_parameter<double>(controller_namespace + "pid_controller.k_d_course", 0.0);

      controller_props.controller_ptr = std::make_unique<PIDControllerOtter>(
          controller_frequncy, usv_model, costmap_ros, tf, goal_distance_tolerance, lookahead_time, k_p_speed,
          k_i_speed, k_d_speed, k_p_course, k_i_course, k_d_course);

      R_INFO << "Using controller: PIDControllerOtter";
    }
    else if (controller_name == "lqr")
    {
      int max_iterations;
      double discrete_dt, solve_eps;
      double goal_distance_tolerance, lookahead_time;
      std::vector<double> Q_matrix_diag, R_matrix_diag;

      goal_distance_tolerance = nh->declare_parameter<double>(controller_namespace + "lqr_controller.goal_distance_tolerance", 1.0);
      lookahead_time = nh->declare_parameter<double>(controller_namespace + "lqr_controller.lookahead_time", 1.5);
      discrete_dt = nh->declare_parameter<double>(controller_namespace + "lqr_controller.discrete_dt", 0.1);
      solve_eps = nh->declare_parameter<double>(controller_namespace + "lqr_controller.solve_eps", 0.1);
      max_iterations = nh->declare_parameter<int>(controller_namespace + "lqr_controller.max_iterations", 1000);
      Q_matrix_diag = nh->declare_parameter<std::vector<double>>(controller_namespace + "lqr_controller.Q_matrix_diag",
                                                                 std::vector<double>());
      R_matrix_diag = nh->declare_parameter<std::vector<double>>(controller_namespace + "lqr_controller.R_matrix_diag",
                                                                 std::vector<double>());

      controller_props.controller_ptr = std::make_unique<LQRControllerOtter>(
          controller_frequncy, usv_model, costmap_ros, tf, goal_distance_tolerance, lookahead_time, max_iterations,
          discrete_dt, solve_eps, Q_matrix_diag, R_matrix_diag);

      R_INFO << "Using controller: LQRControllerOtter";
    }
    else
    {
      R_ERROR << "Invalid controller name: " << controller_name;
    }
  }
  else
  {
    R_ERROR << "Invalid USV model name: " << usv_model.name << ". Allowed type is [otter].";
  }

  return true;
}
}  // namespace controller
}  // namespace rmp
