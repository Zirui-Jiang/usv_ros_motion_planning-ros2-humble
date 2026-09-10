/**
 * *********************************************************
 *
 * @file: trajectory_planner_factory.cpp
 * @brief: Create the trajectory planner with specifical parameters
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
#include "common/util/log.h"
#include "trajectory_planner/trajectory_planner_factory.h"
#include "trajectory_planner/los_trajectory_planner.h"

using namespace rmp::common::structure;

namespace rmp
{
namespace trajectory_planner
{
namespace
{
// Declare (or fetch) a double parameter with a default value.
double declareDouble(const rclcpp::Node::SharedPtr& nh, const std::string& name, double default_value)
{
  return nh->declare_parameter<double>(name, default_value);
}
}  // namespace

/**
 * @brief Create and configure trajectory planner
 * @param nh ROS node
 * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool TrajectoryPlannerFactory::createTrajectoryPlanner(rclcpp::Node::SharedPtr nh, USV_MODEL usv_model,
                                                       nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                                                       TrajectoryPlannerProps& trajectoty_planner_props)
{
  // trajectory planner related parameters
  const std::string planner_namespace = "usv_simulation.trajectory_planner.";
  std::string planner_name = nh->declare_parameter<std::string>(planner_namespace + "planner_name", "los");

  if (planner_name == "los")
  {
    double trajectory_dt, trajectory_time;
    double switch_dist, goal_tolerance, lookahead_distance, yaw_rate_max, kp_yaw;

    trajectory_dt = declareDouble(nh, planner_namespace + "los.trajectory_time_interval", 0.5);
    trajectory_time = declareDouble(nh, planner_namespace + "los.trajectory_time", 20.0);
    switch_dist = declareDouble(nh, planner_namespace + "los.switch_dist", 5.0);
    goal_tolerance = declareDouble(nh, planner_namespace + "los.goal_tolerance", 1.0);
    lookahead_distance = declareDouble(nh, planner_namespace + "los.lookahead_distance", 5.0);
    yaw_rate_max = declareDouble(nh, planner_namespace + "los.yaw_rate_max", 0.3);
    kp_yaw = declareDouble(nh, planner_namespace + "los.kp_yaw", 1.0);

    trajectoty_planner_props.trajectory_planner_ptr = std::make_unique<LosTrajectoryPlanner>(
        trajectory_dt, usv_model, costmap_ros, tf, trajectory_time, switch_dist, goal_tolerance, lookahead_distance,
        yaw_rate_max, kp_yaw);

    R_INFO << "Using trajectory planner: Line of Sight (LOS)";
  }
  else
  {
    R_ERROR << "Invalid trajectory planner name: " << planner_name;
  }

  return true;
}
}  // namespace trajectory_planner
}  // namespace rmp
