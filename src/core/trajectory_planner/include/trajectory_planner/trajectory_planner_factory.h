/**
 * *********************************************************
 *
 * @file: trajectory_planner_factory.h
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
#ifndef RMP_PATH_PLANNER_UTILS_TRAJECTORY_PLANNER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_TRAJECTORY_PLANNER_FACTORY_H_

#include <rclcpp/rclcpp.hpp>

#include "trajectory_planner/trajectory_planner.h"

namespace rmp
{
namespace trajectory_planner
{
class TrajectoryPlannerFactory
{
public:
  struct TrajectoryPlannerProps
  {
    std::unique_ptr<TrajectoryPlanner> trajectory_planner_ptr;  // trajectory planner
  };

public:
  /**
   * @brief Create and configure trajectory planner
   * @param nh ROS node
   * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
   * @param costmap_ros costmap ROS wrapper
   * @param planner_props planner property
   * @return bool true if create successful, else false
   */
  static bool createTrajectoryPlanner(rclcpp::Node::SharedPtr nh, rmp::common::structure::USV_MODEL usv_model,
                                      nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                                      TrajectoryPlannerProps& trajectoty_planner_props);
};
}  // namespace trajectory_planner
}  // namespace rmp
#endif
