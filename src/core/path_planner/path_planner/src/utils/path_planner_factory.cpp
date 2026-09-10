/**
 * *********************************************************
 *
 * @file: path_planner_factory.cpp
 * @brief: Create the planner with specifical parameters
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
#include "common/util/log.h"
#include "path_planner/utils/path_planner_factory.h"

// graph-based planner
#include "path_planner/path_planner_core.h"
#include "path_planner/graph_planner/astar_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Create and configure planner
 * @param nh ROS node
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool PathPlannerFactory::createPlanner(rclcpp::Node::SharedPtr nh, nav2_costmap_2d::Costmap2DROS* costmap_ros,
                                       PlannerProps& planner_props)
{
  std::string planner_name = nh->declare_parameter<std::string>("planner_name", "astar");  // planner name
  double obstacle_factor = nh->declare_parameter<double>("obstacle_factor", 0.5);          // obstacle factor

  if (planner_name == "astar")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "dijkstra")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor, true);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "gbfs")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor, false, true);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else
  {
    R_ERROR << "Unknown planner name: " << planner_name;
    return false;
  }

  R_INFO << "Using path planner: " << planner_name;
  return true;
}
}  // namespace path_planner
}  // namespace rmp
