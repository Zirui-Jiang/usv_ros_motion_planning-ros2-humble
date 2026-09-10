/**
 * *********************************************************
 *
 * @file: trajectory_planner.cpp
 * @brief: Trajectory planner abstract class
 * @author: Yang Haodong
 * @date: 2025-4-11
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>

#include "common/math/math_helper.h"
#include "trajectory_planner/trajectory_planner.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::common::structure;

namespace rmp
{
namespace trajectory_planner
{
/**
 * @brief Default constructor for the trajectory planner
 */
TrajectoryPlanner::TrajectoryPlanner() : traj_dt_(0.0), usv_model_({}), costmap_ros_(nullptr), tf_(nullptr)
{
}

/**
 * @brief Constructor for the trajectory planner with a specified trajectory time interval
 * @param traj_dt    the time interval for trajectory points
 * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros a pointer to the costmap used for collision checking and environmental awareness
 * @param tf         a pointer to the TF2 buffer used for managing transformations
 */
TrajectoryPlanner::TrajectoryPlanner(double traj_dt, USV_MODEL usv_model, nav2_costmap_2d::Costmap2DROS* costmap_ros,
                                     tf2_ros::Buffer* tf)
  : traj_dt_(traj_dt), usv_model_(usv_model), costmap_ros_(costmap_ros), tf_(tf)
{
}
}  // namespace trajectory_planner
}  // namespace rmp
