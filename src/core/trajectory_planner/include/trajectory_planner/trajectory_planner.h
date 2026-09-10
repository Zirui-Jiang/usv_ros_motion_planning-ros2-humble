/**
 * *********************************************************
 *
 * @file: trajectory_planner.h
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
#ifndef RMP_TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_H_
#define RMP_TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_H_

#include <memory>
#include <tf2/utils.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "common/geometry/point.h"
#include "common/structure/usv_model.h"
#include "trajectory_planner/trajectory.h"

namespace rmp
{
namespace trajectory_planner
{
/**
 * @brief Base class for trajectory planning, providing functionality to generate and manage trajectories
 */
class TrajectoryPlanner
{
public:
  /**
   * @brief Default constructor for the trajectory planner
   */
  TrajectoryPlanner();

  /**
   * @brief Constructor for the trajectory planner with a specified trajectory time interval
   * @param traj_dt    the time interval for trajectory points
   * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
   * @param costmap_ros a pointer to the costmap used for collision checking and environmental awareness
   * @param tf         a pointer to the TF2 buffer used for managing transformations
   */
  TrajectoryPlanner(double traj_dt, rmp::common::structure::USV_MODEL usv_model, nav2_costmap_2d::Costmap2DROS* costmap_ros,
                    tf2_ros::Buffer* tf);

  /**
   * @brief Default destructor for the trajectory planner
   */
  ~TrajectoryPlanner() = default;

  /**
   * @brief interface to plan a trajectory based on a given path and state
   * @param path the input path represented as a series of 3D points
   * @param state the current state of the USV (Unmanned Surface Vehicle)
   * @param traj the resulting planned trajectory to be populated
   * @return true if the trajectory planning is successful, false otherwise
   */
  virtual bool plan(const rmp::common::geometry::Points3d& path, const rmp::common::structure::USV_STATE& state,
                    Trajectory& traj) = 0;

protected:
  double traj_dt_;                                             // trajectory time interval
  rmp::common::structure::USV_MODEL usv_model_;                // The model of the USV
  nav2_costmap_2d::Costmap2DROS* costmap_ros_;                      // Pointer to the costmap ROS wrapper
  tf2_ros::Buffer* tf_;                                        // Pointer to the transformation buffer
};
}  // namespace trajectory_planner
}  // namespace rmp

#endif