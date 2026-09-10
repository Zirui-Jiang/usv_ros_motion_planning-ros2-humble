/**
 * *********************************************************
 *
 * @file: los_trajectory_planner.h
 * @brief: Line of sight (LOS) trajectory planner
 * @author: Yang Haodong
 * @date: 2025-1-2
 * @version: 2.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_TRAJECTORY_PLANNER_LOS_TRAJECTORY_PLANNER_H
#define RMP_TRAJECTORY_PLANNER_LOS_TRAJECTORY_PLANNER_H

#include "trajectory_planner/trajectory_planner.h"

namespace rmp
{
namespace trajectory_planner
{
class LosTrajectoryPlanner : public TrajectoryPlanner
{
public:
  /**
   * @brief Constructor for the LosTrajectoryPlanner class
   * @param traj_dt            the time interval for trajectory points
   * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
   * @param costmap_ros a pointer to the costmap used for collision checking and environmental awareness
   * @param tf         a pointer to the TF2 buffer used for managing transformations
   * @param traj_time          the total time for the planned trajectory
   * @param switch_dist        the distance at which the planner switches to next waypoint
   * @param goal_tolerance     the tolerance for reaching the goal position
   * @param lookahead_distance the distance ahead of the current position used for LOS guidance
   * @param yaw_rate_max       the maximum allowable yaw rate
   * @param kp_yaw             the proportional control gain for yaw
   */
  LosTrajectoryPlanner(double traj_dt, rmp::common::structure::USV_MODEL usv_model,
                       nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf, double traj_time, double switch_dist,
                       double goal_tolerance, double lookahead_distance, double yaw_rate_max, double kp_yaw);

  /**
   * @brief Default destructor for the LosTrajectoryPlanner
   */
  ~LosTrajectoryPlanner() = default;

  /**
   * @brief plan a trajectory based on a given path and state
   * @param path the input path represented as a series of 3D points
   * @param state the current state of the USV (Unmanned Surface Vehicle)
   * @param traj the resulting planned trajectory to be populated
   * @return true if the trajectory planning is successful, false otherwise
   */
  bool plan(const rmp::common::geometry::Points3d& path, const rmp::common::structure::USV_STATE& state,
            Trajectory& traj);

private:
  double traj_time_;           // trajectory time
  double switch_dist_;         // distance threshold to switch to next waypoint
  double goal_tolerance_;      // distance lower than goal_tolerance can be regarded as reaching goal
  double lookahead_distance_;  // lookahead distance
  double yaw_rate_max_;        // the maximum allowable yaw rate
  double kp_yaw_;              // the proportional control gain for yaw
};
}  // namespace trajectory_planner
}  // namespace rmp
#endif