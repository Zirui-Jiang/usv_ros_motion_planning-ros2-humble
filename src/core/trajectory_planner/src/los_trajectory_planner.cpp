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
#include "common/math/math_helper.h"
#include "trajectory_planner/los_trajectory_planner.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::common::structure;

namespace rmp
{
namespace trajectory_planner
{
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
LosTrajectoryPlanner::LosTrajectoryPlanner(double traj_dt, USV_MODEL usv_model, nav2_costmap_2d::Costmap2DROS* costmap_ros,
                                           tf2_ros::Buffer* tf, double traj_time, double switch_dist,
                                           double goal_tolerance, double lookahead_distance, double yaw_rate_max,
                                           double kp_yaw)
  : TrajectoryPlanner(traj_dt, usv_model, costmap_ros, tf)
  , traj_time_(traj_time)
  , switch_dist_(switch_dist)
  , goal_tolerance_(goal_tolerance)
  , lookahead_distance_(lookahead_distance)
  , yaw_rate_max_(yaw_rate_max)
  , kp_yaw_(kp_yaw)
{
}

/**
 * @brief plan a trajectory based on a given path and state
 * @param path the input path represented as a series of 3D points
 * @param state the current state of the USV (Unmanned Surface Vehicle)
 * @param traj the resulting planned trajectory to be populated
 * @return true if the trajectory planning is successful, false otherwise
 */
bool LosTrajectoryPlanner::plan(const Points3d& path, const USV_STATE& state, Trajectory& traj)
{
  Points3d los_path;
  los_path.emplace_back(state.x, state.y, state.psi);

  int cur_index = 0;
  double scale_dt = 0.1;
  bool reach_goal = false;
  int N = static_cast<int>(traj_time_ / traj_dt_);
  int scale = static_cast<int>(traj_dt_ / scale_dt);

  Point3d cur_pose = los_path[0];
  for (int i = 0; i < N * scale; ++i)
  {
    double phi_err;
    const auto& cur_waypoint = path[cur_index];

    if (path.size() > 1)
    {
      const auto& next_waypoint = path[cur_index + 1];
      double dist = std::hypot(next_waypoint.x() - cur_pose.x(), next_waypoint.y() - cur_pose.y());
      if (dist < switch_dist_)
      {
        // final waypoint
        if (cur_index + 1 == path.size() - 1)
        {
          if (dist < goal_tolerance_)
          {
            reach_goal = true;
            break;
          }
          double phi_d = std::atan2(next_waypoint.y() - cur_pose.y(), next_waypoint.x() - cur_pose.x());
          phi_err = pi2pi(phi_d - cur_pose.theta());
        }
        else
        {
          double alpha_i = std::atan2(next_waypoint.y() - cur_waypoint.y(), next_waypoint.x() - cur_waypoint.x());
          double e_x = cur_pose.x() - cur_waypoint.x();
          double e_y = cur_pose.y() - cur_waypoint.y();
          double error_lat = -e_x * std::sin(alpha_i) + e_y * std::cos(alpha_i);
          double phi_d = pi2pi(alpha_i - std::atan2(error_lat, lookahead_distance_));
          phi_err = pi2pi(phi_d - cur_pose.theta());
          if (i % scale == 0)
          {
            ++cur_index;
          }
        }
      }
    }
    else if (path.size() == 1)
    {
      if (std::hypot(cur_waypoint.x() - cur_pose.x(), cur_waypoint.y() - cur_pose.y()) < goal_tolerance_)
      {
        reach_goal = true;
        break;
      }
      double phi_d = std::atan2(cur_waypoint.y() - cur_pose.y(), cur_waypoint.x() - cur_pose.x());
      phi_err = pi2pi(phi_d - cur_pose.theta());
    }
    else
    {
      R_ERROR << "There are no waypoints to generate trajectory.";
      return false;
    }

    // dynamic rollout
    double yaw_rate = clamp(kp_yaw_ * phi_err, yaw_rate_max_, -yaw_rate_max_);
    cur_pose.setTheta(pi2pi(cur_pose.theta() + yaw_rate * scale_dt));
    cur_pose.setX(cur_pose.x() + usv_model_.cruising_speed * cos(cur_pose.theta()) * scale_dt);
    cur_pose.setY(cur_pose.y() + usv_model_.cruising_speed * sin(cur_pose.theta()) * scale_dt);

    if (i % scale == 0)
    {
      los_path.push_back(cur_pose);
    }
  }

  // simple linear velocity profile along the LOS path
  double pre_dist = 0.0;
  traj.reset(los_path.size());
  traj.push({ 0.0, 0.0, los_path[0].x(), los_path[0].y(), los_path[0].theta() });
  double remain_dist = std::hypot(los_path[0].x() - path.back().x(), los_path[0].y() - path.back().y());
  for (size_t i = 1; i < los_path.size(); ++i)
  {
    const auto& pt = los_path[i];
    const auto& pt_prev = los_path[i - 1];
    pre_dist += std::hypot(pt.x() - pt_prev.x(), pt.y() - pt_prev.y());
    traj.push({ i * traj_dt_, pre_dist, pt.x(), pt.y(), pt.theta(), 0.0,
                usv_model_.cruising_speed * (1.0 - pre_dist / remain_dist), 0.0 });
  }
  return true;
}
}  // namespace trajectory_planner
}  // namespace rmp
