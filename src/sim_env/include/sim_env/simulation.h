/**
 * *********************************************************
 *
 * @file: simulation.h
 * @brief: USV PNC simulation module
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
#ifndef RMP_SIM_ENV_SIMULATION_H
#define RMP_SIM_ENV_SIMULATION_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "path_planner/path_planner_core.h"
#include "controller/controller.h"
#include "trajectory_planner/trajectory.h"
#include "trajectory_planner/trajectory_planner.h"

namespace rmp
{
class Simulator
{
public:
  Simulator(rclcpp::Node::SharedPtr node, tf2_ros::Buffer* tf, nav2_costmap_2d::Costmap2DROS* costmap_ros);
  ~Simulator() = default;

  void onGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  void onOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void onControlCallBack();

private:
  bool _isCollision(double world_x, double world_y);

private:
  enum FSM_STATE
  {
    NAVIGATING,
    PENDING,
  };

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer* tf_;
  nav2_costmap_2d::Costmap2DROS* costmap_ros_;

  // decision
  FSM_STATE cur_fsm_state_;

  // path planner
  rmp::common::geometry::Points3d global_path_;
  std::unique_ptr<path_planner::PathPlannerCore> path_planner_;

  // trajectory planner
  trajectory_planner::Trajectory local_traj_;
  std::unique_ptr<trajectory_planner::TrajectoryPlanner> traj_planner_;

  // controller
  std::unique_ptr<controller::Controller> controller_;

  // ROS related
  nav_msgs::msg::Odometry odom_;
  std::string robot_namespace_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_thrust_cmd_pub_, right_thrust_cmd_pub_,
      lateral_thrust_cmd_pub_;

  double path_prune_dist_;
};
}  // namespace rmp

#endif
