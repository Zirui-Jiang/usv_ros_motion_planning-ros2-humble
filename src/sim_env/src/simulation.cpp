/**
 * *********************************************************
 *
 * @file: simulation.cpp
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
#include <tf2/utils.h>

#include <Eigen/Dense>

#include "sim_env/simulation.h"
#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "controller/controller_factory.h"
#include "trajectory_planner/trajectory_planner_factory.h"

namespace rmp
{
Simulator::Simulator(rclcpp::Node::SharedPtr node, tf2_ros::Buffer* tf, nav2_costmap_2d::Costmap2DROS* costmap_ros)
  : node_(node), tf_(tf), costmap_ros_(costmap_ros)
{
  robot_namespace_ = node_->get_namespace();
  if (!robot_namespace_.empty() && robot_namespace_[0] == '/')
  {
    robot_namespace_.erase(0, 1);
  }

  // robot related parameters
  common::structure::USV_MODEL usv_model;
  std::string usv_type;
  usv_model.ns = robot_namespace_;
  usv_model.name = node_->declare_parameter<std::string>("dynamics.name", "none");
  usv_type = node_->declare_parameter<std::string>("dynamics.type", "Differential");
  if (usv_type == "Differential")
  {
    usv_model.type = common::structure::USV_TYPE::Differential;
  }
  else if (usv_type == "Omni")
  {
    usv_model.type = common::structure::USV_TYPE::Omni;
  }
  else
  {
    R_ERROR << "Invalid USV type " << usv_type << ". Allowed USV type [Differential | Omni]";
  }
  usv_model.cruising_speed = node_->declare_parameter<double>("dynamics.cruising_speed", 0.0);
  usv_model.max_force = node_->declare_parameter<double>("dynamics.max_force", 0.0);
  usv_model.mass = node_->declare_parameter<double>("dynamics.mass", 0.0);
  usv_model.i_z = node_->declare_parameter<double>("dynamics.i_z", 0.0);
  usv_model.L = node_->declare_parameter<double>("dynamics.L", 0.0);
  usv_model.W = node_->declare_parameter<double>("dynamics.W", 0.0);
  usv_model.D = node_->declare_parameter<double>("dynamics.D", 0.0);
  usv_model.velocity_min = node_->declare_parameter<double>("dynamics.velocity_min", 0.0);
  usv_model.velocity_max = node_->declare_parameter<double>("dynamics.velocity_max", 0.0);
  usv_model.acceleration_min = node_->declare_parameter<double>("dynamics.acceleration_min", 0.0);
  usv_model.acceleration_max = node_->declare_parameter<double>("dynamics.acceleration_max", 0.0);
  usv_model.jerk_min = node_->declare_parameter<double>("dynamics.jerk_min", 0.0);
  usv_model.jerk_max = node_->declare_parameter<double>("dynamics.jerk_max", 0.0);
  usv_model.x_dot_u = node_->declare_parameter<double>("dynamics.x_dot_u", 0.0);
  usv_model.x_dot_v = node_->declare_parameter<double>("dynamics.x_dot_v", 0.0);
  usv_model.x_dot_r = node_->declare_parameter<double>("dynamics.x_dot_r", 0.0);
  usv_model.y_dot_u = node_->declare_parameter<double>("dynamics.y_dot_u", 0.0);
  usv_model.y_dot_v = node_->declare_parameter<double>("dynamics.y_dot_v", 0.0);
  usv_model.y_dot_r = node_->declare_parameter<double>("dynamics.y_dot_r", 0.0);
  usv_model.n_dot_u = node_->declare_parameter<double>("dynamics.n_dot_u", 0.0);
  usv_model.n_dot_v = node_->declare_parameter<double>("dynamics.n_dot_v", 0.0);
  usv_model.n_dot_r = node_->declare_parameter<double>("dynamics.n_dot_r", 0.0);
  usv_model.x_u = node_->declare_parameter<double>("dynamics.x_u", 0.0);
  usv_model.x_v = node_->declare_parameter<double>("dynamics.x_v", 0.0);
  usv_model.x_r = node_->declare_parameter<double>("dynamics.x_r", 0.0);
  usv_model.y_u = node_->declare_parameter<double>("dynamics.y_u", 0.0);
  usv_model.y_v = node_->declare_parameter<double>("dynamics.y_v", 0.0);
  usv_model.y_r = node_->declare_parameter<double>("dynamics.y_r", 0.0);
  usv_model.n_u = node_->declare_parameter<double>("dynamics.n_u", 0.0);
  usv_model.n_v = node_->declare_parameter<double>("dynamics.n_v", 0.0);
  usv_model.n_r = node_->declare_parameter<double>("dynamics.n_r", 0.0);
  usv_model.x_u_u = node_->declare_parameter<double>("dynamics.x_u_u", 0.0);
  usv_model.y_v_v = node_->declare_parameter<double>("dynamics.y_v_v", 0.0);
  usv_model.n_r_r = node_->declare_parameter<double>("dynamics.n_r_r", 0.0);

  // path planner
  path_planner_ = std::make_unique<path_planner::PathPlannerCore>(node_, "path_planner", costmap_ros);

  // trajectory planner
  trajectory_planner::TrajectoryPlannerFactory::TrajectoryPlannerProps trajectory_planner_props;
  if (!trajectory_planner::TrajectoryPlannerFactory::createTrajectoryPlanner(node_, usv_model, costmap_ros, tf,
                                                                             trajectory_planner_props))
  {
    R_ERROR << "Create trajectory planner failed.";
  }
  traj_planner_ = std::move(trajectory_planner_props.trajectory_planner_ptr);

  // controller
  controller::ControllerFactory::ControllerProps controller_props;
  if (!controller::ControllerFactory::createController(node_, usv_model, costmap_ros, tf, controller_props))
  {
    R_ERROR << "Create controller failed.";
  }
  controller_ = std::move(controller_props.controller_ptr);
  controller_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / controller_->control_frequency()),
      std::bind(&Simulator::onControlCallBack, this));

  // start FSM
  cur_fsm_state_ = FSM_STATE::PENDING;

  // ROS related
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 1, std::bind(&Simulator::onGoalCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&Simulator::onOdomCallback, this, std::placeholders::_1));
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  traj_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_trajectory", 1);
  debug_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("debug", 1);
  left_thrust_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>("thrusters/left_thrust_cmd", 1);
  lateral_thrust_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>("thrusters/lateral_thrust_cmd", 1);
  right_thrust_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>("thrusters/right_thrust_cmd", 1);
}

void Simulator::onGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  // path search
  goal_pose_ = *goal;
  goal_pose_.header.frame_id = robot_namespace_ + "/map";
  if (goal_pose_.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    R_ERROR << "The goal pose passed to this planner must be in the " << costmap_ros_->getGlobalFrameID()
            << " frame. It is instead in the " << goal_pose_.header.frame_id << " frame.";
    return;
  }
  geometry_msgs::msg::PoseStamped start_pose;
  costmap_ros_->getRobotPose(start_pose);
  common::geometry::Point3d start_point(start_pose.pose.position.x, start_pose.pose.position.y,
                                        tf2::getYaw(start_pose.pose.orientation));
  common::geometry::Point3d goal_point(goal_pose_.pose.position.x, goal_pose_.pose.position.y,
                                       tf2::getYaw(goal_pose_.pose.orientation));
  path_planner_->makePlan(start_point, goal_point, global_path_);

  // trajectory generation
  if (!traj_planner_->plan(global_path_, { start_point.x(), start_point.y(), start_point.theta() }, local_traj_))
  {
    R_ERROR << "Trajectory planning failed!";
    return;
  }

  // controller prepare
  controller_->setGlobalPlan(global_path_);
  controller_->setLocalPlan(local_traj_);

  cur_fsm_state_ = FSM_STATE::NAVIGATING;
}

void Simulator::onOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  odom_ = *odom;
  if (cur_fsm_state_ == FSM_STATE::PENDING)
  {
    return;
  }

  // update USV state
  common::structure::USV_STATE usv_state;
  usv_state.x = odom_.pose.pose.position.x;
  usv_state.y = odom_.pose.pose.position.y;
  usv_state.psi = tf2::getYaw(odom_.pose.pose.orientation);
  usv_state.x_dot = odom_.twist.twist.linear.x;
  usv_state.y_dot = odom_.twist.twist.linear.y;
  usv_state.psi_dot = odom_.twist.twist.angular.z;

  geometry_msgs::msg::PoseStamped start_pose;
  costmap_ros_->getRobotPose(start_pose);
  common::geometry::Point3d start_point(start_pose.pose.position.x, start_pose.pose.position.y,
                                        tf2::getYaw(start_pose.pose.orientation));
  common::geometry::Point3d goal_point(goal_pose_.pose.position.x, goal_pose_.pose.position.y,
                                       tf2::getYaw(goal_pose_.pose.orientation));

  if (global_path_.empty())
  {
    R_WARN << "No global path found!";
    return;
  }

  if (local_traj_.empty())
  {
    R_WARN << "No local trajectory found!";
    return;
  }

  // collision detection
  for (const auto& traj_pt : local_traj_)
  {
    if (_isCollision(traj_pt.x, traj_pt.y))
    {
      // path replan
      path_planner_->makePlan(start_point, goal_point, global_path_);
      break;
    }
  }

  // prune
  path_planner::PathPlannerCore::prune(start_point, global_path_);

  // trajectory generation
  if (!traj_planner_->plan(global_path_, usv_state, local_traj_))
  {
    R_WARN << "Trajectory planning failed, using the old plan.";
  }

  // controller prepare
  controller_->setGlobalPlan(global_path_);
  controller_->setLocalPlan(local_traj_);
  controller_->setState(usv_state);

  // visualization
  const auto& visualizer = common::util::VisualizerPtr::Instance();
  common::geometry::Points3d traj_position;
  for (const auto& traj_pt : local_traj_)
  {
    traj_position.emplace_back(traj_pt.x, traj_pt.y, traj_pt.theta);
  }
  visualizer->publishPlan(traj_position, traj_pub_, robot_namespace_ + "/map");
  visualizer->publishPlan(global_path_, path_pub_, robot_namespace_ + "/map");
}

void Simulator::onControlCallBack()
{
  if (local_traj_.size() < 2 || cur_fsm_state_ == FSM_STATE::PENDING)
  {
    return;
  }

  if (cur_fsm_state_ != FSM_STATE::PENDING && controller_->isGoalReached())
  {
    cur_fsm_state_ = FSM_STATE::PENDING;
  }

  controller::ControlCommand cmd_vel;
  controller_->computeControlCommand(cmd_vel);
  std_msgs::msg::Float32 thrust_cmd_left, thrust_cmd_lateral, thrust_cmd_right;
  thrust_cmd_left.data = static_cast<float>(cmd_vel.thrust_cmd_left());
  thrust_cmd_lateral.data = static_cast<float>(cmd_vel.thrust_cmd_lateral());
  thrust_cmd_right.data = static_cast<float>(cmd_vel.thrust_cmd_right());
  left_thrust_cmd_pub_->publish(thrust_cmd_left);
  lateral_thrust_cmd_pub_->publish(thrust_cmd_lateral);
  right_thrust_cmd_pub_->publish(thrust_cmd_right);
}

bool Simulator::_isCollision(double world_x, double world_y)
{
  unsigned int mx, my;
  costmap_ros_->getCostmap()->worldToMap(world_x, world_y, mx, my);
  const int id = mx + costmap_ros_->getCostmap()->getSizeInCellsX() * my;
  return costmap_ros_->getCostmap()->getCharMap()[id] >= nav2_costmap_2d::LETHAL_OBSTACLE * 0.9;
}
}  // namespace rmp
