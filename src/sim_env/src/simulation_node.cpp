/**
 * *********************************************************
 *
 * @file: simulation_node.cpp
 * @brief: USV PNC simulation ROS node
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
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sim_env/simulation.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("usv_simulation");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  // environment
  // The launch file loads the costmap configuration on the `usv_simulation`
  // node with a `costmap.` prefix (see global_costmap_params_otter.yaml). The
  // Costmap2DROS wrapper is a standalone node, so it declares its parameters
  // without that prefix and would otherwise fall back to defaults
  // (global_frame=map, robot_base_frame=base_link) and never see the
  // namespaced frames. Forward the `costmap.` overrides from this node to the
  // costmap node so it reads the intended frame ids. We read the parameter
  // *overrides* (not declared parameters) because the launch file provides
  // them only as command-line overrides, which are not auto-declared.
  rclcpp::NodeOptions costmap_options;
  const std::string costmap_prefix = "costmap.";
  const auto& overrides =
    node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& entry : overrides)
  {
    const std::string& name = entry.first;
    if (name.compare(0, costmap_prefix.size(), costmap_prefix) == 0)
    {
      const std::string stripped = name.substr(costmap_prefix.size());
      costmap_options.append_parameter_override(stripped, entry.second);
    }
  }

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
  costmap_ros->on_configure(costmap_ros->get_current_state());
  costmap_ros->on_activate(costmap_ros->get_current_state());

  rmp::Simulator sim_env(node, tf_buffer.get(), costmap_ros.get());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
