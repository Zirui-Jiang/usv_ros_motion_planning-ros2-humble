/**
 * *********************************************************
 *
 * @file: mapping_node.cpp
 * @brief: map server node for USV perceprtion
 * @author: Yang Haodong
 * @date: 2025-1-5
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "perception/mapping/mapping.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("perception_mapping");
  rmp::perception::MapServer map_server(node);

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  // spin_thread = true: the listener keeps the TF buffer fresh on its own
  // thread, so updateOdom() can look up the transform at the scan timestamp
  // (waiting briefly if TF trails the scan) without blocking this loop.
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

  // waits for the first odometry transformation to become available
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(node->get_logger(), "Waiting for Odometry.");
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok())
  {
    // Process incoming scans/TF first so the map is built from the freshest
    // data, then look up the robot pose at that scan's own timestamp so the map
    // stays aligned with the live scan while the boat rotates.
    rclcpp::spin_some(node);
    map_server.updateOdom(tf_buffer.get());
    map_server.generateMap();
    map_server.publishMap();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
