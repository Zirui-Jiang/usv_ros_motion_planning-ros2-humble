/**
 * *********************************************************
 *
 * @file: mapping.h
 * @brief: map server for USV perceprtion
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
#ifndef RMP_PERCEPTION_MAPPING_MAPPING_H
#define RMP_PERCEPTION_MAPPING_MAPPING_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rmp
{
namespace perception
{
class MapServer
{
public:
  /**
   * @brief Constructor for the MapServer class
   */
  MapServer(rclcpp::Node::SharedPtr node);
  /**
   * @brief Destroy the Global MapServer object
   */
  ~MapServer() = default;
  /**
   * @brief Publishes the current map
   */
  void publishMap();
  /**
   * @brief Updates the odometry transformation for accurate mapping and localization.
   * @param tf_buffer Pointer to the transform buffer containing transformation data
   */
  void updateOdom(tf2_ros::Buffer* tf_buffer);
  /**
   * @brief Generates occupancy grid map from laser scan data
   */
  void generateMap();

private:
  /**
   * @brief Callback handler for laser scan messages
   * @param scan_msg Pointer to the constant laser scan message
   */
  void _onScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

private:
  rclcpp::Node::SharedPtr node_;                     // ROS node
  sensor_msgs::msg::LaserScan scan_;                 // Laser scan data received from a laser sensor
  nav_msgs::msg::OccupancyGrid map_;                 // Occupancy grid map representing the environment
  geometry_msgs::msg::TransformStamped odom_transform_;  // Transformation data from the odometry frame to lidar frame
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;    // ROS publisher for publishing the occupancy grid map
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // ROS subscriber for subscribing to laser scan data
  int width_;                                        // Width of the occupancy grid map
  float resolution_;                                 // Resolution of the occupancy grid map (meters per cell)
  float offset_x_;                                   // X-axis offset of the map in meters
  float offset_y_;                                   // Y-axis offset of the map in meters
  float range_min_;                                  // Minimum range value for the laser scan
  std::string robot_namespace_;                      // Namespace of the robot, used for namespacing topics and services
};
}  // namespace perception
}  // namespace rmp
#endif
