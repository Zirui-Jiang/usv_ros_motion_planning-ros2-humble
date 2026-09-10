/**
 * *********************************************************
 *
 * @file: mapping.cpp
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
#include <algorithm>
#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/math/math_helper.h"
#include "common/util/log.h"
#include "perception/mapping/mapping.h"

using namespace rmp::common::math;

namespace rmp
{
namespace perception
{
/**
 * @brief Constructor for the MapServer class
 * @param node ROS node shared pointer
 */
MapServer::MapServer(rclcpp::Node::SharedPtr node) : node_(node)
{
  // parameters intialization
  robot_namespace_ = node_->get_namespace();
  if (!robot_namespace_.empty() && robot_namespace_[0] == '/')
  {
    robot_namespace_.erase(0, 1);
  }

  width_ = node_->declare_parameter<int>("width", 1024);
  resolution_ = static_cast<float>(node_->declare_parameter<double>("resolution", 0.5));
  offset_x_ = static_cast<float>(node_->declare_parameter<double>("offset_x", 0.0));
  offset_y_ = static_cast<float>(node_->declare_parameter<double>("offset_y", 0.0));

  // sets up publisher and subscriber for occupancy grid maps
  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&MapServer::_onScanCallback, this, std::placeholders::_1));
  range_min_ = 0.0;

  // Initializes an empty map with the specified parameters
  map_.info.resolution = resolution_;
  map_.info.width = width_;
  map_.info.height = width_;
  map_.info.origin.position.x = -static_cast<float>(width_) * resolution_ * 0.5f + offset_x_;
  map_.info.origin.position.y = -static_cast<float>(width_) * resolution_ * 0.5f + offset_y_;
  map_.info.origin.orientation.w = 1.0;
  map_.header.frame_id = robot_namespace_ + "/map";
  map_.data.clear();
  for (long int i = 0; i < width_ * width_; i++)
  {
    map_.data.push_back(-1);
  }
}

/**
 * @brief Publishes the current map
 */
void MapServer::publishMap()
{
  map_.header.stamp = node_->now();
  map_pub_->publish(map_);
}

/**
 * @brief Updates the odometry transformation for accurate mapping and localization.
 * @param tf_buffer Pointer to the transform buffer containing transformation data
 */
void MapServer::updateOdom(tf2_ros::Buffer* tf_buffer)
{
  const std::string target = robot_namespace_ + "/odom";
  const std::string source = robot_namespace_ + "/lidar_link";
  try
  {
    // Prefer the transform at the scan's own timestamp so the pose used to
    // project the scan is time-consistent with the scan itself (critical while
    // the boat rotates). TF is published a little behind the sensor, so wait
    // briefly for it to catch up; fall back to the latest transform otherwise.
    if (scan_.header.stamp.sec != 0 || scan_.header.stamp.nanosec != 0)
    {
      odom_transform_ = tf_buffer->lookupTransform(target, source, scan_.header.stamp,
                                                   rclcpp::Duration::from_seconds(0.1));
    }
    else
    {
      odom_transform_ = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);
    }
  }
  catch (tf2::TransformException& ex)
  {
    try
    {
      odom_transform_ = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex2)
    {
      R_DEBUG << "Odometry Not Found.";
    }
  }
}

/**
 * @brief Generates occupancy grid map from laser scan data
 */
void MapServer::generateMap()
{
  if (scan_.ranges.empty())
  {
    return;
  }

  // Pose of the lidar in the map frame. Under the fake-localization TF chain
  // map == odom (identity) and base_link -> lidar_link carries no rotation, so
  // the odom->lidar yaw is the boat heading and the horizontal scan plane maps
  // straight onto the grid.
  const double my_yaw = tf2::getYaw(odom_transform_.transform.rotation);
  const float tx = static_cast<float>(odom_transform_.transform.translation.x);
  const float ty = static_cast<float>(odom_transform_.transform.translation.y);

  const int index_max = width_ * width_ - 1;
  const int origin_x = width_ / 2 - static_cast<int>(offset_x_ / resolution_);
  const int origin_y = width_ / 2 - static_cast<int>(offset_y_ / resolution_);
  const int odom_x = origin_x + static_cast<int>(std::round(tx / resolution_));
  const int odom_y = origin_y + static_cast<int>(std::round(ty / resolution_));

  // Log-odds-style update: symmetric-ish weights (unlike the old +8 / -2) let
  // the map decay stale returns so it always tracks the live scan instead of
  // accumulating obstacles that no longer exist. A single hit already crosses
  // the occupied threshold (>50) so freshly-swept directions appear immediately
  // while the boat rotates; a couple of clear passes remove a stale cell.
  constexpr int increment_value = 55;  // occupied evidence added at a hit cell
  constexpr int decrement_value = 25;  // free evidence added along a clear ray

  const float range_max = scan_.range_max;
  const float range_min = std::max(range_min_, scan_.range_min);
  const float clear_range = std::min(range_max, 30.0f);

  float angle = static_cast<float>(my_yaw) + scan_.angle_min;
  for (std::size_t i = 0; i < scan_.ranges.size(); ++i, angle += scan_.angle_increment)
  {
    const float range = scan_.ranges[i];
    // A genuine obstacle return: finite and within [range_min, range_max).
    // Everything else (NaN, inf, max-range no-return) is treated as free space
    // to be cleared, never marked as an obstacle.
    const bool hit = std::isfinite(range) && range >= range_min && range < range_max;

    // Raytrace as free space up to the hit, or out to the clear range when the
    // beam saw nothing.
    const float trace_range = hit ? range : clear_range;
    const int end_x = odom_x + static_cast<int>(std::round((trace_range / resolution_) * std::cos(angle)));
    const int end_y = odom_y + static_cast<int>(std::round((trace_range / resolution_) * std::sin(angle)));

    // Bresenham from the robot cell towards the beam end, clearing EVERY free
    // cell along the way (no distance band -- that was what left stale marks).
    // The endpoint is excluded so a real hit is not cleared in the same pass.
    int x = odom_x;
    int y = odom_y;
    const int dx = std::abs(end_x - x);
    const int dy = std::abs(end_y - y);
    const int sx = (x < end_x) ? 1 : -1;
    const int sy = (y < end_y) ? 1 : -1;
    int err = dx - dy;
    while (x != end_x || y != end_y)
    {
      const int id = clamp(y * width_ + x, 0, index_max);
      map_.data[id] = std::max(map_.data[id] - decrement_value, 0);
      const int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }

    // Mark the hit cell as occupied evidence.
    if (hit)
    {
      const int id = clamp(end_y * width_ + end_x, 0, index_max);
      map_.data[id] = std::min(map_.data[id] + increment_value, 100);
    }
  }
}

/**
 * @brief Callback handler for laser scan messages
 * @param scan_msg Pointer to the constant laser scan message
 */
void MapServer::_onScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  scan_ = *scan_msg;
}
}  // namespace perception
}  // namespace rmp
