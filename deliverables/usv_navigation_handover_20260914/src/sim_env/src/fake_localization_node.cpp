/**
 * *********************************************************
 *
 * @file: fake_localization_node.cpp
 * @brief: Fake localization node providing absolute-precise localization.
 * @author: Yang Haodong
 * @date: 2026-08-08
 * @version: 2.0
 *
 * Copyright (c) 2026, Yang Haodong.
 * All rights reserved.
 *
 * ********************************************************
 */
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rmp
{
namespace common
{
namespace environment
{
class FakeLocalizationNode : public rclcpp::Node
{
public:
  FakeLocalizationNode() : rclcpp::Node("fake_localization"), tf_broadcaster_(*this)
  {
    model_name_ = declare_parameter<std::string>("model_name", "otter");

    // Frames default to the node-namespace-prefixed names used across the
    // stack (e.g. otter/map, otter/odom, otter/base_link).
    std::string ns = get_namespace();
    if (!ns.empty() && ns[0] == '/')
    {
      ns.erase(0, 1);
    }
    const std::string prefix = ns.empty() ? "" : ns + "/";
    map_frame_ = declare_parameter<std::string>("map_frame", prefix + "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", prefix + "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", prefix + "base_link");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 1,
        std::bind(&FakeLocalizationNode::modelStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "[fake_localization] model='%s' frames: %s -> %s -> %s",
                model_name_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  /**
   * @brief Publish the TF chain and odometry from the ground-truth model state.
   */
  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (msg->name.size() != msg->pose.size() || msg->name.size() != msg->twist.size())
    {
      RCLCPP_WARN(get_logger(), "Ignoring malformed ModelStates: name=%zu pose=%zu twist=%zu",
                  msg->name.size(), msg->pose.size(), msg->twist.size());
      return;
    }
    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] != model_name_)
      {
        continue;
      }
      const auto& pose = msg->pose[i];
      const auto& twist_world = msg->twist[i];

      const rclcpp::Time stamp = now();
      // Skip stale/duplicate stamps: keeps the TF stream strictly monotonic
      // even if the sim clock stalls between messages.
      // A world reset starts a new timestamp epoch; do not suppress all new
      // localization until the previous simulation time has been reached.
      if (have_last_ && stamp == last_stamp_)
      {
        return;
      }
      have_last_ = true;
      last_stamp_ = stamp;

      // map -> odom: static identity (fake localization aligns odom with map).
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = stamp;
      map_to_odom.header.frame_id = map_frame_;
      map_to_odom.child_frame_id = odom_frame_;
      map_to_odom.transform.rotation.w = 1.0;
      tf_broadcaster_.sendTransform(map_to_odom);

      // odom -> base_link: the ground-truth pose (planar).
      geometry_msgs::msg::TransformStamped odom_to_base;
      odom_to_base.header.stamp = stamp;
      odom_to_base.header.frame_id = odom_frame_;
      odom_to_base.child_frame_id = base_frame_;
      odom_to_base.transform.translation.x = pose.position.x;
      odom_to_base.transform.translation.y = pose.position.y;
      odom_to_base.transform.translation.z = 0.0;
      odom_to_base.transform.rotation = pose.orientation;
      tf_broadcaster_.sendTransform(odom_to_base);

      // Odometry: pose in odom (== map), twist rotated into the body frame.
      const double yaw = tf2::getYaw(pose.orientation);
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose = pose;
      odom.pose.pose.position.z = 0.0;
      // Gazebo's model_states twist is in the WORLD frame; nav_msgs/Odometry
      // twist must be expressed in the body (child) frame.
      odom.twist.twist.linear.x = (twist_world.linear.x * cos_yaw) + (twist_world.linear.y * sin_yaw);
      odom.twist.twist.linear.y = (-twist_world.linear.x * sin_yaw) + (twist_world.linear.y * cos_yaw);
      odom.twist.twist.angular.z = twist_world.angular.z;
      odom_pub_->publish(odom);
      return;
    }
  }

  std::string model_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool have_last_{ false };
  rclcpp::Time last_stamp_;
};
}  // namespace environment
}  // namespace common
}  // namespace rmp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rmp::common::environment::FakeLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
