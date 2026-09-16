// 仿真真值定位：将 Gazebo 船模状态转为 odom 与 TF；位置 Z 归零，姿态仍保留原四元数。
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
// 订阅模型状态并发布定位；这是仿真真值适配，不包含真实传感器融合或定位滤波。
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
    // 坐标系名称随节点命名空间统一加前缀，保持 map、odom、base_link 的对应关系。
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
    // 消息三组数组长度必须一致，先检查边界再按同一下标取模型状态。
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
      // 同一仿真时刻不重复发布；时间回退视为新纪元，沿用现有逻辑允许重新开始。
      if (have_last_ && stamp == last_stamp_)
      {
        return;
      }
      have_last_ = true;
      last_stamp_ = stamp;

      // map -> odom: static identity (fake localization aligns odom with map).
      // 此工程令 map 到 odom 为恒等变换；不能据此假定其他定位系统也如此。
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
      // 提取艏向，以仅使用 yaw 的平面旋转近似计算船体系线速度，不包含横摇与纵摇旋转。
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
      // 船体系 x 向前、y 向左；投影后的纵横向速度分别进入控制状态 u、v。
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

// 初始化 ROS，持续处理定位回调，退出时释放节点与通信上下文。
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rmp::common::environment::FakeLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
