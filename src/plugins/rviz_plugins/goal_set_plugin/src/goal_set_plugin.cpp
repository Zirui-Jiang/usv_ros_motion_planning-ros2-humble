/***********************************************************
 *
 * @file: goal_set_plugin.cpp
 * @breif: Publish waypoints goal for Navigation
 * @author: Yang Haodong
 * @update: 2025-3-20
 * @version: 1.0
 *
 * Copyright (c) 2025， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
// 自定义 RViz 目标工具，把鼠标选定的平面位置和方向发布为 PoseStamped。
#include "rviz_plugins/goal_set_plugin/goal_set_plugin.h"

#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>

#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>

// 向 pluginlib 导出类，与 XML 中的类名和基类相配后 RViz 才能发现此工具。
PLUGINLIB_EXPORT_CLASS(rmp::plugins::goal_set_plugin::GoalSetPlugin, rviz_common::Tool)

namespace rmp
{
namespace plugins
{
namespace goal_set_plugin
{
// 快捷键为 g；Topic 与 Name 属性变化会通过 Qt 槽更新发布话题和工具名称。
GoalSetPlugin::GoalSetPlugin() : rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new rviz_common::properties::StringProperty("Topic", "/goal", "The topic on which to publish navigation goal.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
  name_property_ = new rviz_common::properties::StringProperty("Name", "Goal", "The goal name for specifical robot.",
                                            getPropertyContainer(), SLOT(updateName()), this);
}

// 初始化父类交互箭头、图标及 ROS 2 节点，再按当前属性建立发布器。
void GoalSetPlugin::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
  setName("Goal");
  setIcon(rviz_common::loadPixmap("package://goal_set_plugin/icons/classes/goal.png"));

  node_ = rclcpp::Node::make_shared("goal_set_plugin");
  updateTopic();
}

// 重建目标话题发布器，消息队列深度为 1；非法话题等异常被记录到 ROS 日志。
void GoalSetPlugin::updateTopic()
{
  try
  {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_property_->getStdString(), 1);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

// 更新工具栏显示名称，支持为多艘船分别命名目标设置工具。
void GoalSetPlugin::updateName()
{
  try
  {
    setName(name_property_->getString());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

// x/y 单位为 m，theta 为 rad；使用 RViz 当前 Fixed Frame 作消息坐标系，Z 固定为零。
void GoalSetPlugin::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped target;
  tf2::Quaternion q;
  // 将仅有艏向角的平面姿态转换为四元数，roll/pitch 均为零。
  q.setRPY(0, 0, theta);
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  target.header.frame_id = fixed_frame;
  target.header.stamp = node_->now();
  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = 0.0;
  target.pose.orientation.x = q[0];
  target.pose.orientation.y = q[1];
  target.pose.orientation.z = q[2];
  target.pose.orientation.w = q[3];
  RCLCPP_DEBUG(
      node_->get_logger(),
      "Setting fake prediction: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
      "Angle: %.3f\n",
      fixed_frame.c_str(), target.pose.position.x, target.pose.position.y, target.pose.position.z,
      target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w,
      theta);
  pub_->publish(target);
}
}  // namespace goal_set_plugin
}  // namespace plugins
}  // namespace rmp
