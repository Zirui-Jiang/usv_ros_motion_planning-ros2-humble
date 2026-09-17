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
#include "rviz_plugins/goal_set_plugin/goal_set_plugin.h"

#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>

#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmp::plugins::goal_set_plugin::GoalSetPlugin, rviz_common::Tool)

namespace rmp
{
namespace plugins
{
namespace goal_set_plugin
{
GoalSetPlugin::GoalSetPlugin() : rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
  topic_property_ = new rviz_common::properties::StringProperty("Topic", "/goal", "The topic on which to publish navigation goal.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
  name_property_ = new rviz_common::properties::StringProperty("Name", "Goal", "The goal name for specifical robot.",
                                            getPropertyContainer(), SLOT(updateName()), this);
}

void GoalSetPlugin::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
  setName("Goal");
  setIcon(rviz_common::loadPixmap("package://goal_set_plugin/icons/classes/goal.png"));

  node_ = rclcpp::Node::make_shared("goal_set_plugin");
  updateTopic();
}

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

void GoalSetPlugin::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped target;
  tf2::Quaternion q;
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
