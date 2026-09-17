/***********************************************************
 *
 * @file: goal_set_plugin.h
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
#include <QObject>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace rviz_common
{
namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rmp
{
namespace plugins
{
namespace goal_set_plugin
{
class GoalSetPlugin : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  GoalSetPlugin();
  virtual ~GoalSetPlugin()
  {
  }
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();
  void updateName();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  rviz_common::properties::StringProperty* name_property_;
  rviz_common::properties::StringProperty* topic_property_;
};
}  // namespace goal_set_plugin
}  // namespace plugins
}  // namespace rmp
