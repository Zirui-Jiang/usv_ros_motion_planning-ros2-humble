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
// 声明可配置名称与话题的 RViz PoseTool 子类，发布 ROS 2 目标位姿消息。
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
// 继承 PoseTool 复用鼠标拖拽选择位置和航向的交互，只重写目标发布处理。
class GoalSetPlugin : public rviz_default_plugins::tools::PoseTool
{
  // 启用 Qt 元对象能力，配合 CMake AUTOMOC 生成属性变化到槽函数的连接支持。
  Q_OBJECT
public:
  GoalSetPlugin();
  virtual ~GoalSetPlugin()
  {
  }
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

// 两个槽分别响应 Topic 与 Name 属性编辑，发布话题和显示名称可以独立调整。
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
