/**
 * *********************************************************
 *
 * @file: controller_factory.h
 * @brief: Create the controller with specifical parameters
 * @author: Yang Haodong
 * @date: 2025-02-16
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_UTILS_CONTROLLER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_CONTROLLER_FACTORY_H_

#include <rclcpp/rclcpp.hpp>

#include "controller/controller.h"
#include "common/structure/usv_model.h"

namespace rmp
{
namespace controller
{
class ControllerFactory
{
public:
  struct ControllerProps
  {
    std::unique_ptr<Controller> controller_ptr;  // controller
  };

public:
  /**
   * @brief Create and configure planner
   * @param nh ROS node
   * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
   * @param costmap_ros costmap ROS wrapper
   * @param planner_props planner property
   * @return bool true if create successful, else false
   */
  static bool createController(rclcpp::Node::SharedPtr nh, rmp::common::structure::USV_MODEL usv_model,
                               nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf,
                               ControllerProps& controller_props);
};
}  // namespace controller
}  // namespace rmp

#endif
