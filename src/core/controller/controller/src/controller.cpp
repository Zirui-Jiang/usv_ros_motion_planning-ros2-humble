/**
 * *********************************************************
 *
 * @file: controller.cpp
 * @brief: Contains the abstract local controller class
 * @author: Yang Haodong
 * @date: 2024-01-20
 * @version: 1.3
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>

#include "common/math/math_helper.h"

#include "controller/controller.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::trajectory_planner;

namespace rmp
{
namespace controller
{
/**
 * @brief Construct a new Controller object
 * @param control_frequency    the control frequency
 * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
 * @param costmap_ros a pointer to the costmap used for collision checking and environmental awareness
 * @param tf         a pointer to the TF2 buffer used for managing transformations
 */
Controller::Controller(double control_frequency, rmp::common::structure::USV_MODEL usv_model,
                       nav2_costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf)
  : control_frequency_(control_frequency)
  , control_dt_(1.0 / control_frequency)
  , usv_model_(usv_model)
  , costmap_ros_(costmap_ros)
  , tf_(tf)
{
}

/**
 * @brief Gets the control frequency of the controller
 */
double Controller::control_frequency() const
{
  return control_frequency_;
}

/**
 * @brief Checks whether the goal has been reached
 * @return true if the goal has been reached, false otherwise
 */
bool Controller::isGoalReached()
{
  if (!initialized_)
  {
    R_ERROR << "controller has not been initialized.";
    return false;
  }

  if (is_goal_reached_)
  {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

/**
 * @brief Sets the global plan for the controller
 * @param global_plan the global plan represented as a series of 3D points
 * @return true if the global plan is successfully set, false otherwise
 */
bool Controller::setGlobalPlan(const rmp::common::geometry::Points3d& global_plan)
{
  if (!initialized_)
  {
    R_ERROR << "controller has not been initialized.";
    return false;
  }

  R_DEBUG << "Got new global plan";

  // set new plan
  global_plan_.clear();
  global_plan_ = global_plan;

  // reset plan parameters
  if (goal_ != global_plan_.back())
  {
    goal_ = global_plan_.back();
    is_goal_reached_ = false;
  }

  return true;
}

/**
 * @brief Sets the local plan for the controller
 * @param local_plan the local plan represented as a trajectory
 * @return true if the local plan is successfully set, false otherwise
 */
bool Controller::setLocalPlan(const Trajectory& local_plan)
{
  if (!initialized_)
  {
    R_ERROR << "controller has not been initialized.";
    return false;
  }

  R_DEBUG << "Got new local plan";

  // set new plan
  local_plan_.clear();
  local_plan_ = local_plan;
  return true;
}

/**
 * @brief Sets the current state of the USV
 * @param state the current state of the USV
 */
void Controller::setState(const rmp::common::structure::USV_STATE& state)
{
  usv_state_ = state;
}

/**
 * @brief Regularizes the control command to ensure it meets system constraints
 * @param ctrl_cmd the control command to be regularized
 */
void Controller::regularControlCommand(ControlCommand& ctrl_cmd)
{
  ctrl_cmd.setThrustCmdLeft(clamp(ctrl_cmd.thrust_cmd_left(), -1.0, 1.0));
  ctrl_cmd.setThrustCmdLateral(clamp(ctrl_cmd.thrust_cmd_lateral(), -1.0, 1.0));
  ctrl_cmd.setThrustCmdRight(clamp(ctrl_cmd.thrust_cmd_right(), -1.0, 1.0));
}

}  // namespace controller
}  // namespace rmp