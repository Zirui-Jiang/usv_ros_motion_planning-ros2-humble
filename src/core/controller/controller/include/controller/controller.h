/**
 * *********************************************************
 *
 * @file: controller.h
 * @brief: Contains the abstract Controller class
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
#ifndef RMP_CONTROLLER_CONTROLLER_H_
#define RMP_CONTROLLER_CONTROLLER_H_

#include <tf2/utils.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "common/geometry/point.h"
#include "common/structure/usv_model.h"
#include "trajectory_planner/trajectory_planner.h"

namespace rmp
{
namespace controller
{
/**
 * @brief Represents a control command for a vehicle with three thrust components
 */
struct ControlCommand
{
public:
  /**
   * @brief Constructor to initialize the control command with specified thrust values
   * @param thrust_cmd_left    the thrust command for the left side (default: 0.0)
   * @param thrust_cmd_lateral the thrust command for the lateral direction (default: 0.0)
   * @param thrust_cmd_right   the thrust command for the right side (default: 0.0)
   */
  ControlCommand(double thrust_cmd_left = 0.0, double thrust_cmd_lateral = 0.0, double thrust_cmd_right = 0.0)
    : thrust_cmd_left_(thrust_cmd_left), thrust_cmd_lateral_(thrust_cmd_lateral), thrust_cmd_right_(thrust_cmd_right)
  {
  }

  /**
   * @brief Gets the thrust command for the left side
   */
  double thrust_cmd_left()
  {
    return thrust_cmd_left_;
  }

  /**
   * @brief Gets the thrust command for the lateral direction
   */
  double thrust_cmd_lateral()
  {
    return thrust_cmd_lateral_;
  }

  /**
   * @brief Gets the thrust command for the right side
   */
  double thrust_cmd_right()
  {
    return thrust_cmd_right_;
  }

  /**
   * @brief Sets the thrust command for the left side
   * @param thrust_cmd_left the new thrust command for the left side
   */
  void setThrustCmdLeft(double thrust_cmd_left)
  {
    thrust_cmd_left_ = thrust_cmd_left;
  }

  /**
   * @brief Sets the thrust command for the lateral direction
   * @param thrust_cmd_lateral the new thrust command for the lateral direction
   */
  void setThrustCmdLateral(double thrust_cmd_lateral)
  {
    thrust_cmd_lateral_ = thrust_cmd_lateral;
  }

  /**
   * @brief Sets the thrust command for the right side
   * @param thrust_cmd_right the new thrust command for the right side
   */
  void setThrustCmdRight(double thrust_cmd_right)
  {
    thrust_cmd_right_ = thrust_cmd_right;
  }

private:
  double thrust_cmd_left_;     // Thrust command for the left side
  double thrust_cmd_lateral_;  // Thrust command for the lateral direction
  double thrust_cmd_right_;    // Thrust command for the right side
};

/**
 * @brief Base class for a controller, providing interfaces for managing control commands
 */
class Controller
{
public:
  /**
   * @brief Construct a new Controller object
   * @param control_frequency    the control frequency
   * @param usv_model  the model of the Unmanned Surface Vehicle (USV) being used
   * @param costmap_ros a pointer to the costmap used for collision checking and environmental awareness
   * @param tf         a pointer to the TF2 buffer used for managing transformations
   */
  Controller(double control_frequency = 10.0, rmp::common::structure::USV_MODEL usv_model = {},
             nav2_costmap_2d::Costmap2DROS* costmap_ros = nullptr, tf2_ros::Buffer* tf = nullptr);

  /**
   * @brief Destroy the Controller object
   */
  ~Controller() = default;

  /**
   * @brief Gets the control frequency of the controller
   */
  double control_frequency() const;

  /**
   * @brief Computes the control command for the vehicle
   * @param ctrl_cmd the control command to be populated
   * @return true if the control command is successfully computed, false otherwise
   */
  virtual bool computeControlCommand(ControlCommand& ctrl_cmd) = 0;

  /**
   * @brief Checks whether the goal has been reached
   * @return true if the goal has been reached, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Sets the global plan for the controller
   * @param global_plan the global plan represented as a series of 3D points
   * @return true if the global plan is successfully set, false otherwise
   */
  bool setGlobalPlan(const rmp::common::geometry::Points3d& global_plan);

  /**
   * @brief Sets the local plan for the controller
   * @param local_plan the local plan represented as a trajectory
   * @return true if the local plan is successfully set, false otherwise
   */
  bool setLocalPlan(const rmp::trajectory_planner::Trajectory& local_plan);

  /**
   * @brief Sets the current state of the USV
   * @param state the current state of the USV
   */
  void setState(const rmp::common::structure::USV_STATE& state);

  /**
   * @brief Regularizes the control command to ensure it meets system constraints
   * @param ctrl_cmd the control command to be regularized
   */
  void regularControlCommand(ControlCommand& ctrl_cmd);

protected:
  double control_frequency_;                        // control frequency
  double control_dt_;                               // control time interval
  rmp::common::structure::USV_MODEL usv_model_;     // The model of the USV
  nav2_costmap_2d::Costmap2DROS* costmap_ros_;           // Pointer to the costmap ROS wrapper
  tf2_ros::Buffer* tf_;                             // Pointer to the transformation buffer
  rmp::common::structure::USV_STATE usv_state_;     // current state of the USV
  bool initialized_{ false };                       // Flag indicating if the controller is initialized
  bool is_goal_reached_{ false };                   // Flag indicating if the goal is reached
  rmp::common::geometry::Point3d goal_;             // Goal position for navigation
  rmp::common::geometry::Points3d global_plan_;     // Global plan from planning module
  rmp::trajectory_planner::Trajectory local_plan_;  // Global plan from planning module
};
}  // namespace controller
}  // namespace rmp
#endif