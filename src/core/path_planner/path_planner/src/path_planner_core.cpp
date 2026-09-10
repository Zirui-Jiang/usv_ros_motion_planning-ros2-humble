/**
 * *********************************************************
 *
 * @file: path_planner_node.cpp
 * @brief: Contains the path planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "path_planner/path_planner_core.h"

namespace rmp
{
namespace path_planner
{
using namespace rmp::common::geometry;
using Visualizer = rmp::common::util::Visualizer;

/**
 * @brief Construct a new Graph Planner object
 */
PathPlannerCore::PathPlannerCore() : initialized_(false), node_(nullptr), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param node        ROS node
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
PathPlannerCore::PathPlannerCore(rclcpp::Node::SharedPtr node, std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlannerCore()
{
  initialize(node, name, costmap_ros);
}

/**
 * @brief Planner initialization
 * @param node       ROS node
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void PathPlannerCore::initialize(rclcpp::Node::SharedPtr node, std::string name, nav2_costmap_2d::Costmap2DROS* costmapRos)
{
  node_ = node;
  costmap_ros_ = costmapRos;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void PathPlannerCore::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    tolerance_ = node_->declare_parameter<double>(name + ".default_tolerance", 0.0);        // error tolerance
    is_outline_ = node_->declare_parameter<bool>(name + ".outline_map", false);             // whether outline the map or not
    is_expand_ = node_->declare_parameter<bool>(name + ".expand_zone", false);              // whether publish expand zone or not
    enable_resample_ = node_->declare_parameter<bool>(name + ".enable_resample", false);    // whether enable path resample
    resample_ratio_ = node_->declare_parameter<double>(name + ".resample_ratio", 0.5);      // path resample ratio

    PathPlannerFactory::PlannerProps path_planner_props;
    if (!PathPlannerFactory::createPlanner(node_, costmap_ros_, path_planner_props))
    {
      R_ERROR << "Create path planner failed.";
    }

    g_planner_ = path_planner_props.planner_ptr;
    planner_type_ = path_planner_props.planner_type;

    // register planning publisher
    plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name + "/plan", 1);
    random_tree_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(name + "/random_tree", 1);
    roadmap_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(name + "/roadmap", 1);
    particles_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(name + "/particles", 1);
    expand_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(name + "/expand", 1);

    // register planning service
    make_plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>(
        name + "/make_plan", std::bind(&PathPlannerCore::makePlanService, this, std::placeholders::_1,
                                       std::placeholders::_2));
  }
  else
  {
    R_WARN << "This planner has already been initialized, you can't call it twice, doing nothing";
  }
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool PathPlannerCore::makePlan(const Point3d& start, const Point3d& goal, Points3d& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool PathPlannerCore::makePlan(const Point3d& start, const Point3d& goal, double tolerance, Points3d& plan)
{
  // start thread mutex
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());
  if (!initialized_)
  {
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }
  // clear existing plan
  plan.clear();
  // visualization
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  PathPlanner::Points3d expand;
  bool path_found = false;

  // planning
  path_found = g_planner_->plan(start, goal, plan, expand);

  // convert path to ros plan
  if (path_found)
  {
    plan[0].setTheta(start.theta());
    plan.back().setTheta(goal.theta());
    if (enable_resample_)
    {
      PathPlanner::Points3d resample_plan;
      if (g_planner_->resample(plan, resample_plan, resample_ratio_))
      {
        plan = resample_plan;
      }
    }
    // publish visulization plan
    if (is_expand_)
    {
      if (planner_type_ == GRAPH_PLANNER)
      {
        // publish expand zone
        visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_, frame_id_);
      }
      else if (planner_type_ == SAMPLE_PLANNER || planner_type_ == ROADMAP_PLANNER)
      {
        Visualizer::Lines2d lines;
        for (const auto& node : expand)
        {
          // using theta to record parent id element
          if (node.theta() != 0)
          {
            int px_i, py_i;
            double px_d, py_d, x_d, y_d;
            g_planner_->index2Grid(node.theta(), px_i, py_i);
            g_planner_->map2World(px_i, py_i, px_d, py_d);
            g_planner_->map2World(node.x(), node.y(), x_d, y_d);
            lines.emplace_back(std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x_d, y_d }, { px_d, py_d }));
          }
        }
        if (planner_type_ == SAMPLE_PLANNER)
        {
          visualizer->publishLines2d(lines, random_tree_pub_, frame_id_, "tree", Visualizer::DARK_GREEN, 0.05);
        }
        else
        {
          visualizer->publishLines2d(lines, roadmap_pub_, frame_id_, "roadmap", Visualizer::RED, 0.01);
        }
      }
      else if (planner_type_ == EVOLUTION_PLANNER)
      {
        // publish expand particles
        Visualizer::Points2d markers;
        for (const auto& node : expand)
        {
          double wx, wy;
          g_planner_->map2World(node.x(), node.y(), wx, wy);
          markers.emplace_back(wx, wy);
        }
        visualizer->publishPoints(markers, particles_pub_, frame_id_, "particles", Visualizer::DARK_GREEN, 0.1,
                                  Visualizer::CUBE);
      }
      else
      {
        R_WARN << "Unknown planner type.";
      }
    }

    visualizer->publishPlan(plan, plan_pub_, frame_id_);
  }
  else
  {
    R_ERROR << "Failed to get a path.";
  }
  return !plan.empty();
}

/**
 * @brief Regeister planning service
 * @param req  request from client
 * @param resp response to server
 */
void PathPlannerCore::makePlanService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                                      std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp)
{
  Point3d start(req->start.pose.position.x, req->start.pose.position.y, tf2::getYaw(req->start.pose.orientation));
  Point3d goal(req->goal.pose.position.x, req->goal.pose.position.y, tf2::getYaw(req->goal.pose.orientation));
  Points3d plan;
  makePlan(start, goal, plan);

  resp->plan.header.stamp = node_->now();
  resp->plan.header.frame_id = frame_id_;
  getPlanFromPath(plan, resp->plan.poses);
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool PathPlannerCore::getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  if (!initialized_)
  {
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    // coding as message type
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta());
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    plan.push_back(pose);
  }

  return !plan.empty();
}

/**
 * @brief Prunes the global path to remove portions that the robot has already passed
 * @param robot_pose the current pose of the robot
 * @param path       the global path to be pruned
 * @return true if pruning is successful, false otherwise
 */
bool PathPlannerCore::prune(const Point3d& robot_pose, Points3d& path)
{
  auto calPoseDistance = [](const Point3d& ps_1, const Point3d& ps_2) {
    return std::hypot(ps_1.x() - ps_2.x(), ps_1.y() - ps_2.y());
  };

  auto getMinFuncVal = [](Points3d::iterator begin, Points3d::iterator end, std::function<double(const Point3d&)> cal) {
    if (begin == end)
      return end;

    auto min_val = cal(*begin);
    auto min_iter = begin;
    for (auto it = ++begin; it != end; it++)
    {
      auto val = cal(*it);
      if (val <= min_val)
      {
        min_val = val;
        min_iter = it;
      }
    }
    return min_iter;
  };

  // find the closest pose on the path to the robot
  auto transform_begin =
      getMinFuncVal(path.begin(), path.end(), [&](const Point3d& ps) { return calPoseDistance(robot_pose, ps); });

  // path pruning: remove the portion of the global plan that already passed so don't process it on the next iteration
  path.erase(std::begin(path), transform_begin);

  return true;
}
}  // namespace path_planner
}  // namespace rmp
