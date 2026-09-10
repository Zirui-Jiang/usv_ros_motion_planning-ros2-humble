/**
 * *********************************************************
 *
 * @file: visualizer.h
 * @brief: Contains visualization functions
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_UTIL_VISUALIZER_H_
#define RMP_COMMON_UTIL_VISUALIZER_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/geometry/point.h"
#include "common/structure/singleton.h"

namespace rmp
{
namespace common
{
namespace util
{
class Visualizer
{
public:
  using Point2d = rmp::common::geometry::Point2d;
  using Points2d = rmp::common::geometry::Points2d;
  using Point3d = rmp::common::geometry::Point3d;
  using Points3d = rmp::common::geometry::Points3d;
  using Line2d = std::pair<Point2d, Point2d>;
  using Lines2d = std::vector<Line2d>;

public:
  /**
   * @brief Publish path
   * @param path planning path
   * @param publisher publisher
   */
  template <typename Points>
  static void publishPlan(const Points& plan, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher,
                          const std::string& frame_id)
  {
    // create visulized path plan
    nav_msgs::msg::Path gui_plan;
    gui_plan.poses.resize(plan.size());
    gui_plan.header.frame_id = frame_id;
    gui_plan.header.stamp = rclcpp::Clock().now();
    for (unsigned int i = 0; i < plan.size(); i++)
    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, plan[i].theta());
      gui_plan.poses[i].header.stamp = gui_plan.header.stamp;
      gui_plan.poses[i].header.frame_id = frame_id;
      gui_plan.poses[i].pose.position.x = plan[i].x();
      gui_plan.poses[i].pose.position.y = plan[i].y();
      gui_plan.poses[i].pose.position.z = 0.0;
      gui_plan.poses[i].pose.orientation.x = q.getX();
      gui_plan.poses[i].pose.orientation.y = q.getY();
      gui_plan.poses[i].pose.orientation.z = q.getZ();
      gui_plan.poses[i].pose.orientation.w = q.getW();
    }

    publisher->publish(gui_plan);
  }

  template <typename Points>
  static void publishExpandZone(const Points& expand, const nav2_costmap_2d::Costmap2D* costmap,
                                const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& publisher,
                                const std::string& frame_id)
  {
    nav_msgs::msg::OccupancyGrid grid;
    float resolution = costmap->getResolution();

    // build expand
    grid.header.frame_id = frame_id;
    grid.header.stamp = rclcpp::Clock().now();
    grid.info.resolution = resolution;
    grid.info.width = costmap->getSizeInCellsX();
    grid.info.height = costmap->getSizeInCellsY();

    double wx, wy;
    costmap->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height);

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
      grid.data[i] = 0;
    }

    auto grid2index = [&](const Point3d& pt) {
      return static_cast<int>(pt.x()) + static_cast<int>(grid.info.width) * static_cast<int>(pt.y());
    };

    for (const auto& pt : expand)
    {
      grid.data[grid2index(pt)] = 50;
    }

    publisher->publish(grid);
  }

  /**
   * @brief publish points
   */
  template <typename Points>
  static void publishPoints(const Points& points,
                            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
                            const std::string& frame_id, const std::string& ns, std_msgs::msg::ColorRGBA color = RED,
                            double scale = 0.2, int type = SPHERE)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = 0;

    switch (type)
    {
      case CUBE:
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        break;
      case SPHERE:
      default:
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    }

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;

    marker.points.reserve(points.size());
    for (const auto& pt : points)
    {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = 0.0;
      marker.points.push_back(p);
    }

    if (!marker.points.empty())
    {
      publisher->publish(marker);
    }
  }

  /**
   * @brief publish poses as arrows
   */
  template <typename Pose>
  static void publishPoses(const Pose& pose,
                           const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
                           const std::string& frame_id, const std::string& ns, std_msgs::msg::ColorRGBA color = RED,
                           double length = 0.5, double shaft_diameter = 0.1, double head_diameter = 0.2)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.x();
    marker.pose.position.y = pose.y();
    marker.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta());
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    marker.scale.x = length;
    marker.scale.y = shaft_diameter;
    marker.scale.z = head_diameter;
    marker.color = color;

    publisher->publish(marker);
  }

  /**
   * @brief publish circles
   */
  template <typename Points>
  static void publishCircles2d(const Points& points, const std::vector<double> radius,
                               const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
                               const std::string& frame_id, const std::string& ns, std_msgs::msg::ColorRGBA color = RED,
                               double scale = 0.2)
  {
    if (points.empty() || points.size() != radius.size())
    {
      R_WARN << "Invalid input: points(" << points.size() << ") and radius(" << radius.size() << ") size mismatch";
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;

    const int num_segments = 36;
    const double delta_theta = 2 * M_PI / num_segments;

    for (size_t i = 0; i < points.size(); ++i)
    {
      geometry_msgs::msg::Point center;
      center.x = points[i].x();
      center.y = points[i].y();
      center.z = 0.0;
      const double r = radius[i];

      for (int j = 0; j < num_segments; ++j)
      {
        const double theta = j * delta_theta;
        const double theta_next = theta + delta_theta;

        geometry_msgs::msg::Point p1, p2;
        p1.x = center.x + r * cos(theta);
        p1.y = center.y + r * sin(theta);
        p1.z = center.z;

        p2.x = center.x + r * cos(theta_next);
        p2.y = center.y + r * sin(theta_next);
        p2.z = center.z;

        marker.points.push_back(center);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }

    publisher->publish(marker);
  }

  /**
   * @brief publish lines
   */
  static void publishLines2d(const Lines2d& lines,
                             const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
                             const std::string& frame_id, const std::string& ns, std_msgs::msg::ColorRGBA color = RED,
                             double scale = 0.2);

private:
  static std_msgs::msg::ColorRGBA _colorInit(double r, double g, double b, double a);

public:
  static std_msgs::msg::ColorRGBA RED;
  static std_msgs::msg::ColorRGBA DARK_GREEN;
  static std_msgs::msg::ColorRGBA DARK_BLUE;
  static std_msgs::msg::ColorRGBA PURPLE;
  static std_msgs::msg::ColorRGBA LIGHTPURPLE;
  enum MARKER_TYPE
  {
    CUBE = 0,
    SPHERE = 1,
  };
};

using VisualizerPtr = rmp::common::structure::Singleton<Visualizer>;

}  // namespace util
}  // namespace common
}  // namespace rmp

#endif
