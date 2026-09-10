/**
 * *********************************************************
 *
 * @file: polygon2d.cpp
 * @brief: geometry: 2D polygon
 * @author: Yang Haodong
 * @date: 2024-9-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>
#include <climits>

#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "common/geometry/polygon2d.h"

namespace rmp
{
namespace common
{
namespace geometry
{
namespace
{
using namespace rmp::common::math;
}

/**
 * @brief Constructor which takes a vector of points as its vertices.
 * @param points The points to construct the polygon.
 */
Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points))
{
  buildFromPoints();
}

/**
 * @brief Get the vertices of the polygon.
 * @return The vertices of the polygon.
 */
const std::vector<Vec2d>& Polygon2d::points() const
{
  return points_;
}

/**
 * @brief Get the edges of the polygon.
 * @return The edges of the polygon.
 */
const std::vector<LineSegment2d>& Polygon2d::line_segments() const
{
  return line_segments_;
}

/**
 * @brief Get the number of vertices of the polygon.
 * @return The number of vertices of the polygon.
 */
int Polygon2d::num_points() const
{
  return num_points_;
}

/**
 * @brief Check if the polygon is convex.
 * @return Whether the polygon is convex or not.
 */
bool Polygon2d::is_convex() const
{
  return is_convex_;
}

/**
 * @brief Get the area of the polygon.
 * @return The area of the polygon.
 */
double Polygon2d::area() const
{
  return area_;
}

/**
 * @brief Compute the distance from a point to the polygon. If the point is
 *        within the polygon, return 0. Otherwise, this distance is
 *        the minimal distance from the point to the edges of the polygon.
 * @param point The point to compute whose distance to the polygon.
 * @return The distance from the point to the polygon.
 */
double Polygon2d::distanceTo(const Vec2d& point) const
{
  CHECK_GE(points_.size(), 3U);
  if (isPointIn(point))
  {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i)
  {
    distance = std::min(distance, line_segments_[i].distanceTo(point));
  }
  return distance;
}

/**
 * @brief Compute the distance from a line segment to the polygon.
 *        If the line segment is within the polygon, or it has intersect with
 *        the polygon, return 0. Otherwise, this distance is
 *        the minimal distance between the distances from the two ends
 *        of the line segment to the polygon.
 * @param line_segment The line segment to compute whose distance to
 *        the polygon.
 * @return The distance from the line segment to the polygon.
 */
double Polygon2d::distanceTo(const LineSegment2d& line_segment) const
{
  if (line_segment.length() <= kMathEpsilon)
  {
    return distanceTo(line_segment.start());
  }
  CHECK_GE(points_.size(), 3U);
  if (isPointIn(line_segment.center()))
  {
    return 0.0;
  }
  if (std::any_of(line_segments_.begin(), line_segments_.end(),
                  [&](const LineSegment2d& poly_seg) { return poly_seg.hasIntersect(line_segment); }))
  {
    return 0.0;
  }

  double distance = std::min(distanceTo(line_segment.start()), distanceTo(line_segment.end()));
  for (int i = 0; i < num_points_; ++i)
  {
    distance = std::min(distance, line_segment.distanceTo(points_[i]));
  }
  return distance;
}

/**
 * @brief Check if a point is within the polygon.
 * @param point The target point. To check if it is within the polygon.
 * @return Whether a point is within the polygon or not.
 */
bool Polygon2d::isPointIn(const Vec2d& point) const
{
  CHECK_GE(points_.size(), 3U);
  if (isPointOnBoundary(point))
  {
    return true;
  }
  int j = num_points_ - 1;
  int c = 0;
  for (int i = 0; i < num_points_; ++i)
  {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y()))
    {
      const double side = crossProd(point, points_[i], points_[j]);
      if (points_[i].y() < points_[j].y() ? side > 0.0 : side < 0.0)
      {
        ++c;
      }
    }
    j = i;
  }
  return c & 1;
}

/**
 * @brief Check if a point is on the boundary of the polygon.
 * @param point The target point. To check if it is on the boundary
 *        of the polygon.
 * @return Whether a point is on the boundary of the polygon or not.
 */
bool Polygon2d::isPointOnBoundary(const Vec2d& point) const
{
  CHECK_GE(points_.size(), 3U);
  return std::any_of(line_segments_.begin(), line_segments_.end(),
                     [&](const LineSegment2d& poly_seg) { return poly_seg.isPointIn(point); });
}

void Polygon2d::buildFromPoints()
{
  num_points_ = static_cast<int>(points_.size());
  CHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i)
  {
    area_ += crossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0)
  {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;
  CHECK_GT(area_, kMathEpsilon);

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i)
  {
    line_segments_.emplace_back(points_[i], points_[next(i)]);
  }

  // Check convexity.
  is_convex_ = true;
  for (int i = 0; i < num_points_; ++i)
  {
    if (crossProd(points_[prev(i)], points_[i], points_[next(i)]) <= -kMathEpsilon)
    {
      is_convex_ = false;
      break;
    }
  }

  // Compute aabox.
  min_x_ = points_[0].x();
  max_x_ = points_[0].x();
  min_y_ = points_[0].y();
  max_y_ = points_[0].y();
  for (const auto& point : points_)
  {
    min_x_ = std::min(min_x_, point.x());
    max_x_ = std::max(max_x_, point.x());
    min_y_ = std::min(min_y_, point.y());
    max_y_ = std::max(max_y_, point.y());
  }
}

/**
 * @brief Get the next index of given index.
 * @param at the given index
 * @return next index
 */
int Polygon2d::next(int at) const
{
  return at >= num_points_ - 1 ? 0 : at + 1;
}

/**
 * @brief Get the previous index of given index.
 * @param at the given index
 * @return previous index
 */
int Polygon2d::prev(int at) const
{
  return at == 0 ? num_points_ - 1 : at - 1;
}

}  // namespace geometry
}  // namespace common
}  // namespace rmp