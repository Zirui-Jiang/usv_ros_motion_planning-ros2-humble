/**
 * *********************************************************
 *
 * @file: aabox2d.cpp
 * @brief: geometry: 2D axes-aligned bounding box class
 * @author: Yang Haodong
 * @date: 2025-3-19
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>
#include <cmath>

#include "common/geometry/aabox2d.h"
#include "common/math/math_helper.h"

namespace rmp
{
namespace common
{
namespace geometry
{
using namespace rmp::common::math;

/**
 * @brief Creates an axes-aligned box with given center, length, and width.
 * @param center The center of the box
 * @param length The size of the box along the x-axis
 * @param width The size of the box along the y-axis
 */
AABox2d::AABox2d(const Vec2d& center, const double length, const double width)
  : center_(center), width_(length), height_(width), half_width_(length / 2.0), half_height_(width / 2.0)
{
  CHECK_GT(width_, -kMathEpsilon);
  CHECK_GT(height_, -kMathEpsilon);
}
/**
 * @brief Creates an axes-aligned box from two opposite corners.
 * @param one_corner One corner of the box
 * @param opposite_corner The opposite corner to the first one
 */
AABox2d::AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner)
  : AABox2d((one_corner + opposite_corner) / 2.0, std::abs(one_corner.x() - opposite_corner.x()),
            std::abs(one_corner.y() - opposite_corner.y()))
{
}
/**
 * @brief Parameterized constructor.
 * Creates an axes-aligned box containing all points in a given vector.
 * @param points Vector of points to be included inside the box.
 */
AABox2d::AABox2d(const std::vector<Vec2d>& points)
{
  R_CHECK(!points.empty());
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto& point : points)
  {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  center_ = { (min_x + max_x) / 2.0, (min_y + max_y) / 2.0 };
  width_ = max_x - min_x;
  height_ = max_y - min_y;
  half_width_ = width_ / 2.0;
  half_height_ = height_ / 2.0;
}

/**
 * @brief Getter of center_
 * @return Center of the box
 */
const Vec2d& AABox2d::center() const
{
  return center_;
}
/**
 * @brief Getter of x-component of center_
 * @return x-component of the center of the box
 */
double AABox2d::center_x() const
{
  return center_.x();
}
/**
 * @brief Getter of y-component of center_
 * @return y-component of the center of the box
 */
double AABox2d::center_y() const
{
  return center_.y();
}
/**
 * @brief Getter of width
 * @return The width of the box
 */
double AABox2d::width() const
{
  return width_;
}
/**
 * @brief Getter of height_
 * @return The height of the box
 */
double AABox2d::height() const
{
  return height_;
}
/**
 * @brief Getter of half_width_
 * @return Half of the width of the box
 */
double AABox2d::half_width() const
{
  return half_width_;
}
/**
 * @brief Getter of half_height_
 * @return Half of the height of the box
 */
double AABox2d::half_height() const
{
  return half_height_;
}
/**
 * @brief Getter of area
 * @return The area of the box
 */
double AABox2d::area() const
{
  return width_ * height_;
}
/**
 * @brief Returns the minimum x-coordinate of the box
 *
 * @return x-coordinate
 */
double AABox2d::min_x() const
{
  return center_.x() - half_width_;
}
/**
 * @brief Returns the maximum x-coordinate of the box
 *
 * @return x-coordinate
 */
double AABox2d::max_x() const
{
  return center_.x() + half_width_;
}
/**
 * @brief Returns the minimum y-coordinate of the box
 *
 * @return y-coordinate
 */
double AABox2d::min_y() const
{
  return center_.y() - half_height_;
}
/**
 * @brief Returns the maximum y-coordinate of the box
 *
 * @return y-coordinate
 */
double AABox2d::max_y() const
{
  return center_.y() + half_height_;
}

/**
 * @brief Gets all corners in counter clockwise order.
 * @param corners Output where the corners are written
 */
void AABox2d::getCorners(std::vector<Vec2d>* const corners) const
{
  CHECK_NOTNULL(corners)->clear();
  corners->reserve(4);
  corners->emplace_back(center_.x() + half_width_, center_.y() - half_height_);
  corners->emplace_back(center_.x() + half_width_, center_.y() + half_height_);
  corners->emplace_back(center_.x() - half_width_, center_.y() + half_height_);
  corners->emplace_back(center_.x() - half_width_, center_.y() - half_height_);
}

/**
 * @brief Determines whether a given point is in the box.
 * @param point The point we wish to test for containment in the box
 */
bool AABox2d::isPointIn(const Vec2d& point) const
{
  return std::abs(point.x() - center_.x()) <= half_width_ + kMathEpsilon &&
         std::abs(point.y() - center_.y()) <= half_height_ + kMathEpsilon;
}

/**
 * @brief Determines whether a given point is on the boundary of the box.
 * @param point The point we wish to test for boundary membership
 */
bool AABox2d::isPointOnBoundary(const Vec2d& point) const
{
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_width_) <= kMathEpsilon && dy <= half_height_ + kMathEpsilon) ||
         (std::abs(dy - half_height_) <= kMathEpsilon && dx <= half_width_ + kMathEpsilon);
}

/**
 * @brief Determines the distance between a point and the box.
 * @param point The point whose distance to the box we wish to determine.
 */
double AABox2d::distanceTo(const Vec2d& point) const
{
  const double dx = std::abs(point.x() - center_.x()) - half_width_;
  const double dy = std::abs(point.y() - center_.y()) - half_height_;
  if (dx <= 0.0)
  {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0)
  {
    return dx;
  }
  return hypot(dx, dy);
}

/**
 * @brief Determines the distance between two boxes.
 * @param box Another box.
 */
double AABox2d::distanceTo(const AABox2d& box) const
{
  const double dx = std::abs(box.center_x() - center_.x()) - box.half_width() - half_width_;
  const double dy = std::abs(box.center_y() - center_.y()) - box.half_height() - half_height_;
  if (dx <= 0.0)
  {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0)
  {
    return dx;
  }
  return hypot(dx, dy);
}

/**
 * @brief Determines whether two boxes overlap.
 * @param box Another box
 */
bool AABox2d::hasOverlap(const AABox2d& box) const
{
  return std::abs(box.center_x() - center_.x()) <= box.half_width() + half_width_ &&
         std::abs(box.center_y() - center_.y()) <= box.half_height() + half_height_;
}

/**
 * @brief Shift the center of AABox by the input vector.
 * @param shift_vec The vector by which we wish to shift the box
 */
void AABox2d::shift(const Vec2d& shift_vec)
{
  center_ += shift_vec;
}

/**
 * @brief Changes box to include another given box, as well as the current one.
 * @param other_box Another box
 */
void AABox2d::mergeFrom(const AABox2d& other_box)
{
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  width_ = x2 - x1;
  height_ = y2 - y1;
  half_width_ = width_ / 2.0;
  half_height_ = height_ / 2.0;
}

/**
 * @brief Changes box to include a given point, as well as the current box.
 * @param other_point Another point
 */
void AABox2d::mergeFrom(const Vec2d& other_point)
{
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  width_ = x2 - x1;
  height_ = y2 - y1;
  half_width_ = width_ / 2.0;
  half_height_ = height_ / 2.0;
}

}  // namespace geometry
}  // namespace common
}  // namespace rmp
