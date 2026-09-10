/**
 * *********************************************************
 *
 * @file: aabox2d.h
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

#pragma once

#include <string>
#include <vector>

#include "common/util/log.h"
#include "common/geometry/vec2d.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @class AABox2d
 * @brief Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
 * This class is referential-agnostic.
 */
class AABox2d
{
public:
  /**
   * @brief Creates an axes-aligned box with zero length and width at the origin.
   */
  AABox2d() = default;
  /**
   * @brief Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  AABox2d(const Vec2d& center, const double length, const double width);
  /**
   * @brief Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  AABox2d(const Vec2d& one_corner, const Vec2d& opposite_corner);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const std::vector<Vec2d>& points);

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  const Vec2d& center() const;
  /**
   * @brief Getter of x-component of center_
   * @return x-component of the center of the box
   */
  double center_x() const;
  /**
   * @brief Getter of y-component of center_
   * @return y-component of the center of the box
   */
  double center_y() const;
  /**
   * @brief Getter of width
   * @return The width of the box
   */
  double width() const;
  /**
   * @brief Getter of height_
   * @return The height of the box
   */
  double height() const;
  /**
   * @brief Getter of half_width_
   * @return Half of the width of the box
   */
  double half_width() const;
  /**
   * @brief Getter of half_height_
   * @return Half of the height of the box
   */
  double half_height() const;
  /**
   * @brief Getter of area
   * @return The area of the box
   */
  double area() const;
  /**
   * @brief Returns the minimum x-coordinate of the box
   * @return x-coordinate
   */
  double min_x() const;
  /**
   * @brief Returns the maximum x-coordinate of the box
   * @return x-coordinate
   */
  double max_x() const;
  /**
   * @brief Returns the minimum y-coordinate of the box
   * @return y-coordinate
   */
  double min_y() const;
  /**
   * @brief Returns the maximum y-coordinate of the box
   * @return y-coordinate
   */
  double max_y() const;

  /**
   * @brief Gets all corners in counter clockwise order.
   * @param corners Output where the corners are written
   */
  void getCorners(std::vector<Vec2d>* const corners) const;

  /**
   * @brief Determines whether a given point is in the box.
   * @param point The point we wish to test for containment in the box
   */
  bool isPointIn(const Vec2d& point) const;

  /**
   * @brief Determines whether a given point is on the boundary of the box.
   * @param point The point we wish to test for boundary membership
   */
  bool isPointOnBoundary(const Vec2d& point) const;

  /**
   * @brief Determines the distance between a point and the box.
   * @param point The point whose distance to the box we wish to determine.
   */
  double distanceTo(const Vec2d& point) const;

  /**
   * @brief Determines the distance between two boxes.
   * @param box Another box.
   */
  double distanceTo(const AABox2d& box) const;

  /**
   * @brief Determines whether two boxes overlap.
   * @param box Another box
   */
  bool hasOverlap(const AABox2d& box) const;

  /**
   * @brief Shift the center of AABox by the input vector.
   * @param shift_vec The vector by which we wish to shift the box
   */
  void shift(const Vec2d& shift_vec);

  /**
   * @brief Changes box to include another given box, as well as the current one.
   * @param other_box Another box
   */
  void mergeFrom(const AABox2d& other_box);

  /**
   * @brief Changes box to include a given point, as well as the current box.
   * @param other_point Another point
   */
  void mergeFrom(const Vec2d& other_point);

private:
  Vec2d center_;
  double width_ = 0.0;
  double height_ = 0.0;
  double half_width_ = 0.0;
  double half_height_ = 0.0;
};

}  // namespace geometry
}  // namespace common
}  // namespace rmp
