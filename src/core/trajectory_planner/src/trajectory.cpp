/**
 * *********************************************************
 *
 * @file: trajectory.cpp
 * @brief: Trajectory with time, position, velocity and acceleration
 * @author: Yang Haodong
 * @date: 2025-4-13
 * @version: 2.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>

#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "trajectory_planner/trajectory.h"

using namespace rmp::common::math;

namespace rmp
{
namespace trajectory_planner
{
/**
 * @brief Represents a single trajectory point with various motion states
 * @param time_c         the timestamp of the trajectory point (default: 0.0)
 * @param s_c            the longitudinal distance along the trajectory (default: 0.0)
 * @param x_c            the x-coordinate in the Cartesian plane (default: 0.0)
 * @param y_c            the y-coordinate in the Cartesian plane (default: 0.0)
 * @param theta_c        the orientation angle in radians (default: 0.0)
 * @param curv_c         the curvature of the trajectory (default: 0.0)
 * @param velocity_c     the velocity at the trajectory point (default: 0.0)
 * @param acceleration_c the acceleration at the trajectory point (default: 0.0)
 */
TrajectoryCell::TrajectoryCell(double time_c, double s_c, double x_c, double y_c, double theta_c, double curv_c,
                               double velocity_c, double acceleration_c)
  : time(time_c)
  , s(s_c)
  , x(x_c)
  , y(y_c)
  , theta(theta_c)
  , curv(curv_c)
  , velocity(velocity_c)
  , acceleration(acceleration_c)
{
}

/**
 * @brief Constructs an iterator for the trajectory
 * @param traj  the trajectory to iterate over
 * @param index the initial index of the iterator
 */
Trajectory::Iterator::Iterator(const Trajectory& traj, size_t index) : traj_(traj), index_(index)
{
}

/**
 * @brief Dereferences the iterator to access the current trajectory cell
 * @return The current trajectory cell
 */
TrajectoryCell Trajectory::Iterator::operator*() const
{
  return { traj_.time_[index_],  traj_.s_[index_],    traj_.x_[index_],        traj_.y_[index_],
           traj_.theta_[index_], traj_.curv_[index_], traj_.velocity_[index_], traj_.acceleration_[index_] };
}

/**
 * @brief Advances the iterator to the next trajectory cell
 * @return Reference to the updated iterator
 */
Trajectory::Iterator& Trajectory::Iterator::operator++()
{
  ++index_;
  return *this;
}

/**
 * @brief Compares two iterators for inequality
 * @param other the other iterator to compare
 * @return true if the iterators are not equal, false otherwise
 */
bool Trajectory::Iterator::operator!=(const Iterator& other) const
{
  return index_ != other.index_;
}

/**
 * @brief Constructs a trajectory with a specified capacity
 * @param capacity the initial capacity for the trajectory (default: 0)
 */
Trajectory::Trajectory(int capacity)
{
  if (capacity > 0)
  {
    time_.reserve(capacity);
    s_.reserve(capacity);
    x_.reserve(capacity);
    y_.reserve(capacity);
    theta_.reserve(capacity);
    curv_.reserve(capacity);
    velocity_.reserve(capacity);
    acceleration_.reserve(capacity);
  }
}

/**
 * @brief Destructs the trajectory and releases resources
 */
Trajectory::~Trajectory()
{
  reset();
}

/**
 * @brief Checks if the trajectory is empty
 * @return true if the trajectory is empty, false otherwise
 */
bool Trajectory::empty() const
{
  return size_ == 0;
}

/**
 * @brief Gets the number of trajectory points
 * @return The size of the trajectory
 */
int Trajectory::size() const
{
  return size_;
}

/**
 * @brief Clears all trajectory points
 */
void Trajectory::clear()
{
  size_ = 0;
  time_.clear();
  s_.clear();
  x_.clear();
  y_.clear();
  theta_.clear();
  curv_.clear();
  velocity_.clear();
  acceleration_.clear();
}

/**
 * @brief Resets the trajectory with a new capacity
 * @param capacity the new capacity to reset to (default: 0)
 */
void Trajectory::reset(int capacity)
{
  size_ = 0;
  if (!time_.empty())
    std::vector<double>().swap(time_);

  if (!x_.empty())
    std::vector<double>().swap(x_);

  if (!y_.empty())
    std::vector<double>().swap(y_);

  if (!s_.empty())
    std::vector<double>().swap(s_);

  if (!theta_.empty())
    std::vector<double>().swap(theta_);

  if (!curv_.empty())
    std::vector<double>().swap(curv_);

  if (!velocity_.empty())
    std::vector<double>().swap(velocity_);

  if (!acceleration_.empty())
    std::vector<double>().swap(acceleration_);

  if (capacity > 0)
  {
    time_.reserve(capacity);
    s_.reserve(capacity);
    x_.reserve(capacity);
    y_.reserve(capacity);
    theta_.reserve(capacity);
    curv_.reserve(capacity);
    velocity_.reserve(capacity);
    acceleration_.reserve(capacity);
  }
}

/**
 * @brief Adds a new trajectory cell to the trajectory
 * @param cell the trajectory cell to add
 */
void Trajectory::push(const TrajectoryCell& cell)
{
  size_++;
  time_.push_back(cell.time);
  s_.push_back(cell.s);
  x_.push_back(cell.x);
  y_.push_back(cell.y);
  theta_.push_back(cell.theta);
  curv_.push_back(cell.curv);
  velocity_.push_back(cell.velocity);
  acceleration_.push_back(cell.acceleration);
}

/**
 * @brief Gets an iterator pointing to the beginning of the trajectory
 * @return An iterator to the first trajectory cell
 */
Trajectory::Iterator Trajectory::begin() const
{
  return Iterator(*this, 0);
}

/**
 * @brief Gets an iterator pointing to the end of the trajectory
 * @return An iterator to one past the last trajectory cell
 */
Trajectory::Iterator Trajectory::end() const
{
  return Iterator(*this, size_);
}

/**
 * @brief Gets a trajectory reference cell at a specific time
 * @param ref_t    the reference time
 * @param ref_data the resulting trajectory cell to populate
 * @return true if the reference cell is found, false otherwise
 */
bool Trajectory::getReferencWithTime(double ref_t, TrajectoryCell& ref_data)
{
  if (ref_t < 0.0)
  {
    R_ERROR << "Invalid time point.";
    return false;
  }
  else if (ref_t >= time_.back())
  {
    ref_data = requireTrajectoryCellAt(size_ - 1);
    return true;
  }
  else
  {
    auto it = std::lower_bound(time_.begin(), time_.end(), ref_t);
    size_t idx = std::distance(time_.begin(), it);

    const double t0 = time_[idx];
    const double t1 = time_[idx + 1];
    const double ratio = (ref_t - t0) / (t1 - t0);

    auto lerp = [](double a, double b, double t) { return a + t * (b - a); };

    ref_data.time = ref_t;
    ref_data.s = lerp(s_[idx], s_[idx + 1], ratio);
    ref_data.x = lerp(x_[idx], x_[idx + 1], ratio);
    ref_data.y = lerp(y_[idx], y_[idx + 1], ratio);

    double theta1_reg = pi2pi(theta_[idx]);
    double theta2_reg = pi2pi(theta_[idx + 1]);
    double delta_theta = theta2_reg - theta1_reg;
    if (delta_theta > M_PI)
    {
      delta_theta -= M_PI * 2.0;
    }
    else if (delta_theta < -M_PI)
    {
      delta_theta += M_PI * 2.0;
    }
    ref_data.theta = pi2pi(theta1_reg + ratio * delta_theta);

    ref_data.curv = lerp(curv_[idx], curv_[idx + 1], ratio);
    ref_data.velocity = lerp(velocity_[idx], velocity_[idx + 1], ratio);
    ref_data.acceleration = lerp(acceleration_[idx], acceleration_[idx + 1], ratio);
    return true;
  }
}

/**
 * @brief Gets a trajectory reference cell at a specific distance
 * @param ref_s    the reference distance
 * @param ref_data the resulting trajectory cell to populate
 * @return true if the reference cell is found, false otherwise
 */
bool Trajectory::getReferencWithDistance(double ref_s, TrajectoryCell& ref_data)
{
  if (ref_s < 0.0)
  {
    R_ERROR << "Invalid distance point.";
    return false;
  }
  else if (ref_s >= s_.back())
  {
    ref_data = requireTrajectoryCellAt(size_ - 1);
    return true;
  }
  else
  {
    auto it = std::lower_bound(s_.begin(), s_.end(), ref_s);
    size_t idx = std::distance(s_.begin(), it);

    const double s0 = s_[idx];
    const double s1 = s_[idx + 1];
    const double ratio = (ref_s - s0) / (s1 - s0);

    auto lerp = [](double a, double b, double t) { return a + t * (b - a); };

    ref_data.s = ref_s;
    ref_data.x = lerp(x_[idx], x_[idx + 1], ratio);
    ref_data.y = lerp(y_[idx], y_[idx + 1], ratio);
    ref_data.time = lerp(time_[idx], time_[idx + 1], ratio);

    double theta1_reg = pi2pi(theta_[idx]);
    double theta2_reg = pi2pi(theta_[idx + 1]);
    double delta_theta = theta2_reg - theta1_reg;
    if (delta_theta > M_PI)
    {
      delta_theta -= M_PI * 2.0;
    }
    else if (delta_theta < -M_PI)
    {
      delta_theta += M_PI * 2.0;
    }
    ref_data.theta = pi2pi(theta1_reg + ratio * delta_theta);

    ref_data.curv = lerp(curv_[idx], curv_[idx + 1], ratio);
    ref_data.velocity = lerp(velocity_[idx], velocity_[idx + 1], ratio);
    ref_data.acceleration = lerp(acceleration_[idx], acceleration_[idx + 1], ratio);
    return true;
  }
}
}  // namespace trajectory_planner
}  // namespace rmp