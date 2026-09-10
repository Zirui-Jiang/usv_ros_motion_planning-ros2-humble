/**
 * *********************************************************
 *
 * @file: trajectory.h
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
#ifndef RMP_TRAJECTORY_PLANNER_TRAJECTORY_H_
#define RMP_TRAJECTORY_PLANNER_TRAJECTORY_H_

#include <vector>
#include <limits>

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
struct TrajectoryCell
{
  TrajectoryCell(double time_c = 0.0, double s_c = 0.0, double x_c = 0.0, double y_c = 0.0, double theta_c = 0.0,
                 double curv_c = 0.0, double velocity_c = 0.0, double acceleration_c = 0.0);
  double time{ 0.0 };
  double s{ 0.0 };
  double x{ 0.0 };
  double y{ 0.0 };
  double theta{ 0.0 };
  double curv{ 0.0 };
  double velocity{ 0.0 };
  double acceleration{ 0.0 };
};

/**
 * @brief Represents the entire trajectory consisting of multiple trajectory points
 */
class Trajectory
{
public:
  /**
   * @brief Iterator for traversing through the trajectory
   */
  class Iterator
  {
  public:
    /**
     * @brief Constructs an iterator for the trajectory
     * @param traj  the trajectory to iterate over
     * @param index the initial index of the iterator
     */
    Iterator(const Trajectory& traj, size_t index);

    /**
     * @brief Dereferences the iterator to access the current trajectory cell
     * @return The current trajectory cell
     */
    TrajectoryCell operator*() const;

    /**
     * @brief Advances the iterator to the next trajectory cell
     * @return Reference to the updated iterator
     */
    Iterator& operator++();

    /**
     * @brief Compares two iterators for inequality
     * @param other the other iterator to compare
     * @return true if the iterators are not equal, false otherwise
     */
    bool operator!=(const Iterator& other) const;

  private:
    const Trajectory& traj_;  // Reference to the trajectory being iterated
    size_t index_;            // Current index of the iterator
  };

public:
  /**
   * @brief Constructs a trajectory with a specified capacity
   * @param capacity the initial capacity for the trajectory (default: 0)
   */
  Trajectory(int capacity = 0);

  /**
   * @brief Destructs the trajectory and releases resources
   */
  ~Trajectory();

  /**
   * @brief Checks if the trajectory is empty
   * @return true if the trajectory is empty, false otherwise
   */
  bool empty() const;

  /**
   * @brief Gets the number of trajectory points
   * @return The size of the trajectory
   */
  int size() const;

  /**
   * @brief Clears all trajectory points
   */
  void clear();

  /**
   * @brief Resets the trajectory with a new capacity
   * @param capacity the new capacity to reset to (default: 0)
   */
  void reset(int capacity = 0);

  /**
   * @brief Adds a new trajectory cell to the trajectory
   * @param cell the trajectory cell to add
   */
  void push(const TrajectoryCell& cell);

  /**
   * @brief Retrieves the time at a specific index
   * @param idx the index of the trajectory point
   * @return The time at the specified index, or NaN if the index is invalid
   */
  inline double requireTimeAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? time_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the longitudinal distance at a specific index
   * @param idx the index of the trajectory point
   * @return The longitudinal distance at the specified index, or NaN if the index is invalid
   */
  inline double requireDistanceAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? s_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the x-coordinate at a specific index
   * @param idx the index of the trajectory point
   * @return The x-coordinate at the specified index, or NaN if the index is invalid
   */
  inline double requireXposAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? x_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the y-coordinate at a specific index
   * @param idx the index of the trajectory point
   * @return The y-coordinate at the specified index, or NaN if the index is invalid
   */
  inline double requireYposAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? y_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the orientation angle at a specific index
   * @param idx the index of the trajectory point
   * @return The orientation angle at the specified index, or NaN if the index is invalid
   */
  inline double requireThetaAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? theta_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the curvature at a specific index
   * @param idx the index of the trajectory point
   * @return The curvature at the specified index, or NaN if the index is invalid
   */
  inline double requireCurvatureAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? curv_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the velocity at a specific index
   * @param idx the index of the trajectory point
   * @return The velocity at the specified index, or NaN if the index is invalid
   */
  inline double requireVelocityAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? velocity_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves the acceleration at a specific index
   * @param idx the index of the trajectory point
   * @return The acceleration at the specified index, or NaN if the index is invalid
   */
  inline double requireAccelerationAt(int idx)
  {
    return (idx >= 0 && idx < size_) ? acceleration_[idx] : std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Retrieves a full trajectory cell at a specific index
   * @param idx the index of the trajectory point
   * @return The trajectory cell at the specified index
   */
  inline TrajectoryCell requireTrajectoryCellAt(int idx)
  {
    return { requireTimeAt(idx),  requireDistanceAt(idx),  requireXposAt(idx),     requireYposAt(idx),
             requireThetaAt(idx), requireCurvatureAt(idx), requireVelocityAt(idx), requireAccelerationAt(idx) };
  }

  /**
   * @brief Gets an iterator pointing to the beginning of the trajectory
   * @return An iterator to the first trajectory cell
   */
  Iterator begin() const;

  /**
   * @brief Gets an iterator pointing to the end of the trajectory
   * @return An iterator to one past the last trajectory cell
   */
  Iterator end() const;

  /**
   * @brief Gets a trajectory reference cell at a specific time
   * @param ref_t    the reference time
   * @param ref_data the resulting trajectory cell to populate
   * @return true if the reference cell is found, false otherwise
   */
  bool getReferencWithTime(double ref_t, TrajectoryCell& ref_data);

  /**
   * @brief Gets a trajectory reference cell at a specific distance
   * @param ref_s    the reference distance
   * @param ref_data the resulting trajectory cell to populate
   * @return true if the reference cell is found, false otherwise
   */
  bool getReferencWithDistance(double ref_s, TrajectoryCell& ref_data);

private:
  int size_{ 0 };                     // Number of trajectory points
  std::vector<double> time_;          // Timestamps of trajectory points
  std::vector<double> s_;             // Longitudinal distances of trajectory points
  std::vector<double> x_;             // X-coordinates of trajectory points
  std::vector<double> y_;             // Y-coordinates of trajectory points
  std::vector<double> theta_;         // Orientation angles of trajectory points
  std::vector<double> curv_;          // Curvatures of trajectory points
  std::vector<double> velocity_;      // Velocities of trajectory points
  std::vector<double> acceleration_;  // Accelerations of trajectory points
};

}  // namespace trajectory_planner
}  // namespace rmp

#endif