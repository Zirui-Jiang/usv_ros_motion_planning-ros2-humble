/**
 * *********************************************************
 *
 * @file: ramer_douglas_peucker.cpp
 * @brief: ramer douglas peucker for path downsampling
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/math/math_helper.h"
#include "common/geometry/line_segment2d.h"
#include "path_planner/path_prune/ramer_douglas_peucker.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp
{
namespace path_planner
{
/**
 * @brief Empty constructor
 */
RDPPathProcessor::RDPPathProcessor() : delta_(0.0), max_interval_(0.0){};

/**
 * @brief Constructor
 * @param delta The error threshold
 * @param max_interval the maximum interval between two points
 */
RDPPathProcessor::RDPPathProcessor(double delta, double max_interval) : delta_(delta), max_interval_(max_interval){};

/**
 * @brief Process the path according to a certain expectation
 * @param path_in The path to process
 * @param path_out The processed path
 */
void RDPPathProcessor::process(const Points3d& path_in, Points3d& path_out)
{
  path_out.clear();
  int max_idx = -1;
  double max_dist = -1.0;
  int path_size = static_cast<int>(path_in.size());
  LineSegment2d line({ path_in[0].x(), path_in[0].y() }, { path_in[path_size - 1].x(), path_in[path_size - 1].y() });
  for (int i = 1; i < path_size - 1; i++)
  {
    double d = line.distanceTo({ path_in[i].x(), path_in[i].y() });
    if (d > max_dist)
    {
      max_dist = d;
      max_idx = i;
    }
  }

  if (max_dist > delta_)
  {
    Points3d left_pts, right_pts;
    left_pts.reserve(max_idx + 1);
    right_pts.reserve(path_size - max_idx);
    for (int i = 0; i <= max_idx; i++)
    {
      left_pts.emplace_back(path_in[i].x(), path_in[i].y(), path_in[i].theta());
    }
    for (int i = max_idx; i < path_size; i++)
    {
      right_pts.emplace_back(path_in[i].x(), path_in[i].y(), path_in[i].theta());
    }

    Points3d left_result, right_result;
    process(left_pts, left_result);
    process(right_pts, right_result);
    path_out.insert(path_out.end(), left_result.begin(), left_result.end() - 1);
    path_out.insert(path_out.end(), right_result.begin(), right_result.end());
  }
  else
  {
    const auto& start = path_in[0];
    const auto& end = path_in[path_size - 1];
    const double start_to_end = std::hypot(start.x() - end.x(), start.y() - end.y());
    int interp_cnt = static_cast<int>(start_to_end / max_interval_);
    path_out.emplace_back(start.x(), start.y(), start.theta());
    if (interp_cnt > 0)
    {
      const double dx = (end.x() - start.x()) / (interp_cnt + 1);
      const double dy = (end.y() - start.y()) / (interp_cnt + 1);
      const double dtheta = (pi2pi(end.theta() - start.theta())) / (interp_cnt + 1);
      for (int i = 1; i <= interp_cnt; i++)
      {
        path_out.emplace_back(start.x() + dx * i, start.y() + dy * i, pi2pi(start.theta() + dtheta * i));
      }
    }
    path_out.emplace_back(end.x(), end.y(), end.theta());
  }
}
}  // namespace path_planner
}  // namespace rmp