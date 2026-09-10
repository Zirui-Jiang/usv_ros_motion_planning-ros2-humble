/**
 * *********************************************************
 *
 * @file: savitzky_golay.cpp
 * @brief: Savitzky-Golay Filter for unconstrained path smooth
 * @author: Yang Haodong
 * @date: 2024-11-17
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <array>

#include "common/util/log.h"
#include "common/geometry/line_segment2d.h"
#include "path_planner/path_smooth/savitzky_golay.h"

namespace rmp
{
namespace path_planner
{
namespace
{
constexpr std::array<double, 7> kernel = { -2.0 / 21.0, 3.0 / 21.0, 6.0 / 21.0, 7.0 / 21.0,
                                           6.0 / 21.0,  3.0 / 21.0, -2.0 / 21.0 };
}
/**
 * @brief Process the path according to a certain expectation
 * @param path_in The path to process
 * @param path_out The processed path
 */
void SavitzkyGolayPathProcessor::process(const Points3d& path_in, Points3d& path_out)
{
  path_out.clear();
  _applyFilterOverPath(path_in, path_out);
}

/**
 * @brief Apply Savitzky-Golay convolution kernel to path subset
 * @param path_subset The path subset whose size is equal to the kernel
 * @param smooth_pt smoothed center point of path subset
 * @return falg true if successful smoothing
 */
bool SavitzkyGolayPathProcessor::_applyFilter(const Points3d& path_subset, Point3d& smooth_pt)
{
  if (path_subset.size() != kernel.size())
  {
    R_WARN << "The size of path subset " << path_subset.size() << " is different with kernel size " << kernel.size()
           << ".";
    return false;
  }

  double smooth_pt_x = 0.0, smooth_pt_y = 0.0;
  for (unsigned int i = 0; i < kernel.size(); i++)
  {
    smooth_pt_x += kernel[i] * path_subset[i].x();
    smooth_pt_y += kernel[i] * path_subset[i].y();
  }
  smooth_pt.setX(smooth_pt_x);
  smooth_pt.setY(smooth_pt_y);
  return true;
}

/**
 * @brief Apply Savitzky-Golay convolution over the path
 * @param path_in The path to process
 * @param path_out The processed path
 * @return falg true if successful smoothing
 */
bool SavitzkyGolayPathProcessor::_applyFilterOverPath(const Points3d& path_in, Points3d& path_out)
{
  int path_size = static_cast<int>(path_in.size());
  if (path_size < kernel.size())
  {
    return true;
  }

  path_out.clear();
  auto pt_m3 = path_in[0];
  auto pt_m2 = path_in[0];
  auto pt_m1 = path_in[0];
  auto pt = path_in[1];
  auto pt_p1 = path_in[2];
  auto pt_p2 = path_in[3];
  auto pt_p3 = path_in[4];

  // First ang last point is fixed
  path_out.emplace_back(path_in[0].x(), path_in[0].y());
  for (unsigned int idx = 1; idx < path_size; idx++)
  {
    Point3d smooth_pt;
    if (!_applyFilter({ pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3 }, smooth_pt))
    {
      return false;
    }
    pt_m3 = pt_m2;
    pt_m2 = pt_m1;
    pt_m1 = pt;
    pt = pt_p1;
    pt_p1 = pt_p2;
    pt_p2 = pt_p3;
    pt_p3 = idx + 4 < path_size - 1 ? path_in[idx + 4] : path_in.back();
    path_out.emplace_back(smooth_pt.x(), smooth_pt.y());
  }
  return true;
}

}  // namespace path_planner
}  // namespace rmp