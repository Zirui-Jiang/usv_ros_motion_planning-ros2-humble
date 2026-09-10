/**
 * *********************************************************
 *
 * @file: savitzky_golay.h
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
#ifndef RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_SMOOTH_SAVITZKY_GOLAY_H_
#define RMP_PATH_PLANNER_PATH_PROCESSOR_PATH_SMOOTH_SAVITZKY_GOLAY_H_

#include "path_planner/path_processor/path_processor.h"

namespace rmp
{
namespace path_planner
{
class SavitzkyGolayPathProcessor : public PathProcessor
{
public:
  /**
   * @brief Empty constructor
   */
  SavitzkyGolayPathProcessor() = default;

  /**
   * @brief  Destructor
   */
  virtual ~SavitzkyGolayPathProcessor() = default;

  /**
   * @brief Process the path according to a certain expectation
   * @param path_in The path to process
   * @param path_out The processed path
   */
  void process(const Points3d& path_in, Points3d& path_out);

private:
  /**
   * @brief Apply Savitzky-Golay convolution kernel to path subset
   * @param path_subset The path subset whose size is equal to the kernel
   * @param smooth_pt smoothed center point of path subset
   * @return falg true if successful smoothing
   */
  bool _applyFilter(const Points3d& path_subset, Point3d& smooth_pt);

  /**
   * @brief Apply Savitzky-Golay convolution over the path
   * @param path_in The path to process
   * @param path_out The processed path
   * @return falg true if successful smoothing
   */
  bool _applyFilterOverPath(const Points3d& path_in, Points3d& path_out);
};
}  // namespace path_planner
}  // namespace rmp

#endif