/**
 * *********************************************************
 *
 * @file: path_processor.h
 * @brief: Path pre-processing or post-processing
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
#include "path_planner/path_processor/path_processor.h"

namespace rmp
{
namespace path_planner
{
void PathProcessor::setObstacles(const Points3d& obstacles)
{
  obstacles_.clear();
  for (const auto& obstacle : obstacles)
  {
    obstacles_.emplace_back(obstacle.x(), obstacle.y());
  }
}

void PathProcessor::setObstacles(Points3d&& obstacles)
{
  obstacles_.clear();
  obstacles_ = std::forward<Points3d&&>(obstacles);
}
}  // namespace path_planner
}  // namespace rmp