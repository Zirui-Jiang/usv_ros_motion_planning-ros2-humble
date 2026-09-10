"""Public navigation interface; ROS messages and publishers stay in adapters."""

import time
import numpy as np

from common.math_helper import path_is_clear
from common.models import AlgorithmStatus, GridSnapshot, PlanResult
from .path_planner import prune, resample
from .utils.path_planner_factory import create_planner


class PathPlannerCore:
    def __init__(self, config: dict | None = None):
        self.config = dict(config or {})
        self.planner = create_planner(self.config)
        self.spacing = float(self.config.get("resample_spacing", self.config.get("resample_ratio", 0.5)))
        if self.config.get("enable_resample", False):
            # Validate even if the first planning attempt fails.
            resample([], self.spacing)

    def make_plan(self, start, goal, grid: GridSnapshot) -> PlanResult:
        begun = time.perf_counter()
        result = self.planner.plan(start, goal, grid)
        if result.status == AlgorithmStatus.SUCCESS:
            threshold = 254.0 * self.planner.factor
            unknown = self.planner.unknown_is_obstacle
            if not path_is_clear(grid, result.path, threshold, unknown):
                result.status = AlgorithmStatus.NO_PATH
                result.path = np.empty((0, 3))
                result.message = "grid path failed segment collision validation"
            elif self.config.get("enable_resample", False):
                try:
                    candidate = resample(result.path, self.spacing)
                except ValueError as error:
                    result.message = f"resampling skipped: {error}"
                else:
                    if path_is_clear(grid, candidate, threshold, unknown):
                        result.path = candidate
                    else:
                        result.message = "resampling crossed an obstacle; retained the grid path"
        result.elapsed = time.perf_counter() - begun
        return result

    @staticmethod
    def prune(path, state, prune_distance=0.0):
        return prune(path, state, prune_distance)

    @staticmethod
    def resample(path, spacing):
        return resample(path, spacing)
