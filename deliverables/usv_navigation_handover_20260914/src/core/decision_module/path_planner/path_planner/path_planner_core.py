"""Public navigation interface; ROS messages and publishers stay in adapters."""

import math
import time
import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, PlanResult
from .path_planner import prune, resample
from .utils.path_planner_factory import create_planner


def path_is_clear_with_inflation_escape(grid, path, threshold=254,
                                       unknown_is_obstacle=True, *, outline_map=False,
                                       allow_corner_cutting=True):
    """Validate segments cell by cell, allowing only an initial soft descent.

    Repeated points/samples in the same cell do not count as a plateau step.
    Each transition into a *different* high-cost cell must strictly decrease
    cost. Once a below-threshold cell is reached, high cells are forbidden.
    The final cell must be free. Hard obstacles and outlined boundaries are
    never permitted. This does not replace the separate swept hull check.

    Grid-boundary traversal visits every cell interior crossed by a segment,
    including short visits that half-cell-spaced samples can miss. Exact
    corner crossings retain the planner's explicit diagonal corner policy.
    """
    points = np.asarray(path, dtype=float)
    if (points.ndim != 2 or len(points) == 0 or points.shape[1] < 2
            or not np.isfinite(points).all()):
        return False
    cells = [grid.world_to_map(*point[:2]) for point in points]
    if any(cell is None for cell in cells):
        return False

    def hard(cell):
        x, y = cell
        if not (0 <= x < grid.width and 0 <= y < grid.height):
            return True
        value = int(grid.data[y, x])
        return (value == 254 or (value == 255 and unknown_is_obstacle)
                or (outline_map and (x in (0, grid.width-1) or y in (0, grid.height-1))))

    def blocked(cell):
        if hard(cell):
            return True
        x, y = cell
        value = int(grid.data[y, x])
        return value != 255 and value >= threshold

    if hard(cells[0]) or blocked(cells[-1]):
        return False
    current = cells[0]
    origin = np.array([grid.origin_x, grid.origin_y])
    for start, end, target in zip(points[:-1, :2], points[1:, :2], cells[1:]):
        p, q = (start-origin)/grid.resolution, (end-origin)/grid.resolution
        dx, dy = q-p
        sx, sy = int(np.sign(dx)), int(np.sign(dy))
        tx = ((current[0] + (sx > 0) - p[0]) / dx) if sx else math.inf
        ty = ((current[1] + (sy > 0) - p[1]) / dy) if sy else math.inf
        step_x, step_y = (1/abs(dx) if sx else math.inf), (1/abs(dy) if sy else math.inf)
        while current != target:
            x, y = current
            # Do not step an axis beyond its endpoint because of a boundary
            # roundoff at t=1. All earlier cell intersections are retained.
            cross_x = x != target[0] and (y == target[1] or tx <= ty)
            cross_y = y != target[1] and (x == target[0] or ty <= tx)
            following = (x + sx if cross_x else x, y + sy if cross_y else y)
            if hard(following):
                return False
            if cross_x and cross_y and not allow_corner_cutting:
                if blocked((following[0], y)) or blocked((x, following[1])):
                    return False
            if blocked(following) and (not blocked(current)
                                      or grid.data[following[1], following[0]] >= grid.data[y, x]):
                return False
            current = following
            if cross_x:
                tx += step_x
            if cross_y:
                ty += step_y
    return True


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
            def valid(path):
                return path_is_clear_with_inflation_escape(
                    grid, path, threshold, unknown, outline_map=self.planner.outline_map,
                    allow_corner_cutting=self.planner.allow_corner_cutting)

            if not valid(result.path):
                result.status = AlgorithmStatus.NO_PATH
                result.path = np.empty((0, 3))
                result.message = "grid path failed segment collision validation"
            elif self.config.get("enable_resample", False):
                try:
                    candidate = resample(result.path, self.spacing)
                except ValueError as error:
                    result.message = f"resampling skipped: {error}"
                else:
                    if valid(candidate):
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
