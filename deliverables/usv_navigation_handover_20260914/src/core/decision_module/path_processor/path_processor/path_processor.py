"""Shared validation and obstacle storage for optional path processors."""

import numpy as np


def as_path(path):
    points = np.asarray(path, dtype=float)
    if not points.size:
        return np.empty((0, 3), dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or not np.isfinite(points).all():
        raise ValueError("path must be a finite (N, 2) or (N, 3) array")
    if points.shape[1] == 2:
        points = np.column_stack((points, np.zeros(len(points))))
    return points.copy()


class PathProcessor:
    """Processors return an owned path; geometric processing is unconstrained.

    Stored obstacles remain available for caller diagnostics. As in the old
    implementations, RDP and SG do not consume them: callers must collision-
    check any changed path before navigation.
    """

    def __init__(self):
        self.obstacles = np.empty((0, 3))

    def set_obstacles(self, obstacles):
        self.obstacles = as_path(obstacles)

    def process(self, path):
        raise NotImplementedError
