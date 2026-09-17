"""Iterative Ramer-Douglas-Peucker simplification with optional densification."""

from __future__ import annotations

import math
import numpy as np
from .path_processor import PathProcessor, as_path


def ramer_douglas_peucker(path, epsilon: float = 0.0, max_interval: float | None = None):
    """Simplify by point-to-segment distance in metres.

    ``max_interval=None`` disables interpolation. A supplied value must be
    positive; the old default zero divided by zero. Finite intervals preserve
    the original ``floor(length / interval) + 1`` subdivision policy and
    interpolate heading along the shortest angular arc.
    """
    if not np.isfinite(epsilon) or epsilon < 0:
        raise ValueError("epsilon must be finite and nonnegative")
    if max_interval is not None and (not np.isfinite(max_interval) or max_interval <= 0):
        raise ValueError("max_interval must be positive and finite")
    points = as_path(path)
    if len(points) < 3:
        retained = points
    else:
        kept = {0, len(points) - 1}
        stack = [(0, len(points) - 1)]
        while stack:
            first, last = stack.pop()
            if last - first < 2:
                continue
            offsets = points[first + 1:last, :2] - points[first, :2]
            segment = points[last, :2] - points[first, :2]
            square = float(np.dot(segment, segment))
            if square <= 1e-24:
                distances = np.linalg.norm(offsets, axis=1)
            else:
                t = np.clip(offsets @ segment / square, 0.0, 1.0)
                distances = np.linalg.norm(offsets - t[:, None] * segment, axis=1)
            index = int(np.argmax(distances))
            if distances[index] > epsilon:
                split = first + 1 + index
                kept.add(split)
                stack.extend(((first, split), (split, last)))
        retained = points[sorted(kept)]
    if max_interval is None or len(retained) < 2:
        return retained.copy()
    result = [retained[0].copy()]
    for first, last in zip(retained[:-1], retained[1:]):
        length = np.linalg.norm(last[:2] - first[:2])
        ratio = float(length) / float(max_interval)
        if not math.isfinite(ratio) or len(result) + ratio + 1 > 2_000_000:
            raise ValueError("densification would exceed two million points")
        segments = int(math.floor(ratio)) + 1
        angular_delta = (last[2] - first[2] + math.pi) % (2 * math.pi) - math.pi
        for index in range(1, segments):
            fraction = index / segments
            point = first + fraction * (last - first)
            point[2] = (first[2] + fraction * angular_delta + math.pi) % (2 * math.pi) - math.pi
            result.append(point)
        result.append(last.copy())
    return np.asarray(result, dtype=float)


class RDPPathProcessor(PathProcessor):
    def __init__(self, delta: float = 0.0, max_interval: float | None = None):
        super().__init__()
        # Validate constructor parameters even before the first process call.
        ramer_douglas_peucker([], delta, max_interval)
        self.delta, self.max_interval = delta, max_interval

    def process(self, path):
        return ramer_douglas_peucker(path, self.delta, self.max_interval)
