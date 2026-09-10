"""Ordered integer-evidence LiDAR mapping, independent of ROS.

Unknown cells start at -1. A hit adds 55 (first hit: 54), while each
traversing beam subtracts 25. These scores are not calibrated probabilities.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GridConfig:
    width: int = 1024
    resolution: float = 0.5
    offset_x: float = 0.0
    offset_y: float = 0.0

    def __post_init__(self):
        if isinstance(self.width, bool) or not isinstance(self.width, int) or self.width <= 0:
            raise ValueError("width must be a positive integer")
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        if not all(math.isfinite(v) for v in (self.offset_x, self.offset_y)):
            raise ValueError("map offsets must be finite")


@dataclass(frozen=True)
class UpdateStats:
    input_beams: int
    valid_beams: int
    hit_beams: int
    invalid_beams: int
    updated_cells: int


def _trace_rays(data, width, x0, y0, end_x, end_y, hits):
    """Clip continuous grid coordinates, then visit integer cells in order.

    The implementation is deliberately scalar for repeated cell updates.
    Keeping this function numeric-only also allows an optional JIT backend.
    """
    upper = np.nextafter(float(width), -np.inf)
    updates = 0
    for index in range(len(end_x)):
        ex, ey = end_x[index], end_y[index]
        vx, vy = ex - x0, ey - y0
        entry, leave = 0.0, 1.0
        intersects = True
        # Liang-Barsky clipping prevents both negative indexing and row wrap.
        for axis in range(2):
            start = x0 if axis == 0 else y0
            delta = vx if axis == 0 else vy
            if delta == 0.0:
                if start < 0.0 or start > upper:
                    intersects = False
                    break
            else:
                a, b = -start / delta, (upper - start) / delta
                if a > b:
                    a, b = b, a
                entry, leave = max(entry, a), min(leave, b)
                if entry > leave:
                    intersects = False
                    break
        if not intersects:
            continue
        x = int(math.floor(min(upper, max(0.0, x0 + entry * vx))))
        y = int(math.floor(min(upper, max(0.0, y0 + entry * vy))))
        target_x = int(math.floor(min(upper, max(0.0, x0 + leave * vx))))
        target_y = int(math.floor(min(upper, max(0.0, y0 + leave * vy))))
        dx, dy = abs(target_x - x), abs(target_y - y)
        sx, sy = (1 if x < target_x else -1), (1 if y < target_y else -1)
        error = dx - dy
        while x != target_x or y != target_y:
            cell = y * width + x
            value = int(data[cell]) - 25
            data[cell] = value if value > 0 else 0
            updates += 1
            twice_error = 2 * error
            if twice_error > -dy:
                error -= dy
                x += sx
            if twice_error < dx:
                error += dx
                y += sy
        real_hit = hits[index] and 0.0 <= ex < width and 0.0 <= ey < width
        cell = target_y * width + target_x
        if real_hit:
            value = int(data[cell]) + 55
            data[cell] = value if value < 100 else 100
            updates += 1
        elif leave < 1.0:
            # The physical ray continues outside the map: clear the border
            # cell, without converting its off-map endpoint into a hit.
            value = int(data[cell]) - 25
            data[cell] = value if value > 0 else 0
            updates += 1
    return updates


class GridMapper:
    def __init__(self, config=None, *, backend="python"):
        self.config = config if config is not None else GridConfig()
        self.origin_x = -self.config.width * self.config.resolution / 2.0 + self.config.offset_x
        self.origin_y = -self.config.width * self.config.resolution / 2.0 + self.config.offset_y
        self.data = np.full((self.config.width, self.config.width), -1, dtype=np.int8)
        if backend != "python":
            raise ValueError("only the python reference backend is currently installed")
        self.backend = "python"
        self._kernel = _trace_rays

    def reset(self):
        self.data.fill(-1)

    def world_to_grid(self, x, y):
        return (math.floor((x - self.origin_x) / self.config.resolution),
                math.floor((y - self.origin_y) / self.config.resolution))

    def integrate_scan(self, ranges, angle_min, angle_increment, range_min,
                       range_max, pose_x, pose_y, pose_yaw):
        metadata = (angle_min, angle_increment, range_min, range_max,
                    pose_x, pose_y, pose_yaw)
        if not all(math.isfinite(v) for v in metadata):
            raise ValueError("scan geometry, limits and pose must be finite")
        if range_min < 0.0 or range_max <= range_min:
            raise ValueError("scan limits must satisfy 0 <= range_min < range_max")
        values = np.asarray(ranges, dtype=np.float64)
        if values.ndim != 1:
            raise ValueError("ranges must be a one-dimensional sequence")
        hit = np.isfinite(values) & (values >= range_min) & (values < range_max)
        clear = np.isposinf(values) | (np.isfinite(values) & (values >= range_max))
        valid = hit | clear
        with np.errstate(over="ignore", invalid="ignore"):
            angles = pose_yaw + angle_min + np.arange(values.size) * angle_increment
            lengths = np.where(hit, values, min(range_max, 30.0))
            end_x = (pose_x + lengths * np.cos(angles) - self.origin_x) / self.config.resolution
            end_y = (pose_y + lengths * np.sin(angles) - self.origin_y) / self.config.resolution
        valid &= np.isfinite(end_x) & np.isfinite(end_y)
        start_x = (pose_x - self.origin_x) / self.config.resolution
        start_y = (pose_y - self.origin_y) / self.config.resolution
        if not math.isfinite(start_x) or not math.isfinite(start_y):
            raise ValueError("pose lies outside representable grid coordinates")
        count = int(np.count_nonzero(valid))
        updates = self._kernel(self.data.reshape(-1), self.config.width, start_x, start_y,
                               end_x[valid], end_y[valid], hit[valid])
        return UpdateStats(values.size, count, int(np.count_nonzero(hit & valid)),
                           values.size - count, int(updates))
