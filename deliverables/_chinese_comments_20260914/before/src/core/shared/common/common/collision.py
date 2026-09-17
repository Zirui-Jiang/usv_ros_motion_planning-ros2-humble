"""Conservative rectangular footprint checks against raw Nav2 costmap cells.

Inflation values 1..252 are traversal costs. A physical footprint is tested
against complete lethal (254) and unknown (255) cell squares, rather than
applying the global planner's soft-cost preference to the vessel centre.
The 253 inscribed band is already an inflation of lethal cells; adding the
whole footprint to that band would inflate the same geometry twice.
"""
import math
import numpy as np
from .math_helper import wrap_angle


def _pose_clear(grid, x, y, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    ac, ass = abs(c), abs(s)
    extent_x = half_length * ac + half_width * ass
    extent_y = half_length * ass + half_width * ac
    resolution = grid.resolution
    x0 = math.floor(math.nextafter((x - extent_x - grid.origin_x) / resolution, -math.inf))
    x1 = math.floor((x + extent_x - grid.origin_x) / resolution)
    y0 = math.floor(math.nextafter((y - extent_y - grid.origin_y) / resolution, -math.inf))
    y1 = math.floor((y + extent_y - grid.origin_y) / resolution)
    if x0 < 0 or y0 < 0 or x1 >= grid.width or y1 >= grid.height:
        return False
    iy, ix = np.nonzero(grid.data[y0:y1 + 1, x0:x1 + 1] >= 254)
    if not len(ix):
        return True
    dx = grid.origin_x + (ix + x0 + .5) * resolution - x
    dy = grid.origin_y + (iy + y0 + .5) * resolution - y
    cell_half = .5 * resolution
    # Separating-axis theorem: the axes of both rectangles are sufficient.
    overlap = ((np.abs(dx) <= extent_x + cell_half)
               & (np.abs(dy) <= extent_y + cell_half)
               & (np.abs(c * dx + s * dy) <= half_length + cell_half * (ac + ass))
               & (np.abs(-s * dx + c * dy) <= half_width + cell_half * (ac + ass)))
    return not bool(np.any(overlap))


def footprint_path_is_clear(grid, poses, length=2.2, width=1.2, sampling_distance=.05):
    """Check the interpolated pose path, including rotation and cell area.

    At most ``sampling_distance`` of corner motion separates two samples.
    Half that bound pads each sampled footprint to cover intervening poses.
    This bounds the stated straight-position/shortest-angle interpolation,
    not an unobserved vessel motion between message timestamps.
    """
    try:
        points = np.asarray(poses, dtype=float)
        if (points.ndim != 2 or not len(points) or points.shape[1] != 3
                or not np.isfinite(points).all()
                or not all(math.isfinite(v) and v > 0 for v in (length, width, sampling_distance))):
            return False
        half_length, half_width = .5 * length, .5 * width
        radius = math.hypot(half_length, half_width)
        if not _pose_clear(grid, *points[0], half_length, half_width):
            return False
        for start, finish in zip(points[:-1], points[1:]):
            delta = finish[:2] - start[:2]
            angle = wrap_angle(finish[2] - start[2])
            motion_bound = float(np.linalg.norm(delta)) + radius * abs(angle)
            steps = max(1, math.ceil(motion_bound / min(sampling_distance, .25 * grid.resolution)))
            if steps > 100000:
                return False
            pad = .5 * motion_bound / steps
            for fraction in np.linspace(0., 1., steps + 1):
                xy = start[:2] + fraction * delta
                if not _pose_clear(grid, *xy, start[2] + fraction * angle,
                                   half_length + pad, half_width + pad):
                    return False
        return True
    except (ValueError, TypeError, OverflowError):
        return False
