"""Angle and geometry helpers shared by pure numerical algorithms."""
import math
import numpy as np


def wrap_angle(angle):
    result = (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi
    return float(result) if result.ndim == 0 else result


pi2pi = wrap_angle


def clamp(value, lower, upper):
    if lower > upper:
        raise ValueError("lower must not exceed upper")
    return min(upper, max(lower, value))


def quaternion_yaw(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))


def path_is_clear(grid, path, threshold=254, unknown_is_obstacle=True):
    """Check every segment with at most half-cell sample spacing."""
    a = np.asarray(path, dtype=float)
    if a.ndim != 2 or a.shape[0] == 0 or a.shape[1] < 2 or not np.isfinite(a).all():
        return False
    if grid.is_blocked(*a[0, :2], threshold, unknown_is_obstacle):
        return False
    for p, q in zip(a[:-1, :2], a[1:, :2]):
        n = max(1, int(math.ceil(np.linalg.norm(q-p) / (0.5*grid.resolution))))
        for f in np.linspace(0.0, 1.0, n+1):
            if grid.is_blocked(*(p + f*(q-p)), threshold, unknown_is_obstacle):
                return False
    return True
