# 纯数值公共几何工具：角度规整、限幅、四元数偏航提取和路径中心线采样碰撞检查。
"""Angle and geometry helpers shared by pure numerical algorithms."""
import math
import numpy as np


# 将标量或数组角度规整到 [-π,π)，标量输入返回 Python float。
def wrap_angle(angle):
    result = (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi
    return float(result) if result.ndim == 0 else result


pi2pi = wrap_angle


# 检查上下界顺序后把数值限制在闭区间。
def clamp(value, lower, upper):
    if lower > upper:
        raise ValueError("lower must not exceed upper")
    return min(upper, max(lower, value))


# 先验证并归一化四元数，再按 ROS 约定提取平面偏航角。
def quaternion_yaw(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))


# 中心线每段最多半格间距采样；该工具不覆盖船体尺寸，也不提供软膨胀区逃逸例外。
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
