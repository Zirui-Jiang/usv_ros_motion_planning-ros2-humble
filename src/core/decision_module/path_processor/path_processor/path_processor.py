# 可选路径处理器的数组校验与基类：保存障碍诊断数据，具体简化/平滑仍需调用方重新碰撞检查。
"""Shared validation and obstacle storage for optional path processors."""

import numpy as np


# 将二维位置补航向为 N×3 副本，统一拒绝非法维度与非有限值。
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
    """Processors return an owned path; constraints depend on the implementation.

    Stored obstacles remain available for caller diagnostics. As in the old
    implementations, RDP and SG do not consume them: callers must collision-
    check any changed path before navigation. Shortcut processing instead uses
    an explicitly supplied collision callback, not this stored obstacle array.
    """

    def __init__(self):
        self.obstacles = np.empty((0, 3))

    # 保存一份独立障碍数组；RDP 与 SG 当前不消费该数组来约束几何变换。
    def set_obstacles(self, obstacles):
        self.obstacles = as_path(obstacles)

    # 基类仅定义处理接口，实际实现由具体路径处理器提供。
    def process(self, path):
        raise NotImplementedError
