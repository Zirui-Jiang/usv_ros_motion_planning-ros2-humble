# NumPy 最小二乘核实现 Savitzky–Golay 位置平滑，保留端点位姿并重算内部切线航向。
"""Savitzky-Golay position smoothing using a NumPy least-squares kernel."""

from __future__ import annotations

import numpy as np
from .path_processor import PathProcessor, as_path


# 窗口必须为不少于三的奇数，次数小于窗口；短于窗口的路径直接返回独立副本。
def savitzky_golay(path, window_length: int = 7, polyorder: int = 2):
    """Filter positions with endpoint padding and preserve endpoint poses.

    The default kernel matches the existing seven-point filter. Paths shorter
    than the window are returned unchanged. Interior headings follow the
    smoothed tangent; smoothing does not imply collision freedom.
    """
    if isinstance(window_length, bool) or not isinstance(window_length, (int, np.integer)) or window_length < 3 or window_length % 2 != 1:
        raise ValueError("window_length must be an odd integer >= 3")
    if isinstance(polyorder, bool) or not isinstance(polyorder, (int, np.integer)) or not 0 <= polyorder < window_length:
        raise ValueError("polyorder must be an integer in [0, window_length)")
    points = as_path(path)
    if len(points) < window_length:
        return points
    half = window_length // 2
    positions = np.arange(-half, half + 1, dtype=float)
    # Vandermonde 矩阵拟合局部多项式，伪逆首行得到中心位置估计的卷积权重。
    kernel = np.linalg.pinv(np.vander(positions, polyorder + 1, increasing=True))[0]
    # 复制边缘坐标补齐窗口；平滑可改变可通行性，调用方必须复检结果。
    padded = np.pad(points[:, :2], ((half, half), (0, 0)), mode="edge")
    result = points.copy()
    for axis in (0, 1):
        result[:, axis] = np.convolve(padded[:, axis], kernel[::-1], mode="valid")
    result[0], result[-1] = points[0], points[-1]
    tangent = result[2:, :2] - result[:-2, :2]
    valid = np.linalg.norm(tangent, axis=1) > 1e-12
    result[1:-1, 2][valid] = np.arctan2(tangent[valid, 1], tangent[valid, 0])
    return result


# 构造时先验证滤波配置，process 将固定窗口参数传给纯函数。
class SavitzkyGolayPathProcessor(PathProcessor):
    def __init__(self, window_length: int = 7, polyorder: int = 2):
        super().__init__()
        savitzky_golay([], window_length, polyorder)
        self.window_length, self.polyorder = window_length, polyorder

    def process(self, path):
        return savitzky_golay(path, self.window_length, self.polyorder)
