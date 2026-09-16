# 路径几何工具：地图坐标下的 N×3 位姿路径转换、二维栅格索引、按弧长重采样与最近路点剪枝。
"""Path geometry operations; angles are radians in the map frame."""

from __future__ import annotations

import math
import numpy as np


# 接收 N×2 或 N×3 数组，缺省航向补零并复制数据，后续处理不修改调用方输入。
def as_path(path) -> np.ndarray:
    """Return an owned finite N x 3 path, accepting N x 2 position arrays."""
    points = np.asarray(path, dtype=float)
    if points.size == 0:
        return np.empty((0, 3), dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3):
        raise ValueError("path must have shape (N, 2) or (N, 3)")
    if not np.isfinite(points).all():
        raise ValueError("path must contain finite values")
    if points.shape[1] == 2:
        points = np.column_stack((points, np.zeros(len(points))))
    return points.copy()


# 行优先展开为 x + width*y，先分别检查二维边界以防跨行索引别名。
def grid_to_index(x: int, y: int, width: int, height: int) -> int:
    if any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) for v in (x, y, width, height)):
        raise ValueError("grid dimensions and coordinates must be integers")
    if width <= 0 or height <= 0 or not (0 <= x < width and 0 <= y < height):
        raise ValueError("grid coordinate is outside the map")
    return int(x + width * y)


# 用取模恢复列、整除恢复行，并拒绝越界索引及非整数输入。
def index_to_grid(index: int, width: int, height: int) -> tuple[int, int]:
    if any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) for v in (index, width, height)):
        raise ValueError("grid dimensions and index must be integers")
    if width <= 0 or height <= 0 or not 0 <= index < width * height:
        raise ValueError("grid index is outside the map")
    return int(index % width), int(index // width)


# 内部路点使用三点圆切线估计航向；近共线或重合时回退，并令切线指向后继点。
def _heading(previous: np.ndarray, point: np.ndarray, following: np.ndarray) -> float:
    """Circle tangent at an interior point, as in the original resampler."""
    a, b = previous - point, following - point
    cross = a[0] * b[1] - a[1] * b[0]
    if abs(cross) <= 1e-12 * max(1.0, np.linalg.norm(a) * np.linalg.norm(b)):
        tangent = following - previous
        if np.linalg.norm(tangent) <= 1e-12:
            tangent = np.array([-a[1], a[0]])
    else:
        aa, bb = np.dot(a, a), np.dot(b, b)
        center = np.array([aa * b[1] - bb * a[1], a[0] * bb - b[0] * aa]) / (2 * cross)
        tangent = np.array([center[1], -center[0]])
    if np.dot(tangent, b) < 0:
        tangent *= -1
    return math.atan2(tangent[1], tangent[0])


# 按累计弧长以米为间距插值，保留首尾完整位姿；相邻重复坐标先合并。
def resample(path, spacing: float) -> np.ndarray:
    """Interpolate by arc length in metres, preserving both endpoint headings.

    Nonpositive spacing fails explicitly instead of entering the old infinite
    loop. Consecutive duplicate positions are removed before interpolation.
    """
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be positive and finite")
    points = as_path(path)
    if len(points) <= 1:
        return points
    distances = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(distances)]
    total = cumulative[-1]
    if total <= 1e-12:
        return points
    # Retain the final representative of duplicate-position runs.
    keep = np.r_[np.diff(cumulative) > 1e-12, True]
    unique, cumulative = points[keep], cumulative[keep]
    ratio = float(total) / float(spacing)
    # 拒绝异常细采样导致的巨量分配，以免配置错误耗尽内存。
    if not math.isfinite(ratio) or ratio > 2_000_000:
        raise ValueError("resampling would exceed two million points")
    count = int(math.ceil(ratio))
    samples = np.r_[np.arange(count, dtype=float) * spacing, total]
    result = np.zeros((len(samples), 3), dtype=float)
    result[:, 0] = np.interp(samples, cumulative, unique[:, 0])
    result[:, 1] = np.interp(samples, cumulative, unique[:, 1])
    result[0], result[-1] = points[0], points[-1]
    for i in range(1, len(result) - 1):
        result[i, 2] = _heading(result[i - 1, :2], result[i, :2], result[i + 1, :2])
    return result


# 保留最近路点之后的路径，距离相等时选最后一点；可按路径距离向后多保留一段。
def prune(path, state, prune_distance: float = 0.0) -> np.ndarray:
    """Remove points before the nearest waypoint; ties choose the last point.

    This retains the original nearest-waypoint policy. ``prune_distance`` can
    retain an optional distance behind that waypoint, measured along the path.
    """
    points = as_path(path)
    if not np.isfinite(prune_distance) or prune_distance < 0:
        raise ValueError("prune_distance must be finite and nonnegative")
    if not len(points):
        return points
    position = np.asarray((state.x, state.y) if hasattr(state, "x") else state[:2], dtype=float)
    if position.shape != (2,) or not np.isfinite(position).all():
        raise ValueError("state position must be finite")
    squared = np.sum((points[:, :2] - position) ** 2, axis=1)
    nearest = len(points) - 1 - int(np.argmin(squared[::-1]))
    retained = 0.0
    while nearest > 0 and retained < prune_distance:
        retained += np.linalg.norm(points[nearest, :2] - points[nearest - 1, :2])
        nearest -= 1
    return points[nearest:].copy()
