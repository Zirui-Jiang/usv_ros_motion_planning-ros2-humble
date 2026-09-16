"""检查 Hybrid A* 的矩形船体和常曲率前进运动段。

船中心按 ``254 * factor`` 检查软代价；实体船体只与致命单元、按策略
阻塞的未知单元及可选地图描边相交，避免再次膨胀软代价带。圆弧采样
加入半步运动上界裕量，保守覆盖几何运动的采样间隙，不描述水动力误差。
"""

from __future__ import annotations

import math
import time

import numpy as np

from common.models import GridSnapshot
from .dubins import advance_pose


class HybridCollisionChecker:
    """A per-plan checker; masks are owned and never modify the grid snapshot."""

    def __init__(self, grid: GridSnapshot, *, factor=.5, unknown_is_obstacle=True,
                 outline_map=False, footprint_length=2.2, footprint_width=1.2,
                 footprint_padding=0.0, collision_step=.2):
        self.grid = grid
        self.factor = float(factor)
        self.collision_step = float(collision_step)
        length, width, padding = map(float, (footprint_length, footprint_width, footprint_padding))
        if not math.isfinite(self.factor) or not 0.0 < self.factor <= 1.0:
            raise ValueError("factor must be finite and in (0, 1]")
        if not all(math.isfinite(value) and value > 0.0
                   for value in (length, width, self.collision_step)):
            raise ValueError("footprint dimensions and collision_step must be positive and finite")
        if not math.isfinite(padding) or padding < 0.0:
            raise ValueError("footprint_padding must be nonnegative and finite")
        self.half_length = .5 * length + padding
        self.half_width = .5 * width + padding
        self.corner_radius = math.hypot(self.half_length, self.half_width)
        if not math.isfinite(self.corner_radius):
            raise ValueError("padded footprint dimensions are too large")

        costs = grid.data
        self.center_blocked = costs >= 254.0 * self.factor
        self.hard_blocked = costs == 254
        if unknown_is_obstacle:
            self.hard_blocked |= costs == 255
        else:
            self.center_blocked[costs == 255] = False
        if outline_map:
            for blocked in (self.center_blocked, self.hard_blocked):
                blocked[[0, -1], :] = True
                blocked[:, [0, -1]] = True

    def _bounds(self, x, y, extent_x, extent_y):
        """完整单元面积参与碰撞；刚好接触单元边界也纳入检测。"""
        grid = self.grid
        resolution = grid.resolution
        x0 = math.floor(math.nextafter((x - extent_x - grid.origin_x) / resolution, -math.inf))
        x1 = math.floor((x + extent_x - grid.origin_x) / resolution)
        y0 = math.floor(math.nextafter((y - extent_y - grid.origin_y) / resolution, -math.inf))
        y1 = math.floor((y + extent_y - grid.origin_y) / resolution)
        if x0 < 0 or y0 < 0 or x1 >= grid.width or y1 >= grid.height:
            return None
        return x0, x1, y0, y1

    def _hull_is_clear(self, x, y, yaw, padding=0.0):
        """用分离轴定理检测旋转船体矩形与占据单元方块。"""
        c, s = math.cos(yaw), math.sin(yaw)
        ac, ass = abs(c), abs(s)
        half_length = self.half_length + padding
        half_width = self.half_width + padding
        extent_x = half_length * ac + half_width * ass
        extent_y = half_length * ass + half_width * ac
        bounds = self._bounds(x, y, extent_x, extent_y)
        if bounds is None:
            return False
        x0, x1, y0, y1 = bounds
        iy, ix = np.nonzero(self.hard_blocked[y0:y1 + 1, x0:x1 + 1])
        if not len(ix):
            return True
        grid = self.grid
        dx = grid.origin_x + (ix + x0 + .5) * grid.resolution - x
        dy = grid.origin_y + (iy + y0 + .5) * grid.resolution - y
        cell_half = .5 * grid.resolution
        overlap = ((np.abs(dx) <= extent_x + cell_half)
                   & (np.abs(dy) <= extent_y + cell_half)
                   & (np.abs(c * dx + s * dy) <= half_length + cell_half * (ac + ass))
                   & (np.abs(-s * dx + c * dy) <= half_width + cell_half * (ac + ass)))
        return not bool(np.any(overlap))

    def _centre_is_clear(self, x, y, padding=0.0):
        if padding == 0.0:
            cell = self.grid.world_to_map(x, y)
            return cell is not None and not bool(self.center_blocked[cell[1], cell[0]])
        # 半步弧长形成的圆盘被此轴对齐方形包含，故细小的掠角访问也不会漏掉。
        bounds = self._bounds(x, y, padding, padding)
        if bounds is None:
            return False
        x0, x1, y0, y1 = bounds
        return not bool(np.any(self.center_blocked[y0:y1 + 1, x0:x1 + 1]))

    def _motion_region_is_clear(self, start, curvature, distance, centre_padding, hull_padding):
        """只证明空区域可通过；障碍、边界或数值疑义一律交回逐位姿精查。

        任意中心点到圆弧中点的距离不超过半段弧长。再分别加上中心检查
        的 padding、已加 padding 的矩形外接圆半径，所得两个轴对齐方形
        包含原精查访问的全部中心区域和全部旋转船体。必须分别查询软带
        与实体障碍 mask，不能将中心软带再膨胀到船体。
        """
        # 极大角度的浮点三角函数不使用此几何证明，保持原精查的判定。
        if abs(float(start[2])) > math.tau or abs(curvature * distance) > math.tau:
            return False
        try:
            x, y, _ = map(float, advance_pose(start, curvature, .5 * distance))
            # 向外留浮点舍入量，避免包围盒恰落栅格边界时缩入一个单元。
            roundoff = 16.0 * math.ulp(max(abs(x), abs(y), abs(float(start[0])),
                                        abs(float(start[1])), distance))
            centre_extent = .5 * distance + centre_padding + roundoff
            hull_extent = (.5 * distance
                           + math.hypot(self.half_length + hull_padding,
                                        self.half_width + hull_padding) + roundoff)
            for blocked, extent in ((self.center_blocked, centre_extent),
                                    (self.hard_blocked, hull_extent)):
                if not math.isfinite(extent):
                    return False
                bounds = self._bounds(x, y, extent, extent)
                if bounds is None:
                    return False
                x0, x1, y0, y1 = bounds
                # 切片是 view；any 直接扫描既有 mask，不建立整段采样数组。
                if np.any(blocked[y0:y1 + 1, x0:x1 + 1]):
                    return False
            return True
        except (ValueError, TypeError, OverflowError):
            return False

    def pose_is_clear(self, pose) -> bool:
        """严格检查单个位姿；超软阈值起点不采用普通 A* 的软带逃逸。"""
        try:
            point = np.asarray(pose, dtype=float)
            if point.shape != (3,) or not np.isfinite(point).all():
                return False
            x, y, yaw = map(float, point)
            return self._centre_is_clear(x, y) and self._hull_is_clear(x, y, yaw)
        except (ValueError, TypeError, OverflowError):
            return False

    def motion_is_clear(self, start, curvature, distance, deadline=math.inf) -> bool:
        """检查整段精确圆弧；耗时达到规划器给定 deadline 时抛出 TimeoutError。

        弧长步长为 h 时，离最近样本的船中心位移不超过 h/2；船体角点位移
        不超过 (1 + 角点半径 * |曲率|) * h/2。将此界分别加入中心和船体
        检查，覆盖样本之间的运动。裕量可能保守拒绝非常贴近障碍物的路径。
        """
        if time.perf_counter() >= deadline:
            raise TimeoutError("collision checking time budget exceeded")
        try:
            point = np.asarray(start, dtype=float)
            curvature, distance = float(curvature), float(distance)
            if (point.shape != (3,) or not np.isfinite(point).all()
                    or not math.isfinite(curvature) or not math.isfinite(distance) or distance < 0.0):
                return False
            if distance == 0.0:
                return self.pose_is_clear(point)
            corner_speed_bound = 1.0 + self.corner_radius * abs(curvature)
            max_step = min(self.collision_step, .25 * self.grid.resolution) / corner_speed_bound
            if not math.isfinite(corner_speed_bound) or max_step <= 0.0:
                return False
            steps = max(1, math.ceil(distance / max_step))
            arc_step = distance / steps
            centre_padding = .5 * arc_step
            hull_padding = centre_padding * corner_speed_bound
            if time.perf_counter() >= deadline:
                raise TimeoutError("collision checking time budget exceeded")
            region_clear = self._motion_region_is_clear(
                point, curvature, distance, centre_padding, hull_padding)
            if time.perf_counter() >= deadline:
                raise TimeoutError("collision checking time budget exceeded")
            if not region_clear:
                for index in range(steps + 1):
                    if time.perf_counter() >= deadline:
                        raise TimeoutError("collision checking time budget exceeded")
                    # 从同一起点累计弧长，避免大量逐步积分引入误差。
                    pose = advance_pose(point, curvature, distance * (index / steps))
                    x, y, yaw = map(float, pose)
                    if (not self._centre_is_clear(x, y, centre_padding)
                            or not self._hull_is_clear(x, y, yaw, hull_padding)):
                        return False
            if time.perf_counter() >= deadline:
                raise TimeoutError("collision checking time budget exceeded")
            return True
        except (ValueError, TypeError, OverflowError):
            return False
