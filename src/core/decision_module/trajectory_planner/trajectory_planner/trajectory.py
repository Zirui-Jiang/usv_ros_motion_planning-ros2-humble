# 只读时间参数轨迹：八列依次是相对秒、弧长米、x米、y米、yaw弧度、曲率1/米、速度米/秒、加速度米/秒²。
"""Time-parameterized trajectory; columns are t, s, x, y, yaw, kappa, v, a.

Time is relative to ``stamp``. Coordinates use ``frame_id``; yaw is radians.
The stored NumPy snapshot is read-only so callbacks cannot mutate a plan in use.
Ported from the original C++ implementation by Yang Haodong (2025).
"""

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus


# 单个插值参考点的具名元组，字段顺序与轨迹数组一致。
class TrajectoryCell(NamedTuple):
    time: float = 0.0
    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    curv: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0


# 复制并冻结 N×8 数组，校验时间严格递增和弧长非递减，stamp 保存生成参考的绝对时间。
class Trajectory:
    def __init__(self, points=None, stamp=0.0, frame_id="", goal=None):
        values = np.asarray([] if points is None else points, dtype=float)
        if values.size == 0:
            values = np.empty((0, 8), dtype=float)
        if values.ndim != 2 or values.shape[1] != 8:
            raise ValueError("trajectory points must have shape (N, 8)")
        if not np.isfinite(values).all() or not np.isfinite(stamp):
            raise ValueError("trajectory and stamp must be finite")
        if len(values):
            if np.any(values[:, :2] < 0):
                raise ValueError("relative time and arc length must be nonnegative")
            if np.any(np.diff(values[:, 0]) <= 0):
                raise ValueError("trajectory time must be strictly increasing")
            if np.any(np.diff(values[:, 1]) < 0):
                raise ValueError("trajectory arc length must be nondecreasing")
        if goal is not None:
            target = np.asarray(goal, dtype=float)
            if target.shape != (2,) or not np.isfinite(target).all():
                raise ValueError("goal must be a finite (x, y) pair")
            goal = tuple(float(v) for v in target)
        self.points = values.copy()
        self.points.setflags(write=False)
        self.stamp = float(stamp)
        self.frame_id = str(frame_id)
        self.goal = goal

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return (TrajectoryCell(*row) for row in self.points)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return Trajectory(self.points[index], self.stamp, self.frame_id, self.goal)
        return TrajectoryCell(*self.points[index])

    def empty(self):
        return len(self) == 0

    # 按相对时间秒查询预瞄点，查询范围外取首尾参考。
    def reference_at_time(self, time):
        return self._reference(time, 0)

    # 按累计弧长米查询，可处理停车期间重复的弧长值。
    def reference_at_distance(self, distance):
        return self._reference(distance, 1)

    # 搜索上下包围点后线性插值；航向独立按最短角弧插值，避免 ±π 跳变。
    def _reference(self, value, column):
        if not np.isfinite(value) or value < 0:
            raise ValueError("reference must be finite and nonnegative")
        if self.empty():
            raise ValueError("cannot query an empty trajectory")
        axis = self.points[:, column]
        if value <= axis[0]:
            return self[0]
        if value >= axis[-1]:
            return self[-1]
        # The lower bracket is i-1, not i (the C++ lower_bound bug).
        upper = int(np.searchsorted(axis, value, side="right"))
        lower = upper - 1
        if value == axis[lower]:
            return self[lower]
        fraction = (value - axis[lower]) / (axis[upper] - axis[lower])
        row = self.points[lower] + fraction * (self.points[upper] - self.points[lower])
        row[4] = wrap_angle(self.points[lower, 4] + fraction * wrap_angle(
            self.points[upper, 4] - self.points[lower, 4]))
        row[column] = value
        return TrajectoryCell(*row)


# 全局搜索完成后，把轨迹时钟改到当前时刻。相对时间仍从 0 起算，避免搜索耗时把轨迹提前判过期。
def restart_trajectory_clock(trajectory, now):
    """Start the trajectory clock when the global search has finished."""
    if trajectory is None or trajectory.empty() or not np.isfinite(now):
        return trajectory
    return Trajectory(trajectory.points, now, trajectory.frame_id, trajectory.goal)


# 将状态、轨迹、诊断信息和算法耗时一起返回调用者。
@dataclass(frozen=True)
class TrajectoryResult:
    status: AlgorithmStatus
    trajectory: Trajectory
    message: str = ""
    elapsed: float = 0.0
