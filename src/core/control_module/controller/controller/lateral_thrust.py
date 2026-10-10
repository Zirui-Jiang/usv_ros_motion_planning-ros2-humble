# 接入 0924 横向控制器：艇到航线最近点的横向偏差 → 横向力 → 归一化侧推命令。
# 横向力由谁执行由船模决定：两台全回转桨时在推力输出口分给两台桨，带独立侧推器时直接给该推进器。
"""Adapter from the delivered 0924 lateral controller to a normalized lateral command.

The delivered core is unmodified and keeps its convention: distances and force
are positive to starboard. ROS body y and the lateral command are positive to
port, so the measured offset keeps its magnitude under the changed viewpoint
and the returned force is negated here. The command is the force divided by
``model.max_lateral_force``; ``ThrusterOutput`` allocates it to two
full-rotatable thrusters, or it drives a separate lateral thruster directly.

The measured offset is the body-lateral component of the vector from the vessel
to the closest point of the global route; the desired offset is zero. The
decision module sends the part of the route near the vessel with every
reference (``Trajectory.route``), and the offset is recomputed from the latest
measured pose on every control tick. A reference without a route gives no
lateral thrust. Only NAVIGATE uses this loop: braking and holding commands keep
``lateral == 0`` and their stopping-tail prediction is unchanged.
"""
from dataclasses import replace
import math
from typing import NamedTuple

import numpy as np

from common.models import AlgorithmStatus
from .adrc_core import LateralController
from .controller import stop


# 航线上的一个点，坐标系与船舶状态相同，单位米。
class RoutePoint(NamedTuple):
    x: float
    y: float


# 在折线上找离 (x, y) 最近的点：逐段投影并夹在线段两端之间；单点航线返回该点，空航线返回 None。
def closest_route_point(route, x, y):
    """Return the point of the polyline ``route`` (M, 2) closest to ``(x, y)``."""
    points = np.asarray(route, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        return None
    position = np.array([x, y], dtype=float)
    if not np.isfinite(points).all() or not np.isfinite(position).all():
        raise ValueError("route and position must be finite")
    if len(points) == 1:
        return RoutePoint(*points[0])
    start, delta = points[:-1], np.diff(points, axis=0)
    squared = np.sum(delta * delta, axis=1)
    # 零长度线段退化为它的起点，避免除零。
    safe = np.where(squared > 0, squared, 1.)
    fraction = np.clip(np.sum((position - start) * delta, axis=1) / safe, 0., 1.)
    fraction = np.where(squared > 0, fraction, 0.)
    candidates = start + fraction[:, None] * delta
    nearest = int(np.argmin(np.sum((candidates - position) ** 2, axis=1)))
    return RoutePoint(*candidates[nearest])


# 交付横向控制器的全部键原样透传；适配层只增加启用开关与观测间隔上限（秒）。
class LateralThrust:
    DEFAULTS = {"enabled": False, "max_sample_gap": .5, **LateralController.DEFAULTS}

    # 启动即核验：启用时船模须能产生横向力，且横向力限幅含零并不超过 max_lateral_force。
    def __init__(self, model, config=None):
        supplied = dict(config or {})
        unknown = set(supplied) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown lateral controller config: {sorted(unknown)}")
        cfg = {**self.DEFAULTS, **supplied}
        if type(cfg["enabled"]) is not bool:
            raise ValueError("lateral controller enabled must be a bool")
        self.enabled = cfg.pop("enabled")
        self.max_sample_gap = cfg.pop("max_sample_gap")
        if (type(self.max_sample_gap) not in (int, float) or not math.isfinite(self.max_sample_gap)
                or self.max_sample_gap <= 0):
            raise ValueError("lateral controller max_sample_gap must be finite and positive")
        for key, value in cfg.items():
            if type(value) not in (int, float):
                raise ValueError(f"lateral controller {key} must be numeric")
        # The delivered constructor validates finiteness, ordering and gain signs.
        self.core = LateralController(cfg)
        self.max_force = float(model.max_lateral_force)
        if self.enabled:
            if not math.isfinite(self.max_force) or self.max_force <= 0:
                raise ValueError("lateral controller requires a vessel that can produce lateral force "
                                 "(dynamics.max_lateral_force > 0)")
            lower, upper = self.core.config["force_min"], self.core.config["force_max"]
            # A limit beyond the normalization force would be clipped here
            # without the core's anti-windup noticing the saturation.
            if not -self.max_force <= lower < 0 < upper <= self.max_force:
                raise ValueError(f"lateral force limits must contain zero and stay within "
                                 f"dynamics.max_lateral_force {self.max_force:.3f} N")
        self.reset()

    # 新任务、暂停跟踪或观测中断后调用；下一有效观测按首拍处理。
    def reset(self):
        self.core.reset()
        self.last_stamp = None
        self.last_output = 0.

    # 返回归一化侧推命令，向左舷为正；target 为航线上离艇最近的点，位置与 state 同一坐标系。
    def compute(self, state, target):
        """Return the normalized lateral command in [-1, 1], positive to port."""
        if not self.enabled:
            # Mode 0 of the delivered controller: reset and output no force.
            self.core.compute(0, 0.)
            return 0.
        stamp = float(state.stamp)
        step = self.core.config["sample_time"]
        if self.last_stamp is not None:
            elapsed = stamp - self.last_stamp
            if elapsed < 0:
                raise ValueError("measurement timestamp moved backwards")
            if elapsed == 0:
                return self.last_output
            if elapsed > self.max_sample_gap + 1e-9:
                self.core.reset()
            else:
                step = elapsed
        dx, dy = target.x - state.x, target.y - state.y
        # 航线点在艇体左侧为正（ROS 艇体系 +y）；此时艇位于航线右侧同样距离，即交付约定的右偏量。
        offset = -math.sin(state.psi) * dx + math.cos(state.psi) * dy
        force_starboard = self.core.compute(1, 0., offset, step)
        output = -force_starboard / self.max_force
        if not math.isfinite(output):
            raise ValueError("lateral controller produced a nonfinite command")
        self.last_stamp = stamp
        self.last_output = max(-1., min(1., output))
        return self.last_output


# 在任一航向/航速控制器外叠加横向环：内层只管左右桨，横向环只填侧推分量，互不改写。
class LateralAssistedController:
    def __init__(self, inner, lateral):
        self.inner, self.lateral = inner, lateral

    # 配置、在线调参等其余接口沿用内层控制器；自身两个成员缺失时不再转发，避免递归。
    def __getattr__(self, name):
        if name in ("inner", "lateral"):
            raise AttributeError(name)
        return getattr(self.inner, name)

    def reset(self):
        self.inner.reset()
        self.lateral.reset()

    # 内层按各自规则保留或清除历史；横向环在停止跟踪时一律从首拍重新开始。
    def on_control_hold(self):
        self.inner.on_control_hold()
        self.lateral.reset()

    # 只有内层给出正常跟踪命令且参考带有航线时才叠加侧推；到达、减速等待、失败和无航线都保持零侧推。
    def compute(self, state, trajectory, dt):
        command = self.inner.compute(state, trajectory, dt)
        if command.status != AlgorithmStatus.SUCCESS:
            self.lateral.reset()
            return command
        try:
            target = closest_route_point(trajectory.route, state.x, state.y)
            if target is None:
                self.lateral.reset()
                return command
            return replace(command, lateral=self.lateral.compute(state, target))
        except (ArithmeticError, AttributeError, TypeError, ValueError) as error:
            self.reset()
            return stop(AlgorithmStatus.INVALID_INPUT, str(error))
