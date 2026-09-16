# 无倒车 Dubins 几何：米制圆弧/直线连接，供 Hybrid A* 的终点解析连接与碰撞采样使用。
"""Forward-only Dubins connections and exact constant-curvature sampling.

The six families are LSL, RSR, LSR, RSL, RLR and LRL. Positions and segment
lengths use metres; headings use radians; left turns have positive curvature.
The unit-radius tangent/circle formulas agree with OMPL's Dubins geometry:
https://ompl.kavrakilab.org/DubinsStateSpace_8cpp_source.html
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


_TAU = 2.0 * math.pi
_ROUND_OFF = 1e-12
_MAX_SAMPLES = 2_000_000


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def _wrap(angle):
    return (angle + math.pi) % _TAU - math.pi


def _mod2pi(angle):
    wrapped = angle % _TAU
    # 浮点舍入产生的 -1e-16 不应变成多转一整圈。
    return 0.0 if _TAU - wrapped <= 1e-14 else wrapped


def _pose(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("pose must be finite (x, y, yaw)")
    return np.array([pose[0], pose[1], _wrap(float(pose[2]))])


@dataclass(frozen=True)
class DubinsPath:
    """Immutable segment modes and physical lengths for one connection."""

    modes: tuple[str, ...]
    lengths: tuple[float, ...]
    radius: float

    def __post_init__(self):
        modes, lengths = tuple(self.modes), tuple(float(s) for s in self.lengths)
        radius = _positive(self.radius, "radius")
        if len(modes) != len(lengths) or any(mode not in ("L", "S", "R") for mode in modes):
            raise ValueError("each length must have a matching L, S or R mode")
        if any(not math.isfinite(s) or s < 0 for s in lengths) or not math.isfinite(sum(lengths)):
            raise ValueError("segment lengths must be finite and nonnegative")
        object.__setattr__(self, "modes", modes)
        object.__setattr__(self, "lengths", lengths)
        object.__setattr__(self, "radius", radius)

    @property
    def length(self):
        """Return the total physical length in metres."""
        return sum(self.lengths)


def advance_pose(pose, curvature, distance) -> np.ndarray:
    """Advance a pose by nonnegative arc length at constant curvature."""
    start = _pose(pose)
    curvature, distance = float(curvature), float(distance)
    if not math.isfinite(curvature) or not math.isfinite(distance) or distance < 0:
        raise ValueError("curvature must be finite and distance finite and nonnegative")
    angle = curvature * distance
    if not math.isfinite(angle):
        raise ValueError("turn angle exceeds the numerical range")
    # 圆弧位移=弦长×中点航向；sinc 形式在曲率接近零时不会做两个近似相等数的减法。
    half_angle = 0.5 * angle
    chord = distance if half_angle == 0 else distance * (math.sin(half_angle) / half_angle)
    heading_mid = float(start[2]) + half_angle
    result = np.array([start[0] + chord * math.cos(heading_mid),
                       start[1] + chord * math.sin(heading_mid),
                       _wrap(float(start[2]) + angle)])
    if not np.isfinite(result).all():
        raise ValueError("motion exceeds the numerical range")
    return result


def dubins_paths(start, goal, radius) -> list[DubinsPath]:
    """Return feasible Dubins families sorted by total physical length."""
    first, last = _pose(start), _pose(goal)
    radius = _positive(radius, "radius")
    dx, dy = float(last[0]) - float(first[0]), float(last[1]) - float(first[1])
    distance = math.hypot(dx, dy) / radius
    if not math.isfinite(distance):
        raise ValueError("normalised endpoint distance exceeds the numerical range")
    if dx == 0 and dy == 0 and abs(_wrap(float(last[2] - first[2]))) <= _ROUND_OFF:
        return [DubinsPath((), (), radius)]

    # 平移、旋转并按半径缩放：起点为 (0,0,alpha)，终点为 (d,0,beta)，圆半径为 1。
    direction = math.atan2(dy, dx)
    alpha, beta = _mod2pi(float(first[2]) - direction), _mod2pi(float(last[2]) - direction)
    sa, ca, sb, cb = math.sin(alpha), math.cos(alpha), math.sin(beta), math.cos(beta)
    paths = []

    def add(word, t, p, q):
        lengths = (_mod2pi(t) * radius, p * radius, _mod2pi(q) * radius)
        if all(math.isfinite(value) for value in lengths):
            paths.append(DubinsPath(tuple(word), lengths, radius))

    # 同向圆的外公切线：中间直线长度就是两圆心距离。
    left_dx, left_dy = distance + sa - sb, cb - ca
    left_distance = math.hypot(left_dx, left_dy)
    left_tangent = math.atan2(left_dy, left_dx)
    if left_distance <= _ROUND_OFF:
        add("LSL", beta - alpha, 0.0, 0.0)
    else:
        add("LSL", left_tangent - alpha, left_distance, beta - left_tangent)

    right_dx, right_dy = distance - sa + sb, ca - cb
    right_distance = math.hypot(right_dx, right_dy)
    right_tangent = math.atan2(right_dy, right_dx)
    if right_distance <= _ROUND_OFF:
        add("RSR", alpha - beta, 0.0, 0.0)
    else:
        add("RSR", alpha - right_tangent, right_distance, right_tangent - beta)

    # 异向圆的内公切线仅在圆心距 >= 2 时存在；(d-2)(d+2) 降低相切附近消减误差。
    inner_dx, inner_dy = distance + sa + sb, -ca - cb
    inner_distance = math.hypot(inner_dx, inner_dy)
    if inner_distance >= 2.0 - _ROUND_OFF:
        p = math.sqrt(max(0.0, inner_distance - 2.0)) * math.sqrt(inner_distance + 2.0)
        tangent = math.atan2(inner_dy, inner_dx) - math.atan2(-2.0, p)
        add("LSR", tangent - alpha, p, tangent - beta)

    inner_dx, inner_dy = distance - sa - sb, ca + cb
    inner_distance = math.hypot(inner_dx, inner_dy)
    if inner_distance >= 2.0 - _ROUND_OFF:
        p = math.sqrt(max(0.0, inner_distance - 2.0)) * math.sqrt(inner_distance + 2.0)
        tangent = math.atan2(inner_dy, inner_dx) - math.atan2(2.0, p)
        add("RSL", alpha - tangent, p, beta - tangent)

    # 三圆弧连接的中间圆必须同时接触两端圆，因此同向圆心距不能大于 4。
    # acos(1-d²/8) 在圆心几乎重合时会消减成 acos(1)，改用等价的 2*asin(d/4)。
    if right_distance <= 4.0 + _ROUND_OFF:
        middle = _TAU - 2*math.asin(min(1.0, right_distance/4.0))
        t = _mod2pi(alpha - right_tangent + 0.5 * middle)
        add("RLR", t, _mod2pi(middle), alpha - beta - t + middle)
    if left_distance <= 4.0 + _ROUND_OFF:
        middle = _TAU - 2*math.asin(min(1.0, left_distance/4.0))
        t = _mod2pi(-alpha + left_tangent + 0.5 * middle)
        add("LRL", t, _mod2pi(middle), beta - alpha - t + middle)
    return sorted(paths, key=lambda path: path.length)


def sample_dubins(start, path: DubinsPath, spacing) -> np.ndarray:
    """Sample every segment, including its end, without endpoint snapping."""
    current = _pose(start)
    if not isinstance(path, DubinsPath):
        raise ValueError("path must be a DubinsPath")
    spacing = _positive(spacing, "spacing")
    counts = []
    total = 1
    for distance in path.lengths:
        ratio = distance / spacing
        if not math.isfinite(ratio) or ratio > _MAX_SAMPLES:
            raise ValueError("Dubins sampling exceeds the 2000000 point limit")
        count = int(math.ceil(ratio)) if distance > 0 else 0
        total += count
        if total > _MAX_SAMPLES:
            raise ValueError("Dubins sampling exceeds the 2000000 point limit")
        counts.append(count)

    result = np.empty((total, 3), dtype=float)
    result[0] = current
    offset = 1
    for mode, distance, count in zip(path.modes, path.lengths, counts):
        if count == 0:
            continue
        curvature = {"L": 1.0, "S": 0.0, "R": -1.0}[mode] / path.radius
        angle = curvature * distance
        if not math.isfinite(angle):
            raise ValueError("turn angle exceeds the numerical range")
        # 每段从自己的起点积分，保留曲率切换点；不把最后一行替换成传入目标。
        distances = np.linspace(distance / count, distance, count)
        angles = curvature * distances
        chords = distances * np.sinc(angles / _TAU)
        headings_mid = current[2] + 0.5 * angles
        segment = result[offset:offset + count]
        segment[:, 0] = current[0] + chords * np.cos(headings_mid)
        segment[:, 1] = current[1] + chords * np.sin(headings_mid)
        segment[:, 2] = (current[2] + angles + math.pi) % _TAU - math.pi
        current = segment[-1].copy()
        offset += count
    if not np.isfinite(result).all():
        raise ValueError("sampled path exceeds the numerical range")
    return result
