"""复用 Hybrid 全局参考几何；当前位置到参考线的运动由局部规划器另行检查。

缓存只沿原直线/圆弧前移，不将实测横向偏差或艏向写进全局路径。
返回 ``None`` 表示调用者应重新全局搜索；超时始终保留 ``TimeoutError``。
直线快捷连接只应在新全局计划产生后、逐个任务航段调用一次。
"""
from __future__ import annotations

import math
import time

import numpy as np

from common.models import GridSnapshot
from .dubins import advance_pose
from .hybrid_collision import HybridCollisionChecker


_ANGLE_TOLERANCE = 1e-6
_POSITION_TOLERANCE = 1e-7
_DISTANCE_EPSILON = 1e-10
_MAX_SHORTCUT_CHECKS = 16
_MAX_SHORTCUT_PROBES = 4096


def _check_deadline(deadline):
    if time.perf_counter() >= deadline:
        raise TimeoutError("Hybrid reference path processing time budget exceeded")


def _wrap(angle):
    return math.remainder(float(angle), math.tau)


def _path(value):
    points = np.asarray(value, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0
            or not np.isfinite(points).all()):
        raise ValueError("path must contain finite (x, y, yaw) rows")
    return points


def _pose(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("pose must be finite (x, y, yaw)")
    return pose


def _number(config, key, default, *, positive=False):
    value = config.get(key, default)
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{key} must be numeric")
    value = float(value)
    if not math.isfinite(value) or value < 0. or (positive and value == 0.):
        raise ValueError(f"{key} must be finite and nonnegative")
    return value


def _checker(grid, config):
    return HybridCollisionChecker(
        grid, factor=config.get("obstacle_factor", .5),
        unknown_is_obstacle=config.get("unknown_is_obstacle", True),
        outline_map=config.get("outline_map", False),
        footprint_length=config.get("footprint_length", 2.2),
        footprint_width=config.get("footprint_width", 1.2),
        footprint_padding=config.get("footprint_padding", 0.),
        collision_step=config.get("collision_step", .2))


def _segments(points, radius, deadline):
    """由圆弦恒等式恢复每段 (曲率, 弧长)，拒绝倒车、折角及超曲率缓存。"""
    segments = []
    for start, end in zip(points[:-1], points[1:]):
        _check_deadline(deadline)
        dx, dy = float(end[0]-start[0]), float(end[1]-start[1])
        chord = math.hypot(dx, dy)
        if not math.isfinite(chord) or chord <= _DISTANCE_EPSILON:
            raise ValueError("cache contains a zero-length or invalid segment")
        angle = _wrap(end[2]-start[2])
        middle_heading = float(start[2])+.5*angle
        if abs(_wrap(middle_heading-math.atan2(dy, dx))) > _ANGLE_TOLERANCE:
            raise ValueError("cache chord and forward tangent headings disagree")
        curvature = 2.*math.sin(.5*angle)/chord
        if abs(curvature) > (1./radius)*(1.+1e-8)+1e-10:
            raise ValueError("cache exceeds the configured curvature bound")
        distance = angle/curvature if curvature != 0. else chord
        if not math.isfinite(distance) or distance <= 0.:
            raise ValueError("cache contains an invalid forward arc")
        segments.append((curvature, distance))
    return segments


def _project_segment(start, end, curvature, distance, limit, position):
    """投影到本段前 limit 米；圆弧使用圆心方向，非弦上线性插值。"""
    dx, dy = float(position[0]-start[0]), float(position[1]-start[1])
    c, s = math.cos(float(start[2])), math.sin(float(start[2]))
    along, across = c*dx+s*dy, -s*dx+c*dy
    if curvature == 0.:
        projected = min(limit, max(0., along))
    else:
        # 在起点船体系内，圆心为 (0,1/k)。此写法避免显式相减巨大圆心。
        turn = math.atan2(curvature*along, 1.-curvature*across)
        projected = turn/curvature
    candidates = [0.]
    if 0. < projected < limit:
        candidates.append(projected)
    candidates.append(limit)
    best = None
    for offset in candidates:
        point = (start if offset == 0. else end if offset == distance
                 else advance_pose(start, curvature, offset))
        separation = math.hypot(float(point[0]-position[0]), float(point[1]-position[1]))
        if best is None or separation < best[0]-_DISTANCE_EPSILON:
            best = (separation, offset, point)
    return best


def _remaining_path(points, segments, state, search_distance, deadline):
    best = None
    consumed = 0.
    for index, (curvature, distance) in enumerate(segments):
        _check_deadline(deadline)
        limit = min(distance, search_distance-consumed)
        separation, offset, point = _project_segment(
            points[index], points[index+1], curvature, distance, limit, state)
        # 等距时保留最早一段，避免闭合/自交路线被投影到后续圈次。
        if best is None or separation < best[0]-_DISTANCE_EPSILON:
            best = (separation, index, offset, point)
        consumed += distance
        if consumed >= search_distance:
            break
    separation, index, offset, point = best
    curvature, distance = segments[index]
    if offset <= _DISTANCE_EPSILON:
        remaining, motions = points[index:].copy(), segments[index:]
    elif distance-offset <= _DISTANCE_EPSILON:
        remaining, motions = points[index+1:].copy(), segments[index+1:]
    else:
        remaining = np.vstack((point, points[index+1:]))
        motions = [(curvature, distance-offset), *segments[index+1:]]
    return remaining, motions, separation, point


def _same_motion(start, end, curvature, distance):
    reconstructed = advance_pose(start, curvature, distance)
    return (math.hypot(*(reconstructed[:2]-end[:2])) <= _POSITION_TOLERANCE
            and abs(_wrap(reconstructed[2]-end[2])) <= _POSITION_TOLERANCE)


def _remaining_is_clear(points, segments, checker, deadline):
    """合并同曲率采样后检查完整剩余圆弧，避免每个密集样本重复包围盒查询。"""
    _check_deadline(deadline)
    if not segments:
        clear = checker.pose_is_clear(points[0])
        _check_deadline(deadline)
        return clear
    start = points[0]
    curvature, distance = segments[0]
    for index, (following_curvature, following_distance) in enumerate(segments[1:], 1):
        _check_deadline(deadline)
        combined_distance = distance+following_distance
        if (math.isclose(curvature, following_curvature, rel_tol=1e-7, abs_tol=1e-10)
                and _same_motion(start, points[index+1], curvature, combined_distance)):
            distance = combined_distance
            continue
        if not checker.motion_is_clear(start, curvature, distance, deadline=deadline):
            return False
        start = points[index]
        curvature, distance = following_curvature, following_distance
    clear = checker.motion_is_clear(start, curvature, distance, deadline=deadline)
    _check_deadline(deadline)
    return clear


def prepare_cached_path(path, state_pose, goal_pose, grid, config, *, deadline=math.inf):
    """返回沿原几何前移且通过当前地图复查的 N×3 缓存，失效则返回 None。

    投影只搜索已缓存起点之后 ``reuse_search_distance`` 米；位置/艏向
    偏差上限分别为 ``reuse_max_deviation``、``reuse_max_heading_error``。
    新起点位于原参考曲线上，局部 LOS 仍需从实测船位生成并独立检查运动。
    """
    _check_deadline(deadline)
    try:
        points, state, goal = _path(path), _pose(state_pose), _pose(goal_pose)
        if not isinstance(grid, GridSnapshot):
            return None
        # 本算法输出已经保存了精确目标；目标变化不能用容差悄悄继续旧任务。
        if (not np.array_equal(points[-1, :2], goal[:2])
                or abs(_wrap(points[-1, 2]-goal[2])) > 1e-10):
            return None
        radius = _number(config, "minimum_turning_radius", 4., positive=True)
        search_distance = _number(config, "reuse_search_distance", 10., positive=True)
        max_deviation = _number(config, "reuse_max_deviation", 2.)
        max_heading_error = _number(config, "reuse_max_heading_error", math.pi/3)
        segments = _segments(points, radius, deadline)
        if segments:
            remaining, motions, separation, projection = _remaining_path(
                points, segments, state, search_distance, deadline)
        else:
            remaining, motions, projection = points.copy(), [], points[0]
            separation = math.hypot(*(state[:2]-projection[:2]))
        if (separation > max_deviation
                or abs(_wrap(state[2]-projection[2])) > max_heading_error):
            return None
        _check_deadline(deadline)
        checker = _checker(grid, config)
        _check_deadline(deadline)
        actual_clear = checker.pose_is_clear(state)
        _check_deadline(deadline)
        if not actual_clear or not _remaining_is_clear(remaining, motions, checker, deadline):
            return None
        return remaining
    except (TypeError, ValueError, OverflowError):
        return None
    finally:
        _check_deadline(deadline)


def _straight_connection(start, end):
    dx, dy = float(end[0]-start[0]), float(end[1]-start[1])
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance) or distance <= _DISTANCE_EPSILON:
        return None
    bearing = math.atan2(dy, dx)
    if (abs(_wrap(start[2]-bearing)) > _ANGLE_TOLERANCE
            or abs(_wrap(end[2]-bearing)) > _ANGLE_TOLERANCE):
        return None
    return bearing, distance


def shortcut_straight_connections(path, grid, config, *, deadline=math.inf):
    """在单个新计划航段内尝试有切向连续性的直连，保留所有输出端点原值。

    每条路径最多 16 次完整运动碰撞检查；另限制候选配对扫描，防止弯曲
    长路径退化为平方复杂度。同向连续直线可只保留两端，必要圆弧不削角。
    """
    _check_deadline(deadline)
    points = _path(path)
    radius = _number(config, "minimum_turning_radius", 4., positive=True)
    try:
        segments = _segments(points, radius, deadline)
    except (ValueError, OverflowError):
        # 此函数不声明原路径有效；无法解释其几何时不尝试改形状。
        result = points.copy()
        _check_deadline(deadline)
        return result
    if len(points) < 3:
        result = points.copy()
        _check_deadline(deadline)
        return result
    checker = _checker(grid, config)
    _check_deadline(deadline)
    # 候选扫描耗尽后仍可在线性时间内找到直线采样段的最远端点。
    straight_ends = list(range(len(points)))
    for index in range(len(segments)-1, -1, -1):
        _check_deadline(deadline)
        if abs(segments[index][0]) <= 1e-10:
            straight_ends[index] = max(index+1, straight_ends[index+1])
    checks, probes = 0, 0

    def clear_connection(first, last):
        nonlocal checks
        geometry = _straight_connection(points[first], points[last])
        if geometry is None or checks >= _MAX_SHORTCUT_CHECKS:
            return False
        bearing, distance = geometry
        checks += 1
        start = (points[first, 0], points[first, 1], bearing)
        clear = (checker.pose_is_clear(points[first]) and checker.pose_is_clear(points[last])
                 and checker.motion_is_clear(start, 0., distance, deadline=deadline))
        _check_deadline(deadline)
        return clear

    kept = [0]
    index = 0
    while index < len(points)-1:
        _check_deadline(deadline)
        if checks >= _MAX_SHORTCUT_CHECKS:
            kept.extend(range(index+1, len(points)))
            break
        following = index+1
        straight_end = straight_ends[index]
        tried_straight_end = False
        for candidate in range(len(points)-1, index+1, -1):
            _check_deadline(deadline)
            if probes >= _MAX_SHORTCUT_PROBES or checks >= _MAX_SHORTCUT_CHECKS:
                break
            probes += 1
            tried_straight_end |= candidate == straight_end
            if clear_connection(index, candidate):
                following = candidate
                break
        if (following == index+1 and straight_end > index+1 and not tried_straight_end
                and clear_connection(index, straight_end)):
            following = straight_end
        kept.append(following)
        index = following
    result = points[kept].copy()
    _check_deadline(deadline)
    return result
