# 不依赖 ROS 的验收几何计算；长度用米、角度用弧度，并按完整矩形船体评价圆柱间隙。
"""ROS-free acceptance geometry. Distances use metres; angles use radians."""

import math


# 将角差约束到 [-π, π)，避免跨越正负 π 时产生接近一整圈的虚假转角。
def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# 投影到每条有限线段并取最近距离，端点之外不能按无限直线计算。
def point_to_polyline_distance(point, path):
    if not path:
        return None
    if len(path) == 1:
        return math.hypot(point[0] - path[0][0], point[1] - path[0][1])
    best = math.inf
    for a, b in zip(path[:-1], path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        squared = dx * dx + dy * dy
        fraction = 0.0 if squared == 0 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared))
        best = min(best, math.hypot(point[0] - a[0] - fraction * dx, point[1] - a[1] - fraction * dy))
    return best


# 先把圆心变换到船体系，再求圆与有向矩形的有符号间隙：正为分离、负为重叠。
def rectangle_circle_clearance(pose, obstacle, length=2.2, width=1.2):
    """Signed distance between an oriented rectangle and a circular cylinder.

    Positive means separation, zero tangency, negative overlap. This checks the
    full vessel footprint, including corners, rather than only its centre.
    """
    dx, dy = obstacle["x"] - pose[0], obstacle["y"] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    # 减去矩形半长和半宽，把四角与边界统一为有符号距离问题。
    qx = abs(c * dx + s * dy) - length / 2
    qy = abs(-s * dx + c * dy) - width / 2
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - obstacle["radius"]


# 在相邻位姿间插值并扣除未采样区间的运动上界，覆盖船角转动造成的间隙变化。
def swept_clearance(previous, current, obstacle, length=2.2, width=1.2):
    """Conservative clearance along linear position/shortest-yaw interpolation.

    Samples are no farther than 5 cm / 2 degrees apart. Subtracting half the
    translation-plus-corner-rotation increment bounds unsampled intervals for
    this interpolation. It does not reconstruct unobserved vessel dynamics.
    """
    distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
    angle = wrap_angle(current[2] - previous[2])
    count = max(1, math.ceil(distance / 0.05), math.ceil(abs(angle) / math.radians(2)))
    minimum = math.inf
    for i in range(count + 1):
        fraction = i / count
        pose = (previous[0] + fraction * (current[0] - previous[0]),
                previous[1] + fraction * (current[1] - previous[1]), previous[2] + fraction * angle)
        minimum = min(minimum, rectangle_circle_clearance(pose, obstacle, length, width))
    corner_radius = math.hypot(length / 2, width / 2)
    # 扣除插值步内平移和船角转动的保守上界，不代表重建了未知真实动力学。
    return minimum - 0.5 * (distance + corner_radius * abs(angle)) / count


# 过滤无效观测后计算均值、均方根及分位数；空数据不填零。
def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return {"count": 0, "mean": None, "rms": None, "p95": None, "max": None}
    return {"count": len(values), "mean": sum(values) / len(values),
            "rms": math.sqrt(sum(v * v for v in values) / len(values)),
            "p95": values[max(0, math.ceil(0.95 * len(values)) - 1)], "max": values[-1]}


# 联合到达、推力合法性、横向误差与船体间隙判断试验，而不是只看是否到点。
def trial_verdict(arrived, cross_track, clearance, command_valid, thresholds, has_obstacles):
    reasons = []
    if not arrived:
        reasons.append("arrival_not_observed")
    if not command_valid:
        reasons.append("invalid_or_out_of_range_thruster_command")
    if not cross_track or distribution(cross_track)["max"] > thresholds["maximum_cross_track_m"]:
        reasons.append("cross_track_limit_or_missing_reference")
    if has_obstacles and (clearance is None or clearance < thresholds["minimum_clearance_m"]):
        reasons.append("footprint_clearance_limit_or_missing_observation")
    return {"passed": not reasons, "reasons": reasons}


# 几何边界触发停止采集；仍为正但小于验收裕量的间隙，会验收失败但不触发此边界。
def supervision_stop_reason(clearance, cross_track, cross_track_limit):
    """Stop acquisition at a geometric boundary, independent of human polling.

    Positive clearance below the acceptance margin remains a failed trial but
    does not trigger this emergency stop. A negative conservative swept bound
    triggers a stop even before sampled-pose overlap is independently checked.
    """
    if clearance is not None and clearance < 0:
        return "negative_swept_clearance_bound"
    if cross_track is not None and cross_track > cross_track_limit:
        return "cross_track_boundary_exceeded"
    return None
