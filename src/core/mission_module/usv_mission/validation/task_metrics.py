# 无 ROS 的验收几何与统计：米制航迹偏差、矩形船体与圆柱净距，以及边界停止判据。
"""ROS-free acceptance geometry. Distances use metres; angles use radians."""

import math


# 把角度差包到 [-pi, pi)，用于最短方向插值。
def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# 把点投影到每条有限线段并取最小距离；空路径用 None 表示缺少参考。
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


# 把障碍中心转进船体系，计算矩形到圆的有符号距离：正为分离、负为重叠。
def rectangle_circle_clearance(pose, obstacle, length=2.2, width=1.2):
    """Signed distance between an oriented rectangle and a circular cylinder.

    Positive means separation, zero tangency, negative overlap. This checks the
    full vessel footprint, including corners, rather than only its centre.
    """
    dx, dy = obstacle["x"] - pose[0], obstacle["y"] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    qx = abs(c * dx + s * dy) - length / 2
    qy = abs(-s * dx + c * dy) - width / 2
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - obstacle["radius"]


# 位置线性、航向最短角插值后采样，并扣除间隔误差上界；不重建未观测动力学。
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
    return minimum - 0.5 * (distance + corner_radius * abs(angle)) / count


# 过滤缺失和非有限观测后计算均值、RMS、95 分位和最大值，空样本保留缺失。
def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return {"count": 0, "mean": None, "rms": None, "p95": None, "max": None}
    return {"count": len(values), "mean": sum(values) / len(values),
            "rms": math.sqrt(sum(v * v for v in values) / len(values)),
            "p95": values[max(0, math.ceil(0.95 * len(values)) - 1)], "max": values[-1]}


# 综合实际到达、命令合法性、航迹误差及整船障碍间距形成带理由的判定。
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


# 负扫掠间距或航迹偏差越界时停止采集，与通过阈值的评分逻辑分开。
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
