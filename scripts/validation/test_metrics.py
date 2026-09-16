# 验证矩形船角、姿态变化、采样间扫掠碰撞和折线路径距离，防止中心点近似漏判。
import math
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("acceptance_metrics", Path(__file__).with_name("metrics.py"))
metrics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(metrics)


# 构造中心圆近似会漏判的船角碰撞，验证完整矩形几何检查。
def test_rectangle_corner_collision_detected_when_centre_circle_would_miss():
    obstacle = {"x": 1.2, "y": 0.7, "radius": 0.2}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) < 0
    assert math.hypot(1.2, 0.7) > obstacle["radius"]


# 只改变艏向也会改变矩形船体间隙，不能把船体当作方向无关的圆。
def test_oriented_footprint_changes_clearance():
    obstacle = {"x": 2.0, "y": 0, "radius": 0.5}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) == pytest.approx(0.4)
    assert metrics.rectangle_circle_clearance((0, 0, math.pi / 2), obstacle) == pytest.approx(0.9)


# 两个端点都安全时，仍应发现位姿间运动穿过的障碍。
def test_swept_collision_detects_obstacle_between_disjoint_samples():
    obstacle = {"x": 2, "y": 0, "radius": 0.3}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) > 0
    assert metrics.rectangle_circle_clearance((4, 0, 0), obstacle) > 0
    assert metrics.swept_clearance((0, 0, 0), (4, 0, 0), obstacle) < 0


# 折线距离计算需处理拐角、重复点和空路径。
def test_polyline_handles_corner_duplicate_and_missing_points():
    assert metrics.point_to_polyline_distance((1, 1), [[0, 0], [0, 0], [2, 0], [2, 2]]) == 1
    assert metrics.point_to_polyline_distance((1, 1), []) is None


# 未观测到到达或没有参考路径时，验收不能默认为通过。
def test_verdict_never_passes_unobserved_arrival_or_missing_path():
    thresholds = {"maximum_cross_track_m": 4, "minimum_clearance_m": 0.2}
    assert not metrics.trial_verdict(False, [0.1], None, True, thresholds, False)["passed"]
    assert not metrics.trial_verdict(True, [], None, True, thresholds, False)["passed"]
    assert metrics.trial_verdict(True, [0.1], 0.3, True, thresholds, True)["passed"]


# 区分触发停止的几何边界和仍为正但不足验收裕量的间隙。
def test_supervision_stops_on_boundary_without_hiding_positive_margin_failures():
    assert metrics.supervision_stop_reason(-0.006092, 0.2, 4.0) == "negative_swept_clearance_bound"
    assert metrics.supervision_stop_reason(0.0, 4.00001, 4.0) == "cross_track_boundary_exceeded"
    assert metrics.supervision_stop_reason(None, None, 4.0) is None
    assert metrics.supervision_stop_reason(0.074153, 4.0, 4.0) is None
    # Positive-but-insufficient margins still fail the ordinary verdict.
    assert not metrics.trial_verdict(True, [0.2], 0.074153, True,
                                    {"maximum_cross_track_m": 4, "minimum_clearance_m": 0.2}, True)["passed"]
