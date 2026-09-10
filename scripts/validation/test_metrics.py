import math
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("acceptance_metrics", Path(__file__).with_name("metrics.py"))
metrics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(metrics)


def test_rectangle_corner_collision_detected_when_centre_circle_would_miss():
    obstacle = {"x": 1.2, "y": 0.7, "radius": 0.2}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) < 0
    assert math.hypot(1.2, 0.7) > obstacle["radius"]


def test_oriented_footprint_changes_clearance():
    obstacle = {"x": 2.0, "y": 0, "radius": 0.5}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) == pytest.approx(0.4)
    assert metrics.rectangle_circle_clearance((0, 0, math.pi / 2), obstacle) == pytest.approx(0.9)


def test_swept_collision_detects_obstacle_between_disjoint_samples():
    obstacle = {"x": 2, "y": 0, "radius": 0.3}
    assert metrics.rectangle_circle_clearance((0, 0, 0), obstacle) > 0
    assert metrics.rectangle_circle_clearance((4, 0, 0), obstacle) > 0
    assert metrics.swept_clearance((0, 0, 0), (4, 0, 0), obstacle) < 0


def test_polyline_handles_corner_duplicate_and_missing_points():
    assert metrics.point_to_polyline_distance((1, 1), [[0, 0], [0, 0], [2, 0], [2, 2]]) == 1
    assert metrics.point_to_polyline_distance((1, 1), []) is None


def test_verdict_never_passes_unobserved_arrival_or_missing_path():
    thresholds = {"maximum_cross_track_m": 4, "minimum_clearance_m": 0.2}
    assert not metrics.trial_verdict(False, [0.1], None, True, thresholds, False)["passed"]
    assert not metrics.trial_verdict(True, [], None, True, thresholds, False)["passed"]
    assert metrics.trial_verdict(True, [0.1], 0.3, True, thresholds, True)["passed"]
