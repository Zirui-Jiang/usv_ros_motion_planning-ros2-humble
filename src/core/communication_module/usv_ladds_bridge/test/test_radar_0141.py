"""Tests for clustering a planar scan into 0141 radar targets."""

from types import SimpleNamespace
import math

import pytest
from common.geodesy import GeoReference

from usv_ladds_bridge.config import validate_obstacle_payload
from usv_ladds_bridge.radar_0141 import (
    OwnshipPose,
    cluster_scan_hits,
    scan_to_0141_json,
    scan_to_0141_targets,
    yaw_to_heading,
)


ORIGIN = GeoReference(30.5453, 114.392931)
POSE = OwnshipPose(x=23.0, y=-10.0, yaw=0.0)


def _scan(ranges, *, angle_min=0.0, angle_increment=0.01, range_min=0.5, range_max=25.0):
    return SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
    )


def test_heading_east_is_ninety_degrees():
    assert yaw_to_heading(0.0) == 90.0


def test_contiguous_hits_form_one_cluster():
    clusters = cluster_scan_hits(
        [10.0, 10.1, 10.05],
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.5,
        range_max=25.0,
        min_points=3,
    )
    assert len(clusters) == 1
    assert clusters[0]["range"] == pytest.approx(10.08, abs=0.2)


def test_range_jump_splits_clusters():
    clusters = cluster_scan_hits(
        [10.0, 10.0, 10.0, 18.0, 18.0, 18.0],
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.5,
        range_max=25.0,
        cluster_gap_m=3.0,
        min_points=3,
    )
    assert len(clusters) == 2


def test_0141_payload_uses_wgs84_and_matches_bridge_shape():
    scan = _scan([10.0, 10.0, 10.0])
    text = scan_to_0141_json(scan, POSE, ORIGIN, min_points=3)
    payload = validate_obstacle_payload(text)
    assert payload["count"] == 1
    target = payload["targets"][0]
    east, north, _ = ORIGIN.to_local(target["latitude"], target["longitude"])
    assert east == pytest.approx(33.0, abs=0.3)
    assert north == pytest.approx(-10.0, abs=0.3)
    assert target["source"] == 1
    assert target["batch"] == 1
    assert target["absAzimuth"] == pytest.approx(90.0, abs=1.0)
    assert target["relAzimuth"] == pytest.approx(0.0, abs=1.0)


def test_empty_scan_is_a_valid_zero_target_list():
    payload = validate_obstacle_payload(
        scan_to_0141_json(_scan([]), POSE, ORIGIN)
    )
    assert payload == {"count": 0, "targets": []}


def test_max_targets_keeps_the_nearest_clusters():
    ranges = [12.0, 12.0, 12.0, math.inf, 6.0, 6.0, 6.0]
    targets = scan_to_0141_targets(
        _scan(ranges),
        POSE,
        ORIGIN,
        min_points=3,
        max_targets=1,
    )
    assert len(targets) == 1
    assert targets[0]["distance"] == pytest.approx(6.0, abs=0.2)
    assert targets[0]["batch"] == 1
