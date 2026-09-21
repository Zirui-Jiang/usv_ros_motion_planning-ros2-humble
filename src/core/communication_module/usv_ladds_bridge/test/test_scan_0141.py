"""Behavioral tests for direct LaserScan to 0141 target conversion."""

import json
import math

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.config import validate_obstacle_payload
from usv_ladds_bridge.scan_0141 import (
    scan_to_0141_dict,
    scan_to_0141_json,
)


ORIGIN = GeoReference(30.5453, 114.392931)


def test_contiguous_valid_beams_become_one_target_with_position_and_bearing():
    payload = scan_to_0141_dict(
        ranges=[float("inf"), 10.0, 10.2, 10.1, float("inf"), 5.0],
        angle_min=-math.pi / 2.0,
        angle_increment=math.pi / 8.0,
        range_min=0.2,
        range_max=25.0,
        vehicle_x=100.0,
        vehicle_y=200.0,
        vehicle_yaw=0.0,
        origin=ORIGIN,
        sequence=7,
    )

    assert payload["format"] == "lidar_targets_v1"
    assert payload["source"] == "laser_scan"
    assert payload["count"] == len(payload["targets"]) == 2

    first, second = payload["targets"]
    assert first["batch"] == 7
    assert first["distance"] == pytest.approx(10.1, abs=0.1)
    assert first["relAzimuth"] == pytest.approx(-44.9238, abs=0.001)
    assert first["absAzimuth"] == pytest.approx(134.9238, abs=0.001)
    assert first["longitude"] > ORIGIN.longitude
    assert first["latitude"] > ORIGIN.latitude
    assert first["absSpeed"] == -1.0
    assert first["absCourse"] == -1.0
    assert first["heading"] == -1.0
    assert first["pointCount"] == 3
    assert second["distance"] == pytest.approx(5.0)


def test_invalid_and_out_of_range_beams_are_ignored_and_sequence_wraps():
    payload = scan_to_0141_dict(
        ranges=[0.1, 1.0, float("nan"), 30.0, 2.0],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.2,
        range_max=25.0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=0.0,
        origin=ORIGIN,
        sequence=65536,
    )

    assert payload["count"] == 2
    assert [target["distance"] for target in payload["targets"]] == [1.0, 2.0]
    assert all(target["batch"] == 1 for target in payload["targets"])


def test_vehicle_yaw_rotates_world_position_and_absolute_bearing():
    payload = scan_to_0141_dict(
        ranges=[10.0],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.2,
        range_max=25.0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=math.pi / 2.0,
        origin=ORIGIN,
    )

    target = payload["targets"][0]
    assert target["relAzimuth"] == pytest.approx(0.0)
    assert target["absAzimuth"] == pytest.approx(0.0)
    assert target["longitude"] == pytest.approx(ORIGIN.longitude)
    assert target["latitude"] > ORIGIN.latitude


def test_json_is_accepted_by_bridge_shape_validator():
    text = scan_to_0141_json(
        ranges=[2.0, 2.0],
        angle_min=-0.1,
        angle_increment=0.1,
        range_min=0.2,
        range_max=25.0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=0.0,
        origin=ORIGIN,
    )

    decoded = validate_obstacle_payload(text)
    assert json.loads(text) == decoded
    assert decoded["count"] == 1


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"ranges": []}, "ranges"),
        ({"angle_increment": 0.0}, "angle_increment"),
        ({"range_min": 2.0, "range_max": 1.0}, "range_max"),
        ({"vehicle_x": float("nan")}, "vehicle_x"),
    ],
)
def test_invalid_scan_inputs_are_rejected(kwargs, message):
    values = dict(
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.2,
        range_max=25.0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=0.0,
        origin=ORIGIN,
    )
    values.update(kwargs)

    with pytest.raises(ValueError, match=message):
        scan_to_0141_dict(**values)
