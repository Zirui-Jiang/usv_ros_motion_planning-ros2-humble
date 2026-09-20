"""Tests for the ROS-independent costmap snapshot serializer."""

from types import SimpleNamespace
import json

import pytest

from usv_ladds_bridge.costmap_0141 import (
    costmap_to_0141_dict,
    costmap_to_0141_json,
)
from usv_ladds_bridge.config import validate_obstacle_payload


def _costmap(data, *, size_x=3, size_y=2):
    stamp = SimpleNamespace(sec=12, nanosec=34)
    header = SimpleNamespace(stamp=stamp, frame_id="map")
    origin = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    metadata = SimpleNamespace(
        map_load_time=SimpleNamespace(sec=1, nanosec=2),
        update_time=SimpleNamespace(sec=3, nanosec=4),
        layer="master",
        resolution=0.5,
        size_x=size_x,
        size_y=size_y,
        origin=origin,
    )
    return SimpleNamespace(header=header, metadata=metadata, data=data)


def test_serializer_keeps_costmap_metadata_and_sparse_obstacles():
    payload = costmap_to_0141_dict(_costmap([0, 254, 255, 253, 10, 254]), sequence=7)

    assert payload["format"] == "costmap_snapshot_v1"
    assert payload["source"] == "costmap_raw"
    assert payload["count"] == 1
    target = payload["targets"][0]
    assert target["batch"] == 7
    assert target["kind"] == "costmap_snapshot"
    assert target["frame_id"] == "map"
    assert target["metadata"]["size_x"] == 3
    assert target["obstacle_cells"] == [
        {"x": 1, "y": 0, "cost": 254},
        {"x": 2, "y": 1, "cost": 254},
    ]
    assert target["obstacle_cell_count"] == 2
    assert target["truncated"] is False


def test_unknown_cells_can_be_enabled():
    payload = costmap_to_0141_dict(
        _costmap([255]),
        include_unknown=True,
    )
    assert payload["targets"][0]["obstacle_cells"] == [
        {"x": 0, "y": 0, "cost": 255}
    ]


def test_cell_limit_is_explicitly_reported():
    payload = costmap_to_0141_dict(
        _costmap([254, 254, 254]),
        max_cells=2,
    )
    target = payload["targets"][0]
    assert target["obstacle_cell_count"] == 3
    assert len(target["obstacle_cells"]) == 2
    assert target["truncated"] is True


def test_json_is_accepted_by_the_bridge_shape_validator():
    text = costmap_to_0141_json(_costmap([254]))
    decoded = validate_obstacle_payload(text)
    assert decoded["count"] == len(decoded["targets"]) == 1
    assert json.loads(text) == decoded


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"cost_threshold": -1}, "cost_threshold"),
        ({"cost_threshold": 256}, "cost_threshold"),
        ({"max_cells": 0}, "max_cells"),
    ],
)
def test_invalid_serializer_options_are_rejected(kwargs, message):
    with pytest.raises(ValueError, match=message):
        costmap_to_0141_dict(_costmap([254]), **kwargs)
