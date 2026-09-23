import json
from copy import deepcopy

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.berth_task import parse_berth
from usv_ladds_bridge.single_leg import make_completion


ORIGIN = GeoReference(30.5453, 114.392931)
POINT = {"type": 1, "latitude": 30.546127, "longitude": 114.3938, "speed": 4}


def _content(**changes):
    content = {
        "travelId": 53,
        "taskType": 3,
        "count": 1,
        "points": [deepcopy(POINT)],
        "areasCount": 0,
        "areas": [],
        "guardingHeading": None,
        "guardingRadius": None,
    }
    content.update(changes)
    return content


def test_parse_berth_converts_one_point_and_sets_0212_completion_code():
    message = {
        "head": {"sendPlatform": 1000, "packageSeq": 3},
        "content": _content(),
    }

    route = parse_berth(json.dumps(message), ORIGIN)

    assert route.protocol == "0212"
    assert route.task_type == 3
    assert route.task_finish == 3
    assert route.travel_id == 53
    assert route.sender == 1000
    assert route.wrapped is True
    assert len(route.points) == 1
    assert route.x == pytest.approx(83.38367674866956, abs=1e-9)
    assert route.y == pytest.approx(91.682939134985, abs=1e-9)
    assert route.speed_knots == 4.0
    assert route.speed_mps == pytest.approx(2.057777777777778, abs=1e-15)
    assert route.guard_on_arrival is False
    assert make_completion(route) == {"travelId": 53, "taskType": 3, "taskFinish": 3}


def test_parse_berth_uses_out_berth_lat_lon_as_one_waypoint():
    content = {
        "areas": None,
        "areasCount": None,
        "berthCenterLat": None,
        "berthCenterLon": None,
        "berthDir": None,
        "berthHeading": None,
        "berthLength": None,
        "berthSpeed": 1,
        "berthType": None,
        "berthWidth": None,
        "outBerthLat": 30.5472,
        "outBerthLon": 114.3891,
        "taskType": 1,
        "travelId": 8,
    }
    route = parse_berth(json.dumps({"content": content, "head": {"sendPlatform": 1000}}), ORIGIN)
    expected_x, expected_y, _ = ORIGIN.to_local(30.5472, 114.3891)

    assert len(route.points) == 1
    assert route.travel_id == 8
    assert route.task_type == 1
    assert route.task_finish == 3
    assert route.speed_knots == 1.0
    assert route.x == pytest.approx(expected_x)
    assert route.y == pytest.approx(expected_y)
    assert route.warnings == ()
    assert make_completion(route) == {"travelId": 8, "taskType": 1, "taskFinish": 3}


def test_parse_berth_rejects_multi_point_packet_without_using_first_point():
    points = [deepcopy(POINT), deepcopy(POINT)]
    with pytest.raises(ValueError, match="exactly 1"):
        parse_berth(json.dumps(_content(count=2, points=points)), ORIGIN)
    with pytest.raises(ValueError, match="exactly one point"):
        parse_berth(json.dumps(_content(count=1, points=points)), ORIGIN)


@pytest.mark.parametrize(
    "invalid_point",
    [
        {"type": 2, "latitude": 30.546127, "longitude": 114.3938, "speed": 4},
        {"type": 1, "latitude": 91, "longitude": 114.3938, "speed": 4},
        {"type": 1, "latitude": 30.546127, "longitude": 181, "speed": 4},
        {"type": 1, "latitude": 30.546127, "longitude": 114.3938, "speed": 0},
    ],
)
def test_parse_berth_rejects_invalid_single_point(invalid_point):
    with pytest.raises(ValueError, match=r"points\[0\]"):
        parse_berth(json.dumps(_content(points=[invalid_point])), ORIGIN)


def test_parse_berth_ignores_unused_berth_fields_with_warning():
    route = parse_berth(json.dumps(_content(
        heading=90, berthType=1, dockId=7, areas=[{"funcId": 5}],
    )), ORIGIN)

    assert route.protocol == "0212"
    assert route.task_finish == 3
    assert any("heading" in warning for warning in route.warnings)
    assert any("berthType" in warning for warning in route.warnings)
    assert any("dockId" in warning for warning in route.warnings)
    assert any("areas" in warning for warning in route.warnings)


@pytest.mark.parametrize("task_type", [0, 1, 16, 80, True, 3.0])
def test_parse_berth_rejects_task_type_other_than_departure_or_berthing(task_type):
    with pytest.raises(ValueError, match="taskType"):
        parse_berth(json.dumps(_content(taskType=task_type)), ORIGIN)


def test_parse_berth_berthing_completion_uses_task_type_4():
    route = parse_berth(json.dumps(_content(taskType=4)), ORIGIN)

    assert route.task_type == 4
    assert route.guard_on_arrival is False
    assert make_completion(route) == {"travelId": 53, "taskType": 4, "taskFinish": 3}
