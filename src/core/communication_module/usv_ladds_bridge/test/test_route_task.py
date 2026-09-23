import json
from copy import deepcopy
from dataclasses import FrozenInstanceError
import math

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.route_task import parse_route
from usv_ladds_bridge.single_leg import parse_single_leg


ORIGIN = GeoReference(30.5453, 114.392931)
POINTS = [
    {"type": 1, "latitude": 30.546127, "longitude": 114.3938, "speed": 2},
    {"type": 1, "latitude": 30.546227, "longitude": 114.3938, "speed": 4},
    {"type": 1, "latitude": 30.546327, "longitude": 114.3938, "speed": 2},
]


def _content(points=POINTS, count=None):
    return {
        "travelId": 58,
        "taskType": 16,
        "count": len(points) if count is None else count,
        "points": deepcopy(points),
        "areasCount": 0,
        "areas": [],
    }


def test_parse_route_preserves_three_point_position_and_segment_speed_order():
    message = {
        "head": {"sendPlatform": 1000, "packageSeq": 3},
        "content": _content(),
    }

    route = parse_route(json.dumps(message), ORIGIN)

    assert route.travel_id == 58
    assert route.task_finish == 1
    assert route.sender == 1000
    assert route.identity == (1000, 58)
    assert route.wrapped is True
    assert len(route.points) == 3
    assert [(point.x, point.y) for point in route.points] == pytest.approx([
        (83.38367674866956, 91.682939134985),
        (83.38359129340336, 102.76910894104553),
        (83.38350583759512, 113.85527891640633),
    ], abs=1e-9)
    assert [point.speed_knots for point in route.points] == [2.0, 4.0, 2.0]
    assert [point.speed_mps for point in route.points] == pytest.approx([
        1.028888888888889, 2.057777777777778, 1.028888888888889,
    ], abs=1e-15)
    assert len(route.fingerprint) == 64


@pytest.mark.parametrize("count", [1, 2, 4, 0, 256])
def test_parse_route_rejects_count_that_does_not_match_three_points(count):
    with pytest.raises(ValueError, match="count"):
        parse_route(json.dumps(_content(count=count)), ORIGIN)


@pytest.mark.parametrize(
    "invalid_point",
    [
        {"type": 2, "latitude": 30.546227, "longitude": 114.3938, "speed": 4},
        {"type": 1, "latitude": 91, "longitude": 114.3938, "speed": 4},
        {"type": 1, "latitude": 30.546227, "longitude": 181, "speed": 4},
        {"type": 1, "latitude": 30.546227, "longitude": 114.3938, "speed": 0},
        {"type": 1, "latitude": 30.546227, "longitude": 114.3938, "speed": True},
    ],
)
def test_parse_route_rejects_whole_packet_when_a_later_point_is_invalid(invalid_point):
    points = deepcopy(POINTS)
    points[1] = invalid_point

    with pytest.raises(ValueError, match=r"points\[1\]"):
        parse_route(json.dumps(_content(points=points)), ORIGIN)


@pytest.mark.parametrize("count", [1, 255])
def test_parse_route_accepts_documented_point_count_boundaries(count):
    points = [deepcopy(POINTS[0]) for _ in range(count)]

    route = parse_route(json.dumps(_content(points=points)), ORIGIN)

    assert len(route.points) == count


@pytest.mark.parametrize("count", [0, 256])
def test_parse_route_rejects_point_count_outside_protocol_range(count):
    points = [deepcopy(POINTS[0]) for _ in range(count)]

    with pytest.raises(ValueError, match=r"\[1, 255\]"):
        parse_route(json.dumps(_content(points=points)), ORIGIN)


def test_route_and_points_are_immutable_and_first_point_properties_support_leg_book():
    route = parse_route(json.dumps(_content()), ORIGIN)

    assert isinstance(route.points, tuple)
    assert route.x == route.points[0].x
    assert route.y == route.points[0].y
    assert route.speed_mps == route.points[0].speed_mps
    with pytest.raises(FrozenInstanceError):
        route.travel_id = 59
    with pytest.raises(FrozenInstanceError):
        route.points[0].x = 0.0


def test_single_point_route_matches_public_single_leg_contract():
    content = _content(points=[POINTS[0]])

    route = parse_route(json.dumps(content), ORIGIN)
    leg = parse_single_leg(json.dumps(content), ORIGIN)

    assert route.travel_id == leg.travel_id
    assert route.sender == leg.sender
    assert route.x == leg.x
    assert route.y == leg.y
    assert route.speed_knots == leg.speed_knots
    assert route.speed_mps == leg.speed_mps
    assert route.fingerprint == leg.fingerprint


def test_existing_single_leg_parser_still_rejects_multi_point_route():
    with pytest.raises(ValueError, match="count must be exactly 1"):
        parse_single_leg(json.dumps(_content()), ORIGIN)


@pytest.mark.parametrize('heading,yaw', [(None, None), (0, math.pi / 2), (90, 0), (360, math.pi / 2)])
def test_guard_route_preserves_optional_true_heading(heading, yaw):
    content = dict(_content(), taskType=80, guardingHeading=heading)
    route = parse_route(json.dumps(content), ORIGIN)
    assert route.guard_on_arrival is True
    assert route.task_type == 80
    assert route.task_finish == 2
    if yaw is None:
        assert route.guard_heading is None
    else:
        assert route.guard_heading == pytest.approx(yaw)
    assert len(route.points) == 3


def test_guard_fingerprint_distinguishes_null_zero_and_route_task_type():
    variants = [_content(), dict(_content(), taskType=80),
                dict(_content(), taskType=80, guardingHeading=0)]
    assert len({parse_route(json.dumps(v), ORIGIN).fingerprint for v in variants}) == 3


@pytest.mark.parametrize('heading', [True, '90', -1, 361, float('nan'), float('inf')])
def test_guard_route_rejects_invalid_heading(heading):
    with pytest.raises(ValueError, match='guardingHeading'):
        parse_route(json.dumps(dict(_content(), taskType=80, guardingHeading=heading)), ORIGIN)


def test_guard_regions_and_radius_are_explicitly_unapplied():
    route = parse_route(json.dumps(dict(_content(), taskType=80,
        areas=[{'funcId': 5, 'polygon': []}], areasCount=None, guardingRadius=10)), ORIGIN)
    assert any('未应用区域约束' in warning for warning in route.warnings)
    assert any('guardingRadius' in warning for warning in route.warnings)
    assert route.guard_heading is None


def test_normal_route_still_rejects_unsupported_constraints():
    with pytest.raises(ValueError):
        parse_route(json.dumps(dict(_content(), areas=[{'funcId': 5}])), ORIGIN)


@pytest.mark.parametrize('count', [1, 255])
def test_guard_route_accepts_single_and_maximum_waypoint_count(count):
    route = parse_route(json.dumps(dict(_content(points=[POINTS[0]] * count), taskType=80)), ORIGIN)
    assert len(route.points) == count
    assert route.guard_on_arrival
