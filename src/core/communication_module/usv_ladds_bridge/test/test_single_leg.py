import json
from copy import deepcopy
from dataclasses import FrozenInstanceError

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.single_leg import make_completion, parse_single_leg


ORIGIN = GeoReference(30.5453, 114.392931)
CONTENT = {
    "areas": None,
    "areasCount": None,
    "count": 1,
    "guardingHeading": None,
    "guardingRadius": None,
    "points": [
        {
            "latitude": 30.546127,
            "longitude": 114.3938,
            "speed": 4,
            "type": 1,
        }
    ],
    "taskType": 16,
    "travelId": 53,
}


def test_parse_wrapped_single_leg_converts_wgs84_and_knots_to_enu_si():
    message = {
        "head": {"sendPlatform": 1000, "packageSeq": 3},
        "content": CONTENT,
    }

    leg = parse_single_leg(json.dumps(message), ORIGIN)

    assert leg.travel_id == 53
    assert leg.sender == 1000
    assert leg.identity == (1000, 53)
    assert leg.x == pytest.approx(83.38367674866956, abs=1e-9)
    assert leg.y == pytest.approx(91.682939134985, abs=1e-9)
    assert leg.speed_knots == 4.0
    assert leg.speed_mps == pytest.approx(2.057777777777778, abs=1e-15)
    assert leg.wrapped is True
    assert make_completion(leg) == {"travelId": 53, "taskFinish": 1}


def test_naked_content_has_no_sender_and_same_business_fingerprint():
    wrapped = parse_single_leg(
        json.dumps({"head": {"sendPlatform": 1000}, "content": CONTENT}), ORIGIN
    )
    naked = parse_single_leg(json.dumps(CONTENT, indent=2, sort_keys=True), ORIGIN)

    assert naked.sender is None
    assert naked.identity == (None, 53)
    assert naked.wrapped is False
    assert naked.fingerprint == wrapped.fingerprint
    assert len(naked.fingerprint) == 64


def test_explicit_zero_area_count_and_empty_area_list_are_supported():
    content = deepcopy(CONTENT)
    content["areasCount"] = 0
    content["areas"] = []

    leg = parse_single_leg(json.dumps(content), ORIGIN)

    assert leg.travel_id == 53


def _content_with(**changes):
    content = deepcopy(CONTENT)
    content.update(changes)
    return content


@pytest.mark.parametrize(
    "content",
    [
        _content_with(taskType=15),
        _content_with(taskType=16.0),
        _content_with(taskType=True),
        _content_with(count=2),
        _content_with(count=1.0),
        _content_with(count=True),
        _content_with(points=[]),
        _content_with(points=CONTENT["points"] * 2),
        _content_with(points="not-a-list"),
        _content_with(points=[{**CONTENT["points"][0], "type": 2}]),
        _content_with(points=[{**CONTENT["points"][0], "type": 1.0}]),
        _content_with(points=[{**CONTENT["points"][0], "type": True}]),
    ],
)
def test_rejects_unknown_task_shapes_instead_of_selecting_a_waypoint(content):
    with pytest.raises(ValueError):
        parse_single_leg(json.dumps(content), ORIGIN)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("latitude", 91),
        ("latitude", float("nan")),
        ("latitude", True),
        ("longitude", -181),
        ("longitude", float("inf")),
        ("longitude", False),
        ("speed", 0),
        ("speed", -1),
        ("speed", float("nan")),
        ("speed", True),
    ],
)
def test_rejects_invalid_coordinate_or_speed_numbers(field, value):
    point = {**CONTENT["points"][0], field: value}

    with pytest.raises(ValueError):
        parse_single_leg(json.dumps(_content_with(points=[point])), ORIGIN)


@pytest.mark.parametrize(
    "changes",
    [
        {"areasCount": 1},
        {"areasCount": True},
        {"areasCount": "0"},
        {"areas": [{"name": "unsupported"}]},
        {"areas": {}},
        {"guardingHeading": 0},
        {"guardingRadius": 0},
    ],
)
def test_rejects_area_and_guarding_constraints(changes):
    with pytest.raises(ValueError):
        parse_single_leg(json.dumps(_content_with(**changes)), ORIGIN)


@pytest.mark.parametrize("travel_id", [True, 53.0, "53", None])
def test_travel_id_must_be_an_integer(travel_id):
    with pytest.raises(ValueError):
        parse_single_leg(json.dumps(_content_with(travelId=travel_id)), ORIGIN)


@pytest.mark.parametrize("sender", [True, 1000.0, "1000"])
def test_wrapped_sender_must_be_an_integer_when_present(sender):
    message = {"head": {"sendPlatform": sender}, "content": CONTENT}

    with pytest.raises(ValueError):
        parse_single_leg(json.dumps(message), ORIGIN)


@pytest.mark.parametrize(
    "text",
    [
        "not json",
        "[]",
        json.dumps({"content": CONTENT}),
        json.dumps({"head": [], "content": CONTENT}),
        json.dumps({"head": {}, "content": []}),
    ],
)
def test_malformed_or_incomplete_wrappers_raise_value_error_without_state(text):
    with pytest.raises(ValueError):
        parse_single_leg(text, ORIGIN)

    # A prior failure cannot affect the next pure parse operation.
    assert parse_single_leg(json.dumps(CONTENT), ORIGIN).travel_id == 53


def test_single_leg_is_frozen_and_completion_is_bare_business_json():
    leg = parse_single_leg(json.dumps(CONTENT), ORIGIN)

    with pytest.raises(FrozenInstanceError):
        leg.x = 0.0

    assert json.dumps(make_completion(leg), separators=(",", ":")) == (
        '{"travelId":53,"taskFinish":1}'
    )
