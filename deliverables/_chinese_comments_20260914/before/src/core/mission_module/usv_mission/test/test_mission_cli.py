from pathlib import Path

import pytest

from common.geodesy import GeoReference
from usv_mission.mission_cli import load_mission, main, make_parser, mission_request_values


def test_station_keep_minimal_request_uses_interface_defaults():
    values = mission_request_values(dict(mode="station_keep", latitude=-33.7, longitude=150.6))
    assert values["heading_deg"] == 0.0
    assert values["position_tolerance"] == 0.5
    assert values["speed"] == 0.5
    assert values["clockwise"] is True
    assert values["laps"] == 0


@pytest.mark.parametrize("override", [{"mode": "orbiting"}, {"unknown": 1}, {"latitude": float("nan")},
                                     {"longitude": 181}, {"speed": True}, {"position_tolerance": 0},
                                     {"duration": -1}, {"clockwise": "false"}, {"laps": 2.5},
                                     {"laps": -1}, {"laps": 4294967296}, {"speed": float("inf")},
                                     {"watch_radius": 0.5}])
def test_bad_requests_rejected_locally(override):
    request = dict(mode="standby", latitude=0, longitude=0)
    request.update(override)
    with pytest.raises(ValueError):
        mission_request_values(request)


def test_mission_yaml_duplicate_or_missing_keys_rejected(tmp_path):
    path = tmp_path / "mission.yaml"
    path.write_text("mode: standby\nlatitude: 1\nlatitude: 2\nlongitude: 0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Duplicate"):
        load_mission(path)
    with pytest.raises(ValueError, match="Missing"):
        mission_request_values({"mode": "standby"})


def test_all_installed_examples_validate_and_reference_the_selected_world():
    base = Path(__file__).resolve().parents[1]
    geo = GeoReference.from_world(base.parents[2] / "sim_env/worlds/ocean.world")
    modes = set()
    for path in (base / "config/mission/examples").glob("*.yaml"):
        request = load_mission(path)
        modes.add(request["mode"])
        assert geo.to_local(request["latitude"], request["longitude"])[:2] == pytest.approx((23, -10), abs=1e-6)
    assert modes == {"standby", "orbit", "station_keep"}


def test_cli_commands_and_namespace_are_unambiguous():
    parser = make_parser()
    start = parser.parse_args(["--namespace", "boat", "start", "mission.yaml"])
    assert start.namespace == "boat" and start.yaml == Path("mission.yaml")
    assert parser.parse_args(["status"]).command == "status"
    assert parser.parse_args(["cancel"]).command == "cancel"


def test_invalid_yaml_reports_cli_error_before_ros_import(tmp_path, capsys):
    path = tmp_path / "invalid.yaml"
    path.write_text("mode: [standby\n", encoding="utf-8")
    with pytest.raises(SystemExit) as error:
        main(["start", str(path)])
    assert error.value.code == 2
    assert "error:" in capsys.readouterr().err
