"""ROS-free tests for the VRX station-keeping recorder.

These tests intentionally exercise the recorder's pure-Python core.  ROS 2 is
not required on the Windows development machine; the runtime adapter is tested
only indirectly by checking that importing the module does not import rclpy.
"""

import importlib.util
import json
import math
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("record_station_keeping.py")
SPEC = importlib.util.spec_from_file_location("record_station_keeping", MODULE_PATH)
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


def test_official_pose_error_uses_shortest_heading_arc():
    """The VRX score is distance + 0.75**distance * wrapped yaw error."""

    # pi + 0.2 and -pi + 0.2 are almost the same direction; a raw subtraction
    # would incorrectly report an error close to 2*pi.
    value = recorder.pose_error(
        target=(0.0, 0.0, math.pi - 0.1),
        pose=(1.0, 0.0, -math.pi + 0.1),
    )
    expected_heading = 0.2
    assert value == pytest.approx(1.0 + 0.75**1.0 * expected_heading)


def test_default_phase_boundaries_match_vrx_practice_task():
    assert recorder.phase_for_time(0.0, 20.0, 300.0) == "initial"
    assert recorder.phase_for_time(9.999, 20.0, 300.0) == "initial"
    assert recorder.phase_for_time(10.0, 20.0, 300.0) == "ready"
    assert recorder.phase_for_time(19.999, 20.0, 300.0) == "ready"
    assert recorder.phase_for_time(20.0, 20.0, 300.0) == "running"
    assert recorder.phase_for_time(319.999, 20.0, 300.0) == "running"
    assert recorder.phase_for_time(320.0, 20.0, 300.0) == "finished"


def test_pose_error_rejects_nonfinite_inputs():
    with pytest.raises(ValueError):
        recorder.pose_error((0.0, 0.0, 0.0), (math.nan, 0.0, 0.0))


def test_recorder_filters_score_to_running_window_and_writes_jsonl(tmp_path):
    output = tmp_path / "trial"
    trial = recorder.StationKeepingRecorder(
        output=output,
        target=(0.0, 0.0, 0.0),
        start_delay=2.0,
        duration=3.0,
        metadata={"world": "stationkeeping_vrx"},
    )
    # Initial/ready samples are retained as events but do not enter the score.
    trial.record_sample(0.5, 1.0, 0.0, 0.0, wind_speed=2.0, thrusts={"left": 0.1})
    trial.record_sample(2.0, 1.0, 0.0, 0.0, wind_speed=3.0, thrusts={"left": 0.2})
    trial.record_sample(4.9, 2.0, 0.0, math.pi / 2, wind_speed=4.0, thrusts={"left": 0.3})
    # The end boundary is exclusive, matching [start_delay, start+duration).
    trial.record_sample(5.0, 100.0, 0.0, 0.0, wind_speed=99.0, thrusts={"left": 9.0})
    metrics = trial.finalize()

    assert metrics["samples"]["running"] == 2
    expected = (1.0 + (2.0 + 0.75**2 * math.pi / 2.0)) / 2.0
    assert metrics["score"]["pose_error_mean"] == pytest.approx(expected)
    events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    assert [event["phase"] for event in events] == ["initial", "running", "running", "finished"]
    assert (output / "metadata.json").is_file()
    assert (output / "metrics.json").is_file()
    assert (output / "summary.json").is_file()


def test_recorder_collects_finite_wind_and_thrust_distributions(tmp_path):
    trial = recorder.StationKeepingRecorder(
        output=tmp_path / "trial",
        target=(0.0, 0.0, 0.0),
        start_delay=0.0,
        duration=10.0,
        thrust_limit=1.0,
    )
    trial.record_sample(
        1.0,
        0.0,
        0.0,
        0.0,
        wind_speed=5.0,
        wind_direction=120.0,
        thrusts={"left": 0.5, "right": -0.5, "lateral": math.nan},
    )
    trial.record_sample(
        2.0,
        0.0,
        0.0,
        0.0,
        wind_speed=7.0,
        wind_direction=130.0,
        thrusts={"left": 2.0, "right": 0.0, "lateral": 0.2},
    )
    metrics = trial.finalize()
    assert metrics["wind"]["speed"]["mean"] == pytest.approx(6.0)
    assert metrics["wind"]["direction"]["p95"] == pytest.approx(130.0)
    assert metrics["thrust"]["axes"]["left"]["max"] == pytest.approx(2.0)
    assert metrics["thrust"]["invalid_command_count"] == 2


def test_thrust_validity_counts_only_the_scored_running_window(tmp_path):
    trial = recorder.StationKeepingRecorder(
        output=tmp_path / "trial",
        target=(0.0, 0.0, 0.0),
        start_delay=2.0,
        duration=3.0,
    )
    # Missing setup commands are retained in the event stream but must not
    # affect the running-window command-quality metric.
    trial.record_sample(1.0, 0.0, 0.0, 0.0, thrusts={})
    trial.record_sample(2.0, 0.0, 0.0, 0.0, thrusts={"left": 2.0})
    metrics = trial.finalize()
    assert metrics["thrust"]["missing_command_count"] == 2
    assert metrics["thrust"]["invalid_command_count"] == 1


def test_model_state_velocity_is_written_as_world_velocity(tmp_path):
    trial = recorder.StationKeepingRecorder(
        output=tmp_path / "trial",
        target=(0.0, 0.0, 0.0),
        start_delay=0.0,
        duration=1.0,
    )
    trial.record_sample(0.1, 0.0, 0.0, 0.0, body_velocity=(1.0, 2.0, 0.3))
    trial.finalize()
    event = json.loads((trial.output / "events.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert event["world_velocity"] == [1.0, 2.0, 0.3]
    assert "body_velocity" not in event


def test_output_directory_is_never_overwritten(tmp_path):
    output = tmp_path / "already_there"
    output.mkdir()
    with pytest.raises(FileExistsError):
        recorder.StationKeepingRecorder(output=output, target=(0.0, 0.0, 0.0))


def test_invalid_window_and_target_are_rejected(tmp_path):
    with pytest.raises(ValueError):
        recorder.StationKeepingRecorder(output=tmp_path / "a", target=(0.0, 0.0, 0.0), duration=0)
    with pytest.raises(ValueError):
        recorder.StationKeepingRecorder(output=tmp_path / "b", target=(0.0, math.inf, 0.0))


def test_default_topics_are_read_only_and_cover_three_otter_thrusters():
    assert recorder.DEFAULT_TOPICS["model_states"] == "/gazebo/model_states"
    assert recorder.DEFAULT_TOPICS["wind_speed"] == "/vrx/debug/wind/speed"
    assert recorder.DEFAULT_TOPICS["wind_direction"] == "/vrx/debug/wind/direction"
    assert set(recorder.DEFAULT_TOPICS["thrusts"]) == {"left", "right", "lateral"}
    assert all("publish" not in name for name in dir(recorder))


def test_clock_subscription_uses_humble_clock_compatible_qos():
    class Profile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    class Reliability:
        BEST_EFFORT = "best_effort"

    class Durability:
        VOLATILE = "volatile"

    profile = recorder.make_clock_qos(Profile, Reliability, Durability)
    assert profile.settings == {
        "depth": 1,
        "reliability": "best_effort",
        "durability": "volatile",
    }


def test_live_clock_subscription_uses_the_compatible_profile_helper():
    """Gazebo's best-effort /clock must not be requested as reliable."""

    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "clock_qos = make_clock_qos" in source
    assert 'create_subscription(Clock, topics["clock"], self._on_clock, clock_qos)' in source


def test_wall_time_mode_uses_relative_monotonic_elapsed_time():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "if args.wall_time:" in source
    assert "time.monotonic() - self._wall_start" in source
    assert "if self.latest_clock is not None:" in source


def test_metadata_captures_official_source_and_classic_wave_mapping(tmp_path):
    trial = recorder.StationKeepingRecorder(
        output=tmp_path / "trial",
        target=(0.0, 0.0, 0.0),
    )
    metadata = json.loads((trial.output / "metadata.json").read_text(encoding="utf-8"))
    scenario = metadata["scenario"]
    assert scenario["source"]["repository"] == "https://github.com/osrf/vrx.git"
    assert scenario["source"]["commit"] == "dc30ed8d17aa1083fd872edad9c77c69896d2b07"
    assert scenario["wind"] == {
        "direction_deg": 120.0,
        "mean_velocity_mps": 5.0,
        "random_gain": 2.0,
        "time_constant_s": 2.0,
        "random_seed": 15,
        "update_rate_hz": 10.0,
    }
    assert scenario["official_wave"] == {
        "direction_rad": pytest.approx(math.pi / 4),
        "gain": 0.6,
        "period_s": 6.0,
    }
    assert scenario["classic_wave_mapping"]["direction_xy"] == pytest.approx(
        [math.sqrt(0.5), math.sqrt(0.5)]
    )
    assert scenario["classic_wave_mapping"]["model"] == "PMS"
    trial.finalize()
