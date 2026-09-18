#!/usr/bin/env python3
"""Record and score a VRX-style Station Keeping run.

The recorder is deliberately a *read-only* ROS 2 observer.  It subscribes to
Gazebo model state, wind diagnostics and the three Otter thrust command topics;
it never publishes a goal, a command, or a task/service request.  The pure
Python portion of this module is usable on a machine without ROS 2, which
makes it suitable for offline tests and for inspecting a captured JSONL file.

The running score follows the VRX Station Keeping scoring plugin::

    pose_error = distance + 0.75 ** distance * abs(wrap_to_pi(yaw_error))

The score is an arithmetic mean over samples in the configured running window
``[start_delay, start_delay + duration)``.  Samples before and after that
window are retained in ``events.jsonl`` with a phase label, but do not affect
the running score.

Typical ROS 2 usage (after sourcing the workspace) is::

    python3 scripts/validation/record_station_keeping.py \
        --output results/station_keep_01 --x 0 --y 0 --yaw 0 \
        --start-delay 20 --duration 300

The output directory must not already exist.  This prevents an interrupted
experiment from silently being mixed with a later run.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Optional, Sequence


TAU = 2.0 * math.pi
DEFAULT_MODEL_NAME = "otter"
DEFAULT_TOPICS = {
    "model_states": "/gazebo/model_states",
    "wind_speed": "/vrx/debug/wind/speed",
    "wind_direction": "/vrx/debug/wind/direction",
    "thrusts": {
        "left": "/otter/thrusters/left_thrust_cmd",
        "right": "/otter/thrusters/right_thrust_cmd",
        "lateral": "/otter/thrusters/lateral_thrust_cmd",
    },
    "clock": "/clock",
}
DEFAULT_SCENARIO_METADATA = {
    "name": "stationkeeping_vrx",
    "source": {
        "repository": "https://github.com/osrf/vrx.git",
        "branch": "humble",
        "commit": "dc30ed8d17aa1083fd872edad9c77c69896d2b07",
        "world": "vrx_gz/worlds/2023_practice/practice_2023_stationkeeping1_task.sdf",
    },
    "wind": {
        "direction_deg": 120.0,
        "mean_velocity_mps": 5.0,
        "random_gain": 2.0,
        "time_constant_s": 2.0,
        "random_seed": 15,
        "update_rate_hz": 10.0,
    },
    # These are the three fields explicitly published by the official
    # practice world on /vrx/wavefield/parameters.
    "official_wave": {
        "direction_rad": 0.7853982,
        "gain": 0.6,
        "period_s": 6.0,
    },
    # Gazebo Classic accepts a direction vector in SDF instead of the Gazebo
    # Sim scalar direction message.  The remaining values retain this
    # project's existing three-component PMS setup.
    "classic_wave_mapping": {
        "model": "PMS",
        "period_s": 6.0,
        "gain": 0.6,
        "direction_xy": [0.70710678, 0.70710678],
        "number": 3,
        "scale": 2.5,
        "angle_rad": 0.4,
        "tau_s": 2.0,
        "steepness": 0.0,
    },
}


def _finite(value: Any) -> Optional[float]:
    """Return a finite float, or ``None`` for missing/non-finite input."""

    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if math.isfinite(converted) else None


def _require_finite(value: Any, name: str) -> float:
    converted = _finite(value)
    if converted is None:
        raise ValueError(f"{name} must be a finite number")
    return converted


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)`` without making a ROS dependency."""

    value = _require_finite(angle, "angle")
    return (value + math.pi) % TAU - math.pi


def validate_pose(pose: Sequence[float], name: str = "pose") -> tuple[float, float, float]:
    """Validate and normalize an ``(x, y, yaw)`` sequence."""

    if isinstance(pose, Mapping):
        try:
            values = (pose["x"], pose["y"], pose["yaw"])
        except KeyError:
            try:
                values = (pose["x_m"], pose["y_m"], pose["yaw_rad"])
            except KeyError as error:
                raise ValueError(f"{name} mapping requires x/y/yaw fields") from error
    elif isinstance(pose, (str, bytes)):
        raise ValueError(f"{name} must contain exactly three numbers: x, y, yaw")
    else:
        try:
            values = tuple(pose)
        except TypeError as error:
            raise ValueError(f"{name} must contain exactly three numbers: x, y, yaw") from error
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly three numbers: x, y, yaw")
    return tuple(_require_finite(item, f"{name}[{index}]") for index, item in enumerate(values))


def pose_error(target: Sequence[float], pose: Sequence[float]) -> float:
    """Compute the official VRX Station Keeping pose error.

    ``target`` and ``pose`` are ``(x, y, yaw)`` tuples in metres/radians.  The
    heading component is the shortest signed angular distance and is made
    positive for the score.
    """

    tx, ty, tyaw = validate_pose(target, "target")
    x, y, yaw = validate_pose(pose, "pose")
    distance = math.hypot(tx - x, ty - y)
    heading_error = abs(wrap_angle(yaw - tyaw))
    return distance + (0.75**distance) * heading_error


def pose_components(target: Sequence[float], pose: Sequence[float]) -> dict[str, float]:
    """Return distance, wrapped heading error and official score components."""

    target_pose = validate_pose(target, "target")
    current_pose = validate_pose(pose, "pose")
    distance = math.hypot(target_pose[0] - current_pose[0], target_pose[1] - current_pose[1])
    heading = abs(wrap_angle(current_pose[2] - target_pose[2]))
    return {
        "distance_m": distance,
        "heading_error_rad": heading,
        "pose_error": distance + (0.75**distance) * heading,
    }


def distribution(values: Sequence[Any]) -> dict[str, Optional[float] | int]:
    """Summarize finite values without turning missing data into zero.

    The percentile convention is the same conservative nearest-rank choice as
    the existing validation metrics helper: ``ceil(0.95*n) - 1``.
    """

    finite_values = sorted(value for value in (_finite(item) for item in values) if value is not None)
    if not finite_values:
        return {"count": 0, "mean": None, "rms": None, "p95": None, "max": None, "min": None}
    count = len(finite_values)
    p95_index = max(0, math.ceil(0.95 * count) - 1)
    return {
        "count": count,
        "mean": sum(finite_values) / count,
        "rms": math.sqrt(sum(item * item for item in finite_values) / count),
        "p95": finite_values[p95_index],
        "max": finite_values[-1],
        "min": finite_values[0],
    }


def _json_safe(value: Any) -> Any:
    """Convert values to strict-JSON-compatible primitives.

    In particular, NaN and infinity become JSON ``null`` rather than relying on
    Python's non-standard ``allow_nan`` extension.
    """

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def phase_for_time(sim_time_s: float, start_delay: float, duration: float) -> str:
    """Map simulation time to the recorder's phase label.

    For the official 20-second delay this yields ``initial`` (0--10 s),
    ``ready`` (10--20 s), ``running`` (20--320 s), and ``finished`` afterwards.
    If a shorter custom delay is selected, the pre-running interval is simply
    labelled ``initial``; this keeps the helper useful for quick smoke tests.
    """

    timestamp = _require_finite(sim_time_s, "sim_time_s")
    delay = _require_finite(start_delay, "start_delay")
    span = _require_finite(duration, "duration")
    if delay < 0 or span <= 0:
        raise ValueError("start_delay must be non-negative and duration must be positive")
    if timestamp < 0:
        return "initial"
    if timestamp < delay:
        return "ready" if delay >= 10.0 and timestamp >= 10.0 else "initial"
    if timestamp < delay + span:
        return "running"
    return "finished"


class _JsonlWriter:
    """Small flushing JSONL writer used by the recorder."""

    def __init__(self, path: Path):
        self.path = path
        self._file = path.open("x", encoding="utf-8", newline="\n")

    def write(self, payload: Mapping[str, Any]) -> None:
        self._file.write(json.dumps(_json_safe(dict(payload)), ensure_ascii=False, allow_nan=False) + "\n")
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()


class StationKeepingRecorder:
    """Collect read-only samples and write a VRX-style station-keeping report.

    Parameters
    ----------
    output:
        New directory for ``events.jsonl``, ``metadata.json`` and
        ``metrics.json``.  Existing paths are rejected, even when empty.
    target:
        Target ``(x, y, yaw)`` in world metres/radians.
    start_delay, duration:
        Running score window in simulation seconds.
    thrust_limit:
        Absolute normalized command limit.  Commands outside this range or
        non-finite commands count as invalid observations.  ``None`` disables
        the range check while still counting non-finite values.
    """

    def __init__(
        self,
        output: str | Path,
        target: Sequence[float],
        start_delay: float = 20.0,
        duration: float = 300.0,
        *,
        model_name: str = DEFAULT_MODEL_NAME,
        topics: Optional[Mapping[str, Any]] = None,
        thrust_limit: Optional[float] = 1.0,
        metadata: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.output = Path(output)
        if self.output.exists():
            raise FileExistsError(f"refusing to overwrite existing output directory: {self.output}")
        self.target = validate_pose(target, "target")
        self.start_delay = _require_finite(start_delay, "start_delay")
        self.duration = _require_finite(duration, "duration")
        if self.start_delay < 0 or self.duration <= 0:
            raise ValueError("start_delay must be non-negative and duration must be positive")
        self.model_name = str(model_name).strip()
        if not self.model_name:
            raise ValueError("model_name must not be empty")
        if thrust_limit is not None:
            self.thrust_limit = _require_finite(thrust_limit, "thrust_limit")
            if self.thrust_limit <= 0:
                raise ValueError("thrust_limit must be positive or None")
        else:
            self.thrust_limit = None

        self.topics = _merge_topics(topics)
        self.output.mkdir(parents=True, exist_ok=False)
        self._events = _JsonlWriter(self.output / "events.jsonl")
        self._closed = False
        self._metrics: Optional[dict[str, Any]] = None
        self._samples: list[dict[str, Any]] = []
        self._phase_counts: Counter[str] = Counter()
        self._last_time: Optional[float] = None
        self._max_gap: Optional[float] = None
        self._time_regressions = 0
        # Thrust validity is reported for the scored running window only, so
        # setup/ready samples cannot dilute or inflate the experiment metric.
        self._invalid_thrust_count = 0
        self._missing_thrust_count = 0

        metadata_payload = {
            "schema": "vrx_station_keeping_recorder/v1",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "model_name": self.model_name,
            "target": {"x_m": self.target[0], "y_m": self.target[1], "yaw_rad": self.target[2]},
            "window": {
                "start_delay_s": self.start_delay,
                "duration_s": self.duration,
                "end_time_s": self.start_delay + self.duration,
                "interval": "[start_delay, start_delay + duration)",
            },
            "thrust_limit": self.thrust_limit,
            "topics": self.topics,
            "scenario": DEFAULT_SCENARIO_METADATA,
        }
        if metadata:
            metadata_payload["user_metadata"] = dict(metadata)
        self.metadata = _json_safe(metadata_payload)
        (self.output / "metadata.json").write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )

    @property
    def end_time(self) -> float:
        return self.start_delay + self.duration

    def _thrust_values(
        self,
        thrusts: Optional[Mapping[str, Any]],
        *,
        count_invalid: bool,
    ) -> dict[str, Optional[float]]:
        values: dict[str, Optional[float]] = {}
        source = thrusts or {}
        for axis in ("left", "right", "lateral"):
            raw = source.get(axis)
            value = _finite(raw)
            values[axis] = value
            if count_invalid and raw is None:
                self._missing_thrust_count += 1
            elif count_invalid and (value is None or
                                    (self.thrust_limit is not None and abs(value) > self.thrust_limit)):
                self._invalid_thrust_count += 1
        return values

    def record_sample(
        self,
        sim_time_s: float,
        x: float,
        y: float,
        yaw: float,
        *,
        wind_speed: Optional[float] = None,
        wind_direction: Optional[float] = None,
        thrusts: Optional[Mapping[str, Any]] = None,
        body_velocity: Optional[Sequence[float]] = None,
        world_velocity: Optional[Sequence[float]] = None,
    ) -> bool:
        """Record one model-state sample and return whether it is scored.

        A sample is always written to ``events.jsonl``.  The returned boolean
        is true only inside the running score window.
        """

        if self._closed:
            raise RuntimeError("cannot record a sample after finalize()")
        timestamp = _require_finite(sim_time_s, "sim_time_s")
        pose = validate_pose((x, y, yaw), "pose")
        if timestamp < 0:
            raise ValueError("sim_time_s must be non-negative")

        phase = phase_for_time(timestamp, self.start_delay, self.duration)
        running = phase == "running"
        components = pose_components(self.target, pose)
        thrust_values = self._thrust_values(thrusts, count_invalid=running)
        speed = _finite(wind_speed)
        direction = _finite(wind_direction)
        # ``ModelStates.twist`` is expressed in the world frame.  Keep the
        # legacy body_velocity argument as an input alias, but write an
        # unambiguous world_velocity field to new event files.
        if world_velocity is None:
            world_velocity = body_velocity
        velocity = None
        if world_velocity is not None:
            try:
                velocity = [_finite(item) for item in world_velocity]
            except TypeError:
                velocity = None

        if self._last_time is not None:
            delta = timestamp - self._last_time
            if delta < 0:
                self._time_regressions += 1
            else:
                self._max_gap = delta if self._max_gap is None else max(self._max_gap, delta)
        self._last_time = timestamp

        sample = {
            "event": "sample",
            "wall_time_monotonic_s": time.monotonic(),
            "sim_time_s": timestamp,
            "phase": phase,
            "running": running,
            "pose": {"x_m": pose[0], "y_m": pose[1], "yaw_rad": pose[2]},
            "target": {"x_m": self.target[0], "y_m": self.target[1], "yaw_rad": self.target[2]},
            "errors": components,
            "distance_m": components["distance_m"],
            "heading_error_rad": components["heading_error_rad"],
            "pose_error": components["pose_error"],
            "wind": {"speed_mps": speed, "direction_deg": direction},
            "thrust": thrust_values,
        }
        if velocity is not None:
            sample["world_velocity"] = velocity
        self._events.write(sample)
        self._phase_counts[phase] += 1
        self._samples.append({
            "sim_time_s": timestamp,
            "phase": phase,
            "running": running,
            "distance_m": components["distance_m"],
            "heading_error_rad": components["heading_error_rad"],
            "pose_error": components["pose_error"],
            "wind_speed": speed,
            "wind_direction": direction,
            "thrust": thrust_values,
        })
        return running

    def record_pose(
        self,
        sim_time_s: float,
        pose: Sequence[float] | Mapping[str, Any],
        **kwargs: Any,
    ) -> bool:
        """Convenience wrapper accepting a single ``(x, y, yaw)`` pose."""

        x, y, yaw = validate_pose(pose, "pose")
        return self.record_sample(sim_time_s, x, y, yaw, **kwargs)

    def _build_metrics(self) -> dict[str, Any]:
        running = [sample for sample in self._samples if sample["running"]]
        position_errors = [sample["distance_m"] for sample in running]
        heading_errors = [sample["heading_error_rad"] for sample in running]
        pose_errors = [sample["pose_error"] for sample in running]
        wind_speed = [sample["wind_speed"] for sample in running]
        wind_direction = [sample["wind_direction"] for sample in running]
        pose_stats = distribution(pose_errors)
        position_stats = distribution(position_errors)
        heading_stats = distribution(heading_errors)
        heading_stats_deg = distribution([math.degrees(value) for value in heading_errors])
        axes = {}
        for axis in ("left", "right", "lateral"):
            axes[axis] = distribution([sample["thrust"].get(axis) for sample in running])
        wind_speed_stats = distribution(wind_speed)
        wind_direction_stats = distribution(wind_direction)
        metrics = {
            "schema": "vrx_station_keeping_metrics/v1",
            # These compact aliases make the report easy to consume from a
            # spreadsheet while the nested distributions below retain the
            # complete count/RMS/P95 information.
            "duration_s": self.duration,
            "sample_count": len(running),
            "target": {"x_m": self.target[0], "y_m": self.target[1], "yaw_rad": self.target[2]},
            "window": {
                "start_delay_s": self.start_delay,
                "duration_s": self.duration,
                "end_time_s": self.end_time,
                "interval": "[start_delay, start_delay + duration)",
            },
            "samples": {
                "total": len(self._samples),
                "running": len(running),
                "by_phase": dict(self._phase_counts),
            },
            "score": {
                "pose_error_mean": pose_stats["mean"],
                "official_mean": pose_stats["mean"],
                "pose_error": pose_stats,
                "position_error_m": position_stats,
                "heading_error_rad": heading_stats,
                "heading_error_deg": heading_stats_deg,
                "direction": "lower_is_better",
            },
            "max_position_error_m": position_stats["max"],
            "rms_position_error_m": position_stats["rms"],
            "p95_position_error_m": position_stats["p95"],
            "max_heading_error_rad": heading_stats["max"],
            "rms_heading_error_rad": heading_stats["rms"],
            "p95_heading_error_rad": heading_stats["p95"],
            "max_heading_error_deg": heading_stats_deg["max"],
            "rms_heading_error_deg": heading_stats_deg["rms"],
            "p95_heading_error_deg": heading_stats_deg["p95"],
            "wind": {
                # Keep short keys convenient for ad-hoc analysis while the
                # unit-suffixed aliases make the JSON self-describing.
                "speed": wind_speed_stats,
                "speed_mps": wind_speed_stats,
                "direction": wind_direction_stats,
                "direction_deg": wind_direction_stats,
                "missing_speed_count": sum(1 for value in wind_speed if _finite(value) is None),
                "missing_direction_count": sum(1 for value in wind_direction if _finite(value) is None),
            },
            "wind_speed_mean_mps": wind_speed_stats["mean"],
            "wind_speed_max_mps": wind_speed_stats["max"],
            "wind_direction_mean_deg": wind_direction_stats["mean"],
            "thrust": {
                "axes": axes,
                "invalid_command_count": self._invalid_thrust_count,
                "missing_command_count": self._missing_thrust_count,
                "limit": self.thrust_limit,
            },
            "timing": {
                "max_sample_gap_s": self._max_gap,
                "odom_gap_max_s": self._max_gap,
                "time_regression_count": self._time_regressions,
            },
            "status": "complete" if running else "no_running_samples",
        }
        return _json_safe(metrics)

    def finalize(self) -> dict[str, Any]:
        """Flush events and write ``metrics.json``/``summary.json``.

        The detailed metrics file is the machine-readable source of truth;
        ``summary.json`` is a small convenience artifact matching the existing
        validation-directory convention.
        """

        if self._metrics is not None:
            return self._metrics
        self._metrics = self._build_metrics()
        try:
            self._events.close()
        finally:
            (self.output / "metrics.json").write_text(
                json.dumps(self._metrics, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
            summary = {
                "schema": "vrx_station_keeping_summary/v1",
                "status": self._metrics["status"],
                "duration_s": self._metrics["duration_s"],
                "sample_count": self._metrics["sample_count"],
                "official_pose_error_mean": self._metrics["score"]["official_mean"],
                "window": self._metrics["window"],
                "samples": self._metrics["samples"],
                "artifacts": ["events.jsonl", "metadata.json", "metrics.json", "summary.json"],
            }
            (self.output / "summary.json").write_text(
                json.dumps(_json_safe(summary), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
            self._closed = True
        return self._metrics

    def close(self) -> dict[str, Any]:
        """Alias for ``finalize`` for use in ``finally`` blocks."""

        return self.finalize()

    def __enter__(self) -> "StationKeepingRecorder":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.finalize()


def _merge_topics(topics: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    if topics is not None and not isinstance(topics, Mapping):
        raise ValueError("topics must be a mapping")
    merged = {
        "model_states": DEFAULT_TOPICS["model_states"],
        "wind_speed": DEFAULT_TOPICS["wind_speed"],
        "wind_direction": DEFAULT_TOPICS["wind_direction"],
        "clock": DEFAULT_TOPICS["clock"],
        "thrusts": dict(DEFAULT_TOPICS["thrusts"]),
    }
    if topics:
        for key in ("model_states", "wind_speed", "wind_direction", "clock"):
            if key in topics and topics[key]:
                merged[key] = str(topics[key])
        if isinstance(topics.get("thrusts"), Mapping):
            for axis in ("left", "right", "lateral"):
                if topics["thrusts"].get(axis):
                    merged["thrusts"][axis] = str(topics["thrusts"][axis])
    return merged


def make_clock_qos(qos_profile_type: Any, reliability_policy: Any, durability_policy: Any) -> Any:
    """Build the ROS 2 Humble ``ClockQoS`` equivalent.

    Humble's Python API does not export ``qos_profile_clock``.  Gazebo Classic
    publishes ``/clock`` with keep-last depth 1, best-effort reliability and
    volatile durability; using the default reliable subscription is
    incompatible and can leave simulated time permanently at zero.
    """

    return qos_profile_type(
        depth=1,
        reliability=reliability_policy.BEST_EFFORT,
        durability=durability_policy.VOLATILE,
    )


def _quaternion_yaw(quaternion: Any) -> float:
    """Extract planar yaw from a ROS quaternion-like object."""

    x = _require_finite(getattr(quaternion, "x", None), "orientation.x")
    y = _require_finite(getattr(quaternion, "y", None), "orientation.y")
    z = _require_finite(getattr(quaternion, "z", None), "orientation.z")
    w = _require_finite(getattr(quaternion, "w", None), "orientation.w")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _ros_sim_seconds(clock_message: Any) -> float:
    stamp = getattr(clock_message, "clock", clock_message)
    return _require_finite(getattr(stamp, "sec", None), "clock.sec") + _require_finite(
        getattr(stamp, "nanosec", None), "clock.nanosec"
    ) * 1e-9


def run_ros(args: argparse.Namespace) -> dict[str, Any]:
    """Run the read-only ROS 2 observer until the running window ends."""

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from gazebo_msgs.msg import ModelStates
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float32, Float64
    except ImportError as error:  # pragma: no cover - depends on ROS installation
        raise RuntimeError("ROS 2 Python packages are required for live recording; pure scoring needs no ROS") from error

    topics = {
        "model_states": args.model_states,
        "wind_speed": args.wind_speed,
        "wind_direction": args.wind_direction,
        "clock": args.clock,
        "thrusts": {"left": args.left_thrust, "right": args.right_thrust, "lateral": args.lateral_thrust},
    }
    recorder = StationKeepingRecorder(
        output=args.output,
        target=(args.x, args.y, args.yaw),
        start_delay=args.start_delay,
        duration=args.duration,
        model_name=args.model_name,
        topics=topics,
        thrust_limit=args.thrust_limit,
        metadata={"command": list(getattr(args, "_argv", sys.argv[1:])),
                  "world": args.world, "ros_read_only": True,
                  "time_base": "wall" if args.wall_time else "sim"},
    )

    class Observer(Node):  # pragma: no cover - requires a ROS/Gazebo runtime
        def __init__(self) -> None:
            super().__init__(
                "station_keeping_recorder",
                parameter_overrides=[Parameter("use_sim_time", value=not args.wall_time)],
            )
            self.latest_clock: Optional[float] = None
            # Wall-time mode is deliberately relative to observer startup;
            # using the Unix epoch would skip the configured scoring window.
            self._wall_start = time.monotonic()
            self.latest_wind_speed: Optional[float] = None
            self.latest_wind_direction: Optional[float] = None
            self.latest_thrust: dict[str, Optional[float]] = {"left": None, "right": None, "lateral": None}
            self.received_pose = False
            clock_qos = make_clock_qos(QoSProfile, ReliabilityPolicy, DurabilityPolicy)
            self.create_subscription(Clock, topics["clock"], self._on_clock, clock_qos)
            # Gazebo model state is a best-effort sensor stream; use a shallow
            # QoS profile to match Gazebo's publisher without blocking.
            try:
                sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                        durability=DurabilityPolicy.VOLATILE)
            except Exception:
                sensor_qos = 10
            self.create_subscription(ModelStates, topics["model_states"], self._on_model_states, sensor_qos)
            self.create_subscription(Float64, topics["wind_speed"], self._on_wind_speed, 10)
            self.create_subscription(Float64, topics["wind_direction"], self._on_wind_direction, 10)
            for axis, topic in topics["thrusts"].items():
                self.create_subscription(Float32, topic, lambda msg, axis=axis: self._on_thrust(axis, msg), 10)

        def _on_clock(self, message: Any) -> None:
            try:
                self.latest_clock = _ros_sim_seconds(message)
            except ValueError:
                return

        @staticmethod
        def _scalar(message: Any) -> Optional[float]:
            return _finite(getattr(message, "data", message))

        def _on_wind_speed(self, message: Any) -> None:
            self.latest_wind_speed = self._scalar(message)

        def _on_wind_direction(self, message: Any) -> None:
            self.latest_wind_direction = self._scalar(message)

        def _on_thrust(self, axis: str, message: Any) -> None:
            self.latest_thrust[axis] = self._scalar(message)

        def _sim_time(self) -> float:
            if args.wall_time:
                return max(0.0, time.monotonic() - self._wall_start)
            if self.latest_clock is not None:
                return self.latest_clock
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_model_states(self, message: Any) -> None:
            try:
                index = list(message.name).index(args.model_name)
            except (ValueError, AttributeError):
                return
            try:
                pose = message.pose[index]
                yaw = _quaternion_yaw(pose.orientation)
                position = pose.position
                x, y = _require_finite(position.x, "position.x"), _require_finite(position.y, "position.y")
                velocity = None
                if getattr(message, "twist", None) and len(message.twist) > index:
                    twist = message.twist[index]
                    velocity = [
                        _finite(getattr(twist.linear, "x", None)),
                        _finite(getattr(twist.linear, "y", None)),
                        _finite(getattr(twist.angular, "z", None)),
                    ]
                self.received_pose = True
                recorder.record_sample(
                    self._sim_time(), x, y, yaw,
                    wind_speed=self.latest_wind_speed,
                    wind_direction=self.latest_wind_direction,
                    thrusts=self.latest_thrust,
                    world_velocity=velocity,
                )
            except (IndexError, AttributeError, ValueError):
                return

    rclpy.init(args=[])
    node = Observer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.received_pose and node._sim_time() >= recorder.end_time:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return recorder.finalize()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new output directory (must not already exist)")
    parser.add_argument("--x", "--target-x", dest="x", type=float, required=True, help="target x in metres")
    parser.add_argument("--y", "--target-y", dest="y", type=float, required=True, help="target y in metres")
    parser.add_argument("--yaw", "--target-yaw", dest="yaw", type=float, required=True,
                        help="target yaw in radians")
    parser.add_argument("--start-delay", type=float, default=20.0, help="running window delay in simulation seconds")
    parser.add_argument("--duration", type=float, default=300.0, help="running window duration in simulation seconds")
    parser.add_argument("--model-name", default=DEFAULT_MODEL_NAME, help="Gazebo model name to observe")
    parser.add_argument("--world", default="stationkeeping_vrx",
                        help="world label stored in metadata (does not launch or modify Gazebo)")
    parser.add_argument("--thrust-limit", type=float, default=1.0, help="absolute normalized command limit; use --no-thrust-limit to disable")
    parser.add_argument("--no-thrust-limit", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--wall-time", action="store_true",
                        help="use elapsed wall seconds instead of /clock for the scoring window")
    parser.add_argument("--model-states", default=DEFAULT_TOPICS["model_states"])
    parser.add_argument("--wind-speed", default=DEFAULT_TOPICS["wind_speed"])
    parser.add_argument("--wind-direction", default=DEFAULT_TOPICS["wind_direction"])
    parser.add_argument("--clock", default=DEFAULT_TOPICS["clock"])
    parser.add_argument("--left-thrust", default=DEFAULT_TOPICS["thrusts"]["left"])
    parser.add_argument("--right-thrust", default=DEFAULT_TOPICS["thrusts"]["right"])
    parser.add_argument("--lateral-thrust", default=DEFAULT_TOPICS["thrusts"]["lateral"])
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    args._argv = list(sys.argv[1:] if argv is None else argv)
    if args.no_thrust_limit:
        args.thrust_limit = None
    try:
        metrics = run_ros(args)
    except (FileExistsError, ValueError, RuntimeError) as error:
        parser.error(str(error))
    print(json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":  # pragma: no cover - command-line wrapper
    raise SystemExit(main())
