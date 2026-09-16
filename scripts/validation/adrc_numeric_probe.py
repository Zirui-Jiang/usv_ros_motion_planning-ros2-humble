#!/usr/bin/env python3
"""Run deterministic ADRC tracking checks through the production adapter."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
for relative in (
        "src/core/shared/common", "src/core/control_module/controller",
        "src/core/decision_module/trajectory_planner", "src/core/shared/usv_navigation"):
    sys.path.insert(0, str(ROOT / relative))

from common.dynamics import step_dynamics
from common.models import AlgorithmStatus, USVModel, VesselState
from controller import ADRCController
from trajectory_planner.trajectory import Trajectory
from usv_navigation.configuration import C, flatten, load_yaml, merge_explicit, resolve_parameters, section


DT = 0.1
DURATION = 180.0
TAIL_DURATION = 20.0
SPEED_TAIL_MAE_MAX = 0.1
HEADING_TAIL_MAE_DEG_MAX = 5.0
DEFAULT_DYNAMICS_CONFIG = (
    ROOT / "src/core/shared/usv_navigation/config/robots/otter/dynamics_otter.yaml")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def _trajectory(speed, heading):
    horizon = DURATION + 10.0
    return Trajectory([
        [0.0, 0.0, 0.0, 0.0, heading, 0.0, speed, 0.0],
        [horizon, horizon * speed, horizon * speed * math.cos(heading),
         horizon * speed * math.sin(heading), heading, 0.0, speed, 0.0],
    ], stamp=0.0, frame_id="otter/map", goal=None)


def _run_scenario(name, desired_speed, desired_heading, initial_speed, model, config):
    controller = ADRCController(model, config)
    trajectory = _trajectory(desired_speed, desired_heading)
    state = VesselState(u=initial_speed, frame_id="otter/map")
    rows, compute_times = [], []
    steps = round(DURATION / DT)
    for index in range(steps):
        begun = time.perf_counter()
        command = controller.compute(state, trajectory, DT)
        compute_elapsed = time.perf_counter() - begun
        compute_times.append(compute_elapsed)
        if command.status == AlgorithmStatus.SUCCESS:
            state = step_dynamics(state, command, model, DT)
        heading_error = math.atan2(math.sin(desired_heading - state.psi),
                                   math.cos(desired_heading - state.psi))
        rows.append({
            "scenario": name, "time_s": (index + 1) * DT,
            "reference_speed_mps": desired_speed,
            "reference_heading_rad": desired_heading,
            "reference_heading_deg": math.degrees(desired_heading),
            "state_x_m": state.x, "state_y_m": state.y,
            "state_heading_rad": state.psi, "state_u_mps": state.u,
            "state_v_mps": state.v, "state_yaw_rate_radps": state.r,
            "speed_error_mps": desired_speed - state.u,
            "heading_error_deg": math.degrees(heading_error),
            "command_left": command.left, "command_right": command.right,
            "command_lateral": command.lateral, "command_status": command.status.value,
            "control_elapsed_s": compute_elapsed,
        })
    tail = rows[-round(TAIL_DURATION / DT):]
    finite = all(all(math.isfinite(float(row[key])) for key in (
        "state_x_m", "state_y_m", "state_heading_rad", "state_u_mps", "state_v_mps",
        "state_yaw_rate_radps", "command_left", "command_right", "command_lateral")) for row in rows)
    bounded = all(abs(row[key]) <= 1.0 for row in rows
                  for key in ("command_left", "command_right", "command_lateral"))
    successful = all(row["command_status"] == AlgorithmStatus.SUCCESS.value for row in rows)
    speed_mae = float(sum(abs(row["speed_error_mps"]) for row in tail) / len(tail))
    heading_mae = float(sum(abs(row["heading_error_deg"]) for row in tail) / len(tail))
    tracking_passed = (speed_mae <= SPEED_TAIL_MAE_MAX if name == "speed_1mps"
                       else heading_mae <= HEADING_TAIL_MAE_DEG_MAX)
    metrics = {
        "passed": bool(finite and bounded and successful and tracking_passed),
        "finite_states_and_commands": finite,
        "bounded_commands": bounded,
        "all_control_status_success": successful,
        "tail_speed_mae_mps": speed_mae,
        "tail_heading_mae_deg": heading_mae,
        "steps": steps,
        "compute_wall_time_s": {
            "mean": sum(compute_times) / len(compute_times),
            "p99": _percentile(compute_times, 0.99),
            "max": max(compute_times),
            "over_100ms_count": sum(value >= 0.1 for value in compute_times),
        },
    }
    return rows, metrics, compute_times


def _load_configuration(controller_path, dynamics_path):
    values = flatten(load_yaml(dynamics_path))
    merge_explicit(values, flatten(load_yaml(controller_path), "usv_simulation"))
    resolved = resolve_parameters(values)
    if resolved[C + "controller_name"] != "adrc":
        raise ValueError("controller configuration must select adrc")
    model = USVModel.from_mapping(section(resolved, "dynamics."))
    return model, section(resolved, C + "adrc_controller."), resolved


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-config", type=Path, required=True)
    parser.add_argument("--dynamics-config", type=Path, default=DEFAULT_DYNAMICS_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    controller_path = args.controller_config.resolve()
    dynamics_path = args.dynamics_config.resolve()
    model, config, _ = _load_configuration(controller_path, dynamics_path)
    args.output.mkdir(parents=True, exist_ok=False)

    scenarios = (
        ("speed_1mps", 1.0, 0.0, 0.0),
        ("heading_plus_30deg", 0.3, math.radians(30.0), 0.3),
        ("heading_minus_30deg", 0.3, math.radians(-30.0), 0.3),
    )
    all_rows, results, all_times = [], {}, []
    for name, speed, heading, initial_speed in scenarios:
        rows, metrics, compute_times = _run_scenario(
            name, speed, heading, initial_speed, model, config)
        all_rows.extend(rows)
        results[name] = metrics
        all_times.extend(compute_times)

    csv_path = args.output / "samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)

    source_paths = [
        ROOT / "src/core/control_module/controller/controller/adrc_controller.py",
        ROOT / "src/core/control_module/controller/controller/adrc_core/__init__.py",
        ROOT / "src/core/control_module/controller/controller/adrc_core/adrc_controller.py",
        ROOT / "src/core/control_module/controller/controller/adrc_core/pi_controller.py",
        ROOT / "src/core/control_module/controller/controller/adrc_core/heading_linear_regression_filter.py",
        ROOT / "src/core/shared/common/common/dynamics.py",
    ]
    passed = all(metrics["passed"] for metrics in results.values())
    report = {
        "schema_version": 1,
        "passed": passed,
        "thresholds": {
            "speed_tail_mae_mps_max": SPEED_TAIL_MAE_MAX,
            "heading_tail_mae_deg_max": HEADING_TAIL_MAE_DEG_MAX,
            "command_abs_max": 1.0,
            "duration_s": DURATION,
            "tail_duration_s": TAIL_DURATION,
            "dt_s": DT,
        },
        "config": {
            "path": str(controller_path), "sha256": _sha256(controller_path),
            "controller_name": "adrc", "adrc_controller": config,
        },
        "model_config": {
            "path": str(dynamics_path), "sha256": _sha256(dynamics_path),
            "model": asdict(model),
        },
        "sources": [{"path": str(path.relative_to(ROOT)), "sha256": _sha256(path)}
                    for path in source_paths],
        "timing": {
            "definition": "wall-clock time spent in ADRCController.compute per numerical control step; excludes step_dynamics and file output",
            "samples": len(all_times), "mean_s": sum(all_times) / len(all_times),
            "p99_s": _percentile(all_times, 0.99), "max_s": max(all_times),
            "over_100ms_count": sum(value >= 0.1 for value in all_times),
        },
        "scenarios": results,
        "artifacts": {"samples_csv": csv_path.name},
        "scope": "Deterministic common.step_dynamics model check; not Gazebo or real-vessel evidence.",
    }
    (args.output / "result.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"passed": passed, "result": str(args.output / "result.json")}, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
