#!/usr/bin/env python3
"""One numerical PID/LQR step; JSON is a demonstration, not an autopilot protocol."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("pid", "lqr"), default="pid")
    args = parser.parse_args()
    here = Path(__file__).resolve()
    workspace = here.parents[1]
    if (workspace / "src/controller/controller").is_dir():
        package_roots = [workspace / "src" / name for name in ("common", "trajectory_planner", "controller")]
    else:
        repository = here.parents[2]
        package_roots = [repository / relative for relative in (
            "src/core/common", "src/core/trajectory_planner", "src/core/controller/controller")]
    for package_root in package_roots:
        sys.path.insert(0, str(package_root))

    from common import USVModel, VesselState
    from controller import create_controller
    from trajectory_planner import Trajectory

    dt = 0.1
    # Current PID calibration has nonzero gains; PID constructor defaults are zero.
    config = {
        "pid": {"goal_distance_tolerance": 1.5, "goal_speed_tolerance": 0.25,
                "lookahead_time": 2.6, "k_p_speed": 2.0, "k_i_speed": 0.5,
                "k_d_speed": 0.0, "k_p_course": 2.0, "k_i_course": 0.0, "k_d_course": 0.0},
        "lqr": {"goal_distance_tolerance": 1.0, "goal_speed_tolerance": 0.25,
                "lookahead_time": 2.0, "discrete_dt": dt, "solve_eps": 0.1,
                "max_iterations": 5000, "solve_timeout": 0.08,
                "Q_matrix_diag": [1, 1, 100, 10, 10, 10], "R_matrix_diag": [20, 20]},
    }[args.controller]
    controller = create_controller(args.controller, USVModel(), config)
    state = VesselState(x=0.0, y=0.1, psi=0.05, u=0.3, v=0.0, r=0.0,
                        stamp=100.0, frame_id="otter/map")
    # Columns: relative t (s), arc length (m), x/y (m), yaw (rad), curvature
    # (1/m), reference speed (m/s), reference acceleration (m/s^2).
    trajectory = Trajectory([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0],
        [5.0, 4.0, 4.0, 0.0, 0.0, 0.0, 0.8, 0.0],
        [10.0, 8.0, 8.0, 0.0, 0.0, 0.0, 0.8, 0.0],
    ], stamp=100.0, frame_id="otter/map", goal=(8.0, 0.0))
    command = controller.compute(state, trajectory, dt)
    record = {"demo_only": True, "controller": args.controller, "dt_s": dt,
              "state": asdict(state), "command": asdict(command)}
    print(json.dumps(record, ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if command.is_valid() and command.status.value == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
