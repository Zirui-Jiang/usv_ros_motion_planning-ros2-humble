#!/usr/bin/env python3
"""Replay captured raw-map/state/path inputs, without ROS or Gazebo.

The snapshot is taken immediately before the goal publication and the route is
the first nonempty returned path. Their times are reported separately: this is
a reproducible captured-input comparison, not a claim of bit-exact replication
of a later worker's private input snapshot.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from common.math_helper import path_is_clear
from common.models import GridSnapshot, USVModel, VesselState
from trajectory_planner.los_trajectory_planner import LosTrajectoryPlanner
from metrics import swept_clearance


def read_inputs(trial):
    events = []
    with (trial / "events.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            events.append(json.loads(line))
    snapshot = next(e for e in events if e["event"] == "goal_input_snapshot")
    state_data = json.loads((trial / snapshot["state_path"]).read_text(encoding="utf-8"))
    parameters = next(e["parameters"] for e in events if e["event"] == "controller_verified")
    route_event = next(e for e in events if e["event"] == "reference_path" and e.get("points"))
    with np.load(trial / snapshot["costmap_path"], allow_pickle=False) as capture:
        if str(capture["cost_kind"]) != "raw":
            raise ValueError("This replay requires lossless raw costmap data; OccupancyGrid cannot supply exact raw thresholds")
        grid = GridSnapshot(capture["data"], float(capture["resolution"]),
                            float(capture["origin_x"]), float(capture["origin_y"]),
                            float(capture["stamp"]), str(capture["frame_id"]))
    state = VesselState(*state_data["pose"], *state_data["body_velocity"],
                        stamp=state_data["stamp"], frame_id=state_data["frame_id"])
    xy = np.asarray(route_event["points"], dtype=float)
    route = np.column_stack((xy[:, :2], np.zeros(len(xy))))
    # The runtime replaces the search cell centre with the precise goal before LOS.
    route[-1] = state_data["goal"]
    return grid, state, route, parameters, snapshot, route_event


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("--lookaheads", type=float, nargs="+", default=[5.0, 1.0])
    parser.add_argument("--cruising-speed", type=float, default=1.0,
                        help="fallback only when live parameter capture lacks dynamics.cruising_speed; reported explicitly")
    parser.add_argument("--cylinder", type=float, nargs=3, metavar=("X", "Y", "RADIUS"))
    parser.add_argument("--output", type=Path, required=True, help="new JSON file; never overwrites")
    args = parser.parse_args(argv)
    grid, state, route, parameters, snapshot, route_event = read_inputs(args.trial)
    prefix = "usv_simulation.trajectory_planner.los."
    config = {name[len(prefix):]: value for name, value in parameters.items() if name.startswith(prefix)}
    controller_prefix = "usv_simulation.controller."
    controller = parameters[controller_prefix + "controller_name"]
    config["goal_tolerance"] = min(config.get("goal_tolerance", 1.0),
                                   parameters[controller_prefix + controller + "_controller.goal_distance_tolerance"])
    factor = parameters.get("obstacle_factor", parameters.get("path_planner.obstacle_factor"))
    if factor is None:
        raise ValueError("Captured parameters omit obstacle_factor; refusing to guess the raw threshold")
    speed = parameters.get("dynamics.cruising_speed", args.cruising_speed)
    model = USVModel(cruising_speed=speed)
    results = []
    for lookahead in args.lookaheads:
        effective = dict(config, lookahead_distance=lookahead)
        result = LosTrajectoryPlanner(model, effective).plan(route, state)
        points = result.trajectory.points
        gap = None
        if args.cylinder and len(points) > 1:
            obstacle = dict(zip(("x", "y", "radius"), args.cylinder))
            gap = min(swept_clearance(a[[2, 3, 4]], b[[2, 3, 4]], obstacle)
                      for a, b in zip(points[:-1], points[1:]))
        results.append({"lookahead_distance_m": lookahead, "status": result.status.value,
                        "message": result.message, "config": effective,
                        "raw_path_is_clear": bool(len(points) and path_is_clear(grid, points[:, [2, 3, 4]], 254.0 * factor)),
                        "predicted_minimum_footprint_clearance_m": gap,
                        "trajectory_columns": ["time", "s", "x", "y", "theta", "curv", "velocity", "acceleration"],
                        "trajectory": points.tolist()})
    report = {"classification": "offline captured-input replay, not a closed-loop result",
              "trial": str(args.trial.resolve()), "source_snapshot": snapshot,
              "state_stamp": state.stamp, "map_stamp": grid.stamp,
              "first_route_sim_time_s": route_event.get("sim_time_s"),
              "raw_threshold": 254.0 * factor, "cruising_speed_mps": speed,
              "cruising_speed_source": "captured live parameter" if "dynamics.cruising_speed" in parameters else "explicit CLI/default fallback",
              "cylinder": args.cylinder, "footprint_length_width_m": [2.2, 1.2],
              "reference_route": route.tolist(), "comparisons": results}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({"output": str(args.output), "comparisons": [
        {key: row[key] for key in ("lookahead_distance_m", "status", "raw_path_is_clear", "predicted_minimum_footprint_clearance_m")}
        for row in results]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
