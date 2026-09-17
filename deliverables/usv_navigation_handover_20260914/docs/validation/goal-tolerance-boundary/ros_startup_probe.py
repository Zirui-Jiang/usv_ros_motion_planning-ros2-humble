#!/usr/bin/env python3
"""One isolated real-rclpy startup assertion; does not launch Gazebo or spin."""
import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=("pid", "lqr"), required=True)
    parser.add_argument("--tolerance", type=float, required=True)
    parser.add_argument("--expected", choices=("accepted", "rejected"), required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    for relative in ("src/core/common", "src/core/path_planner/path_planner",
                     "src/core/path_planner/path_processor", "src/core/trajectory_planner",
                     "src/core/controller/controller", "src/sim_env"):
        sys.path.insert(0, str(root / relative))
    import rclpy
    from sim_env.configuration import C
    from sim_env.simulation import Simulator

    key = C + args.controller + "_controller.goal_distance_tolerance"
    report = {"controller": args.controller, "parameter": key,
              "requested_value": args.tolerance, "pid": os.getpid(),
              "domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "simulation_module": sys.modules["sim_env.simulation"].__file__}
    simulator = None
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/goal_tolerance_probe_{os.getpid()}",
                    "-p", C + "controller_name:=" + args.controller,
                    "-p", key + ":=" + format(args.tolerance, ".17e")])
    try:
        try:
            simulator = Simulator()
        except ValueError as error:
            report.update(observed="rejected", exception_type=type(error).__name__,
                          error=str(error))
            assert key in str(error) and "greater than 1e-9 m" in str(error)
        else:
            report.update(observed="accepted", actual_value=simulator.get_parameter(key).value)
            assert report["actual_value"] == args.tolerance
        assert report["observed"] == args.expected
    finally:
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report["probe_passed"] = True
    print(json.dumps(report))


if __name__ == "__main__":
    main()
