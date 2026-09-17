#!/usr/bin/env python3
"""Exercise real Costmap2DROS activation and SIGINT without Gazebo.

Source ROS Humble, select an unused ROS_DOMAIN_ID, then supply --executable.
Logs distinguish startup failures, shutdown timeouts and the reported SIGSEGV.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def run_once(executable, directory, attempt, fixture, scenario, active_seconds, overrides, debugger):
    name = f"costmap_shutdown_{attempt}"
    config = directory / "costmap.yaml"
    config.write_text("""/**:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 20.0
    publish_frequency: 10.0
    width: 10
    height: 10
    resolution: 0.5
    plugins: [inflation_layer]
    inflation_layer:
      plugin: nav2_costmap_2d::InflationLayer
""")
    if scenario in ("traffic", "startup_cancel"):
        config.write_text("""/**:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: true
    update_frequency: 2.0
    publish_frequency: 2.0
    width: 400
    height: 400
    resolution: 0.5
    rolling_window: true
    always_send_full_costmap: true
    plugins: [obstacle_layer, inflation_layer]
    obstacle_layer:
      plugin: nav2_costmap_2d::ObstacleLayer
      observation_sources: scan
      scan:
        topic: /shutdown_scan
        data_type: LaserScan
        clearing: true
        marking: true
        inf_is_valid: true
        obstacle_max_range: 20.0
        raytrace_max_range: 22.0
    inflation_layer:
      plugin: nav2_costmap_2d::InflationLayer
""")
    if overrides:
        content = yaml.safe_load(config.read_text())
        for override in overrides:
            parameter_name, value = override.split(":=", 1)
            content["/**"]["ros__parameters"][parameter_name] = yaml.safe_load(value)
        config.write_text(yaml.safe_dump(content))
    log_path = directory / f"attempt-{attempt}.log"
    client = fixture.create_client(GetState, f"/{name}/get_state")
    request = None
    active = False
    waiting_for_tf = False
    with log_path.open("w") as log:
        command = [str(executable), "--ros-args", "-r", f"__node:={name}",
                   "--params-file", str(config)]
        if debugger:
            command = ["gdb", "--batch", "-ex", "set pagination off",
                       "-ex", "set debuginfod enabled off",
                       "-ex", "handle SIGINT nostop noprint pass", "-ex", "run",
                       "-ex", "thread apply all bt 16", "--args"] + command
        process = subprocess.Popen(
            command,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(fixture, timeout_sec=0.01)
                if scenario == "startup_cancel":
                    waiting_for_tf = "Timed out waiting for transform" in log_path.read_text()
                    if waiting_for_tf:
                        break
                if request is None and client.service_is_ready():
                    request = client.call_async(GetState.Request())
                if request is not None and request.done():
                    response = request.result()
                    active = bool(response and response.current_state.id == 3)
                    if active:
                        break
                    request = None
            # Leave the active update/publishing loop running before SIGINT.
            deadline = time.monotonic() + active_seconds
            while active and time.monotonic() < deadline:
                rclpy.spin_once(fixture, timeout_sec=0.01)
            if process.poll() is None:
                target = process.pid
                if debugger:
                    children = Path(f"/proc/{process.pid}/task/{process.pid}/children").read_text().split()
                    if not children:
                        raise RuntimeError("GDB inferior was not created")
                    target = int(children[0])
                os.kill(target, signal.SIGINT)
            try:
                exit_code = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
                exit_code = "shutdown_timeout"
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            fixture.destroy_client(client)
    output = log_path.read_text(errors="replace")
    crashed = exit_code == -signal.SIGSEGV or "received signal SIGSEGV" in output
    cleaned_up = "Cleaning up" in output
    deactivated = "Deactivating" in output
    if scenario == "startup_cancel":
        # Interrupted activation returns startup failure (1), not SIGABRT (-6).
        passed = waiting_for_tf and not active and exit_code == 1 and not crashed
    else:
        passed = active and deactivated and cleaned_up and exit_code == 0 and not crashed
    return dict(attempt=attempt, scenario=scenario, active=active, exit_code=exit_code,
                waiting_for_tf=waiting_for_tf,
                cleaned_up=cleaned_up, deactivated=deactivated,
                reported_sigsegv=crashed, log=str(log_path),
                passed=passed)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True, type=Path)
    parser.add_argument("--artifacts-dir", required=True, type=Path)
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--scenario", choices=["idle", "traffic", "startup_cancel"], default="traffic")
    parser.add_argument("--active-seconds", type=float, default=1.1)
    parser.add_argument("--override", action="append", default=[])
    parser.add_argument("--no-scan", action="store_true")
    parser.add_argument("--gdb", action="store_true")
    args = parser.parse_args()
    args.artifacts_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    fixture = Node("costmap_shutdown_fixture")
    broadcaster = StaticTransformBroadcaster(fixture)
    transform = TransformStamped()
    transform.header.stamp = fixture.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.rotation.w = 1.0
    if args.scenario != "startup_cancel":
        broadcaster.sendTransform(transform)
    if args.scenario in ("traffic", "startup_cancel"):
        clock_pub = fixture.create_publisher(Clock, "/clock", 10)
        scan_pub = fixture.create_publisher(LaserScan, "/shutdown_scan", qos_profile_sensor_data)
        scan = LaserScan()
        scan.header.frame_id = "base_link"
        scan.angle_min, scan.angle_max = -3.14159265, 3.14159265
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 1999
        scan.range_min, scan.range_max = 1.2, 25.0
        scan.ranges = [10.0] * 2000

        def publish_clock():
            message = Clock()
            message.clock = fixture.get_clock().now().to_msg()
            clock_pub.publish(message)

        def publish_scan():
            scan.header.stamp = fixture.get_clock().now().to_msg()
            scan_pub.publish(scan)

        clock_timer = fixture.create_timer(0.01, publish_clock)
        scan_timer = (None if args.no_scan else
                      fixture.create_timer(1.0 / 15.0, publish_scan))
        costmap_sub = fixture.create_subscription(Costmap, "/costmap_raw", lambda message: None, 10)
    try:
        results = [run_once(args.executable, args.artifacts_dir, number, fixture,
                            args.scenario, args.active_seconds, args.override, args.gdb)
                   for number in range(args.attempts)]
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
    print(json.dumps(results, indent=2))
    (args.artifacts_dir / "summary.json").write_text(json.dumps(results, indent=2))
    return 0 if all(item["passed"] for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
