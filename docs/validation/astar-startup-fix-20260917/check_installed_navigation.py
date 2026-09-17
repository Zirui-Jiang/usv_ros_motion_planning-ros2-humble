#!/usr/bin/env python3
"""Smoke-test the sourced navigation install without importing repository helpers."""

import os
import sys


EXPECTED_DOMAIN_ID = "177"


def require_isolated_ros_environment():
    """Refuse to initialize DDS unless this process is isolated from the user domain."""
    if "rclpy" in sys.modules:
        raise RuntimeError("rclpy was imported before the ROS isolation check")
    domain = os.environ.get("ROS_DOMAIN_ID")
    localhost = os.environ.get("ROS_LOCALHOST_ONLY")
    if domain != EXPECTED_DOMAIN_ID or localhost != "1":
        raise RuntimeError(
            "Refusing to initialize ROS: set ROS_DOMAIN_ID=177 and ROS_LOCALHOST_ONLY=1 "
            f"(got ROS_DOMAIN_ID={domain!r}, ROS_LOCALHOST_ONLY={localhost!r})"
        )
    print(f"[isolation] ROS_DOMAIN_ID={domain} ROS_LOCALHOST_ONLY={localhost}")


def check_installed_imports():
    import path_planner
    import path_planner.path_planner_core as planner_core
    import path_processor
    import usv_decision.decision as decision_module
    from path_processor import ShortcutPathProcessor
    from usv_decision.decision import DecisionNode

    for module in (path_processor, planner_core, path_planner, decision_module):
        print(f"[module] {module.__name__}: {module.__file__}")
    print(f"[import] ShortcutPathProcessor={ShortcutPathProcessor.__module__}.{ShortcutPathProcessor.__name__}")
    print(f"[import] DecisionNode={DecisionNode.__module__}.{DecisionNode.__name__}")

    return path_planner.PathPlannerCore, DecisionNode


def check_planners(path_planner_core):
    import math
    import numpy as np
    from common.models import GridSnapshot

    grid = GridSnapshot(np.zeros((8, 8), dtype=np.uint8), 1.0)
    start, goal = (0.5, 0.5, 0.3), (5.5, 3.5, -0.6)
    raw = path_planner_core({"enable_shortcut": False}).make_plan(start, goal, grid)
    shortened = path_planner_core({"enable_shortcut": True}).make_plan(start, goal, grid)
    assert raw.success and shortened.success, (raw.message, shortened.message)
    assert len(raw.path) > 2, f"unpruned A* unexpectedly has {len(raw.path)} poses"
    assert len(shortened.path) == 2, f"shortcut A* has {len(shortened.path)} poses"
    np.testing.assert_allclose(shortened.path, [start, goal], atol=1e-12)
    assert shortened.expanded == raw.expanded, (shortened.expanded, raw.expanded)
    print(
        f"[planner] A*: shortcut off={len(raw.path)} poses, on={len(shortened.path)} poses, "
        f"expanded={raw.expanded}"
    )

    hybrid_grid = GridSnapshot(np.zeros((100, 100), dtype=np.uint8), 0.25, -5.0, -5.0)
    hybrid_config = {
        "planner_name": "hybrid_astar",
        "minimum_turning_radius": 2.0,
        "motion_step": 0.75,
        "collision_step": 0.2,
        "analytic_expansion_max_length": 16.0,
        "footprint_length": 0.4,
        "footprint_width": 0.3,
        "timeout": 4.0,
        "enable_shortcut": True,
        "enable_resample": True,
    }
    hybrid_goal = (2.0, 2.0, math.pi / 2.0)
    hybrid = path_planner_core(hybrid_config).make_plan((0.0, 0.0, 0.0), hybrid_goal, hybrid_grid)
    assert hybrid.success, hybrid.message
    assert len(hybrid.path) > 10, f"Hybrid A* returned only {len(hybrid.path)} poses"
    np.testing.assert_allclose(hybrid.path[[0, -1]], [(0.0, 0.0, 0.0), hybrid_goal], atol=1e-8)
    print(f"[planner] Hybrid A*: {len(hybrid.path)} poses, expanded={hybrid.expanded}")


def check_decision_ros(decision_node_class):
    import time
    import uuid

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Odometry, Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from usv_navigation_interfaces.msg import NavigationReference

    namespace = "installed_navigation_" + uuid.uuid4().hex[:8]
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            f"__ns:=/{namespace}",
            "-p",
            "planner_name:=astar",
            "-p",
            "path_planner.enable_shortcut:=true",
            "-p",
            "navigation.planning_timeout:=3.0",
        ]
    )
    decision = driver = executor = None
    try:
        decision = decision_node_class()
        driver = Node("installed_navigation_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(decision)
        executor.add_node(driver)

        goal_pub = driver.create_publisher(PoseStamped, "goal", 1)
        odom_pub = driver.create_publisher(Odometry, "odom", 1)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_pub = driver.create_publisher(Costmap, "costmap_raw", map_qos)
        paths, references = [], []
        path_topic = f"/{namespace}/global_path"
        reference_topic = f"/{namespace}/navigation/reference"
        driver.create_subscription(Path, path_topic, paths.append, map_qos)
        driver.create_subscription(NavigationReference, reference_topic, references.append, 10)
        assert decision.path_pub.topic_name == path_topic, decision.path_pub.topic_name
        assert decision.reference_pub.topic_name == reference_topic, decision.reference_pub.topic_name

        discovery_deadline = time.monotonic() + 3.0
        while time.monotonic() < discovery_deadline and any(
            publisher.get_subscription_count() == 0 for publisher in (goal_pub, odom_pub, map_pub)
        ):
            executor.spin_once(timeout_sec=0.02)
        assert all(publisher.get_subscription_count() > 0 for publisher in (goal_pub, odom_pub, map_pub)), (
            goal_pub.get_subscription_count(),
            odom_pub.get_subscription_count(),
            map_pub.get_subscription_count(),
        )

        def publish_inputs():
            stamp = driver.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = decision.frame
            odom.child_frame_id = decision.base_frame
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)

            costmap = Costmap()
            costmap.header.stamp = stamp
            costmap.header.frame_id = decision.frame
            costmap.metadata.size_x = costmap.metadata.size_y = 80
            costmap.metadata.resolution = 0.5
            costmap.metadata.origin.position.x = -10.0
            costmap.metadata.origin.position.y = -10.0
            costmap.metadata.origin.orientation.w = 1.0
            costmap.data = [0] * (80 * 80)
            map_pub.publish(costmap)

        publish_inputs()
        inputs_deadline = time.monotonic() + 3.0
        while time.monotonic() < inputs_deadline and (decision.state is None or decision.grid is None):
            executor.spin_once(timeout_sec=0.02)
        assert decision.state is not None and decision.grid is not None, (decision.state, decision.grid)

        goal = PoseStamped()
        goal.header.stamp = driver.get_clock().now().to_msg()
        goal.header.frame_id = decision.frame
        goal.pose.position.x = 8.0
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)

        deadline = time.monotonic() + 12.0
        next_input = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_input:
                publish_inputs()
                next_input = now + 0.1
            executor.spin_once(timeout_sec=0.02)
            if any(message.poses for message in paths) and any(message.points for message in references):
                break

        nonempty_paths = [message for message in paths if message.poses]
        nonempty_references = [message for message in references if message.points]
        assert nonempty_paths, (
            "no nonempty global_path",
            decision.mode,
            decision.reason,
            dict(decision.counters),
            [len(message.poses) for message in paths],
        )
        assert nonempty_references, (
            "no nonempty NavigationReference",
            decision.mode,
            decision.reason,
            dict(decision.counters),
            [len(message.points) for message in references],
        )
        print(
            f"[ros] {path_topic}: {len(nonempty_paths[-1].poses)} poses; "
            f"{reference_topic}: {len(nonempty_references[-1].points)} points; "
            f"state={decision.mode}"
        )
    finally:
        if executor is not None:
            executor.shutdown()
        if decision is not None:
            decision.destroy_node()
        if driver is not None:
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    require_isolated_ros_environment()
    path_planner_core, decision_node_class = check_installed_imports()
    check_planners(path_planner_core)
    check_decision_ros(decision_node_class)
    print("PASS: installed navigation imports, planners, and ROS publications are healthy")


if __name__ == "__main__":
    main()
