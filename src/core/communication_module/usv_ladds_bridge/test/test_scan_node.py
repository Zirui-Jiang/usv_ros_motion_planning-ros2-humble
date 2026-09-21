"""ROS smoke checks for the direct LaserScan adapter node."""

import pytest


rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("tf2_ros")

from usv_ladds_bridge.scan_node import Scan0141Node


def test_scan_node_constructs_with_rclpy_default_use_sim_time():
    """rclpy predeclares use_sim_time, so the adapter must not redeclare it."""

    already_initialized = rclpy.ok()
    if not already_initialized:
        rclpy.init(args=[])
    node = None
    try:
        node = Scan0141Node()
        assert node.get_name() == "usv_ladds_scan"
    finally:
        if node is not None:
            node.destroy_node()
        if not already_initialized and rclpy.ok():
            rclpy.shutdown()

