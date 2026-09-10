"""ROS smoke/regression test; skipped only when rclpy is unavailable.

Run in an isolated ROS_DOMAIN_ID. No Gazebo or running project nodes are used.
"""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from perception.mapping_node import MappingNode


def test_exact_tf_duplicate_expiry_clock_rollback_and_map_publication():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/perception_smoke",
                     "-p", "width:=8", "-p", "resolution:=1.0",
                     "-p", "use_sim_time:=true"])
    mapper = driver = executor = None
    try:
        mapper = MappingNode()
        driver = Node("mapping_test_driver")
        executor = SingleThreadedExecutor()
        executor.add_node(mapper)
        executor.add_node(driver)
        clock_pub = driver.create_publisher(Clock, "/clock", 10)
        scan_pub = driver.create_publisher(LaserScan, "scan", 10)
        broadcaster = TransformBroadcaster(driver)
        maps = []
        map_sub = driver.create_subscription(OccupancyGrid, "map", maps.append, 10)

        def spin(seconds=0.05):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.005)

        def until(predicate, timeout=1.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.005)
            assert predicate()

        def set_clock(seconds):
            message = Clock()
            message.clock.sec = int(seconds)
            message.clock.nanosec = round((seconds - int(seconds)) * 1e9)
            clock_pub.publish(message)
            until(lambda: mapper.get_clock().now().nanoseconds == round(seconds * 1e9))
            spin(0.03)

        def transform(seconds):
            value = TransformStamped()
            value.header.frame_id = "perception_smoke/map"
            value.child_frame_id = "perception_smoke/lidar"
            value.header.stamp.sec = int(seconds)
            value.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
            value.transform.rotation.w = 1.0
            broadcaster.sendTransform(value)
            spin(0.03)

        def scan(seconds):
            value = LaserScan()
            value.header.frame_id = "perception_smoke/lidar"
            value.header.stamp.sec = int(seconds)
            value.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
            value.angle_min = value.angle_increment = 0.0
            value.range_min, value.range_max = 0.1, 20.0
            value.ranges = [2.0]
            scan_pub.publish(value)

        until(lambda: scan_pub.get_subscription_count() == 1)
        spin(0.1)
        set_clock(10.0)
        transform(10.0)
        scan(10.0)
        until(lambda: mapper.integrated_scans == 1)
        assert mapper.mapper.data[4, 6] == 54
        snapshot = mapper.mapper.data.copy()
        scan(10.0)
        until(lambda: mapper.pending.counters.duplicate == 1)
        spin(0.1)
        np.testing.assert_array_equal(mapper.mapper.data, snapshot)

        # A newer scan has no exact-time TF and must not use the transform at 10s.
        set_clock(10.1)
        scan(10.1)
        until(lambda: mapper.pending.counters.expired == 1)
        assert mapper.integrated_scans == 1
        np.testing.assert_array_equal(mapper.mapper.data, snapshot)

        set_clock(10.2)
        scan(10.2)
        until(lambda: len(mapper.pending) == 1)
        set_clock(1.0)
        assert mapper.pending.counters.resets == 1
        assert len(mapper.pending) == 0
        assert np.all(mapper.mapper.data == -1)
        # A lower scan stamp becomes valid after rollback clears dedup history.
        transform(1.0)
        scan(1.0)
        until(lambda: mapper.integrated_scans == 2)
        assert mapper.mapper.data[4, 6] == 54
        maps.clear()
        set_clock(1.6)
        until(lambda: bool(maps))
        assert maps[-1].header.frame_id == "perception_smoke/map"
        assert maps[-1].info.width == 8
        assert maps[-1].data[4 * 8 + 6] == 54
        result = mapper.set_parameters([Parameter("width", Parameter.Type.INTEGER, 10)])
        assert not result[0].successful
    finally:
        if executor is not None:
            executor.shutdown()
        if mapper is not None:
            mapper.pending.reset()
            mapper.destroy_node()
        if driver is not None:
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
