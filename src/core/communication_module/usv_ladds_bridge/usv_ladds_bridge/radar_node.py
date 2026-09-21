# 订阅仿真激光雷达扫描，编成 0141 目标 JSON 交给通信桥发送。
"""ROS 2 node that converts the simulated LiDAR scan into 0141 target JSON."""
from __future__ import annotations

from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from common.geodesy import GeoReference

from .radar_0141 import (
    DEFAULT_CLUSTER_GAP_M,
    DEFAULT_LIDAR_X,
    DEFAULT_LIDAR_Y,
    DEFAULT_MAX_TARGETS,
    DEFAULT_MIN_POINTS,
    pose_from_odometry,
    scan_to_0141_json,
)


# 与收发节点一起启动：雷达扫描 → /ladds/obstacle_in → DDS 0141。
class RadarAdapterNode(Node):
    """Convert ``sensor_msgs/LaserScan`` into protocol 0141 target lists."""

    def __init__(self) -> None:
        super().__init__("usv_ladds_radar")
        self.declare_parameter("scan_topic", "/otter/sensors/lidars/lidar/scan")
        self.declare_parameter("odom_topic", "/otter/odom_gt")
        self.declare_parameter("output_topic", "/ladds/obstacle_in")
        self.declare_parameter("origin_latitude", 30.5453)
        self.declare_parameter("origin_longitude", 114.392931)
        self.declare_parameter("origin_altitude", 0.0)
        self.declare_parameter("cluster_gap_m", DEFAULT_CLUSTER_GAP_M)
        self.declare_parameter("min_points", DEFAULT_MIN_POINTS)
        self.declare_parameter("max_targets", DEFAULT_MAX_TARGETS)
        self.declare_parameter("lidar_x", DEFAULT_LIDAR_X)
        self.declare_parameter("lidar_y", DEFAULT_LIDAR_Y)
        self.declare_parameter("queue_depth", 10)

        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if not scan_topic.strip() or not odom_topic.strip() or not output_topic.strip():
            raise ValueError("scan_topic, odom_topic and output_topic must not be empty")
        queue_depth = int(self.get_parameter("queue_depth").value)
        if queue_depth <= 0:
            raise ValueError("queue_depth must be greater than zero")

        self._origin = GeoReference(
            float(self.get_parameter("origin_latitude").value),
            float(self.get_parameter("origin_longitude").value),
            float(self.get_parameter("origin_altitude").value),
        )
        self._cluster_gap_m = float(self.get_parameter("cluster_gap_m").value)
        self._min_points = int(self.get_parameter("min_points").value)
        self._max_targets = int(self.get_parameter("max_targets").value)
        self._lidar_x = float(self.get_parameter("lidar_x").value)
        self._lidar_y = float(self.get_parameter("lidar_y").value)
        self._pose = None
        self._publisher = self.create_publisher(String, output_topic, queue_depth)
        self.create_subscription(Odometry, odom_topic, self._on_odom, queue_depth)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"radar 0141 adapter: {scan_topic} + {odom_topic} -> {output_topic}"
        )

    def _on_odom(self, message: Odometry) -> None:
        try:
            self._pose = pose_from_odometry(message)
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"drop invalid odometry for 0141 radar adapter: {exc}")

    def _on_scan(self, message: LaserScan) -> None:
        pose = self._pose
        if pose is None:
            return
        try:
            text = scan_to_0141_json(
                message,
                pose,
                self._origin,
                cluster_gap_m=self._cluster_gap_m,
                min_points=self._min_points,
                max_targets=self._max_targets,
                lidar_x=self._lidar_x,
                lidar_y=self._lidar_y,
            )
        except (AttributeError, TypeError, ValueError, ArithmeticError, OverflowError) as exc:
            self.get_logger().warning(f"drop invalid scan for 0141: {exc}")
            return
        output = String()
        output.data = text
        self._publisher.publish(output)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RadarAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
