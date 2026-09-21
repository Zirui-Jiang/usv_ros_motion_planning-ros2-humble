"""Publish direct LaserScan observations as temporary 0141 target JSON.

The navigation costmap consumes the same scan independently.  This node does
not subscribe to a costmap: it looks up the sensor pose in TF, converts the
valid polar beams into geometric clusters, and publishes the resulting JSON
on the input topic used by the LaDDS bridge.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from common.geodesy import GeoReference

from .scan_0141 import (
    DEFAULT_MAX_TARGETS,
    DEFAULT_THREAT_RADIUS,
    scan_to_0141_json,
)


class Scan0141Node(Node):
    """Convert the simulator's ``sensor_msgs/LaserScan`` into 0141 JSON."""

    def __init__(self) -> None:
        super().__init__("usv_ladds_scan")

        self.declare_parameter("input_topic", "/otter/sensors/lidars/lidar/scan")
        self.declare_parameter("output_topic", "/ladds/obstacle_in")
        self.declare_parameter("target_frame", "otter/map")
        self.declare_parameter("origin_latitude", 30.5453)
        self.declare_parameter("origin_longitude", 114.392931)
        self.declare_parameter("origin_altitude", 0.0)
        self.declare_parameter("cluster_range_jump", 1.0)
        self.declare_parameter("threat_radius", DEFAULT_THREAT_RADIUS)
        self.declare_parameter("max_targets", DEFAULT_MAX_TARGETS)
        self.declare_parameter("tf_timeout", 0.1)
        self.declare_parameter("queue_depth", 10)
        # rclpy declares ``use_sim_time`` on every node; launch can override it
        # without redeclaring the parameter here.

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        target_frame = str(self.get_parameter("target_frame").value).strip().strip("/")
        queue_depth = int(self.get_parameter("queue_depth").value)
        tf_timeout = float(self.get_parameter("tf_timeout").value)
        cluster_range_jump = float(self.get_parameter("cluster_range_jump").value)
        threat_radius = float(self.get_parameter("threat_radius").value)
        max_targets = int(self.get_parameter("max_targets").value)
        if not input_topic or not output_topic:
            raise ValueError("input_topic and output_topic must not be empty")
        if not target_frame:
            raise ValueError("target_frame must not be empty")
        if queue_depth <= 0:
            raise ValueError("queue_depth must be greater than zero")
        if not math.isfinite(tf_timeout) or tf_timeout < 0.0:
            raise ValueError("tf_timeout must be finite and non-negative")
        if not math.isfinite(cluster_range_jump) or cluster_range_jump <= 0.0:
            raise ValueError("cluster_range_jump must be finite and greater than zero")
        if not math.isfinite(threat_radius) or threat_radius < 0.0:
            raise ValueError("threat_radius must be finite and non-negative")
        if not 1 <= max_targets <= 255:
            raise ValueError("max_targets must be in [1, 255]")

        self._target_frame = target_frame
        self._origin = GeoReference(
            float(self.get_parameter("origin_latitude").value),
            float(self.get_parameter("origin_longitude").value),
            float(self.get_parameter("origin_altitude").value),
        )
        self._cluster_range_jump = cluster_range_jump
        self._threat_radius = threat_radius
        self._max_targets = max_targets
        self._tf_timeout = tf_timeout
        self._sequence = 1
        self._last_tf_warning = 0.0

        self._publisher = self.create_publisher(String, output_topic, queue_depth)
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, scan_qos
        )
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.get_logger().info(
            f"direct LaserScan 0141 adapter: {input_topic} -> {output_topic}; "
            f"TF {self._target_frame}, cluster jump={cluster_range_jump:g}m"
        )

    def _on_scan(self, message: LaserScan) -> None:
        frame_id = str(message.header.frame_id).strip().strip("/")
        if not frame_id:
            self.get_logger().warning("discarding LaserScan with an empty frame_id")
            return
        try:
            stamp = Time.from_msg(message.header.stamp)
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                frame_id,
                stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = _quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
            text = scan_to_0141_json(
                message.ranges,
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
                vehicle_x=float(translation.x),
                vehicle_y=float(translation.y),
                vehicle_yaw=yaw,
                origin=self._origin,
                sequence=self._sequence,
                cluster_range_jump=self._cluster_range_jump,
                threat_radius=self._threat_radius,
                max_targets=self._max_targets,
            )
        except TransformException as exc:
            self._warn_tf_once(str(exc))
            return
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(f"failed to serialize LaserScan for 0141: {exc}")
            return

        output = String()
        output.data = text
        self._publisher.publish(output)
        self._sequence = (self._sequence + 1) % 65536

    def _warn_tf_once(self, detail: str) -> None:
        now = time.monotonic()
        if now - self._last_tf_warning < 5.0:
            return
        self._last_tf_warning = now
        self.get_logger().warning(
            f"waiting for TF {self._target_frame} <- LaserScan frame: {detail}"
        )


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw after validating and normalizing a TF quaternion."""

    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("TF quaternion must contain finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("TF quaternion norm must be non-zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Scan0141Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
