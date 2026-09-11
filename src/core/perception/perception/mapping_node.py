"""ROS 2 adapter for the integer-evidence mapping kernel."""

from array import array
from collections import deque
import math
import time
import json
import os
import resource

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .mapping import GridConfig, GridMapper
from .scan_queue import ClockGuard, ScanQueue, ScanResult


class MappingNode(Node):
    def __init__(self):
        super().__init__("perception_mapping")
        descriptor = ParameterDescriptor(read_only=True)

        def startup(name, default):
            return self.declare_parameter(name, default, descriptor).value

        config = GridConfig(width=startup("width", 1024),
                            resolution=startup("resolution", 0.5),
                            offset_x=startup("offset_x", 0.0),
                            offset_y=startup("offset_y", 0.0))
        publish_frequency = startup("map_publish_frequency", 2.0)
        tf_wait_timeout = startup("tf_wait_timeout", 0.2)
        if not math.isfinite(publish_frequency) or publish_frequency <= 0:
            raise ValueError("map_publish_frequency must be finite and positive")
        self.mapper = GridMapper(config)
        self.pending = ScanQueue(capacity=5, tf_wait_timeout=tf_wait_timeout)
        self.clock_guard = ClockGuard()
        namespace = self.get_namespace().strip("/")
        self.map_frame = namespace + "/map" if namespace else "map"
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 10)
        self.status_pub = self.create_publisher(String, "mapping_status", 10)
        # A best-effort subscription accepts both best-effort and reliable
        # publishers, while preserving the original depth and volatile map QoS.
        scan_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self._on_scan, scan_qos)
        self.message = OccupancyGrid()
        self.message.header.frame_id = self.map_frame
        self.message.info.width = self.message.info.height = config.width
        self.message.info.resolution = float(config.resolution)
        self.message.info.origin.position.x = self.mapper.origin_x
        self.message.info.origin.position.y = self.mapper.origin_y
        self.message.info.origin.orientation.w = 1.0
        self.message.info.map_load_time = self.get_clock().now().to_msg()
        self.processing_ms = deque(maxlen=256)
        self.waiting_ms = deque(maxlen=256)
        self.serialization_ms = deque(maxlen=256)
        self.scan_period_ms = deque(maxlen=256)
        self.last_scan_stamp = None
        self.scan_overruns = 0
        self.integrated_scans = 0
        self.invalid_scans = 0
        self.invalid_beams = 0
        # Wall-time queue polling expires scans even while simulation is paused.
        # No TF lookup blocks this executor; the listener can receive new TF
        # messages between callbacks before the scan's waiting deadline.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.queue_timer = self.create_timer(0.01, self._drain, clock=self.steady_clock)
        self.publish_timer = self.create_timer(1.0 / publish_frequency, self._publish_map)
        self.metrics_timer = self.create_timer(5.0, self._log_metrics, clock=self.steady_clock)
        self._observe_clock()
        self.get_logger().info(
            f"LiDAR mapping frame={self.map_frame}, grid={config.width}x{config.width}, "
            f"resolution={config.resolution}m, publication={publish_frequency}Hz, "
            f"TF deadline={tf_wait_timeout}s, backend={self.mapper.backend}")

    def _observe_clock(self):
        now = self.get_clock().now()
        if self.clock_guard.observe(now.nanoseconds):
            self.pending.reset()
            self.mapper.reset()
            self.tf_buffer.clear()
            self.processing_ms.clear()
            self.waiting_ms.clear()
            self.serialization_ms.clear()
            self.scan_period_ms.clear()
            self.last_scan_stamp = None
            self.message.info.map_load_time = now.to_msg()
            self.get_logger().warning("ROS clock moved backwards; map, scan queue and TF history reset")
        return now

    def _on_scan(self, message):
        self._observe_clock()
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.pending.enqueue(message, frame_id=message.header.frame_id.lstrip("/"),
                                stamp_ns=stamp_ns, received_at=time.monotonic()):
            if self.last_scan_stamp is not None and stamp_ns > self.last_scan_stamp:
                self.scan_period_ms.append((stamp_ns - self.last_scan_stamp) * 1e-6)
            self.last_scan_stamp = stamp_ns

    def _drain(self):
        self._observe_clock()
        self.pending.drain(time.monotonic, self._try_process)
        # Expiry and the diagnostic snapshot describe the same real checkpoint.
        # Keep observe_pending read-only; do not clamp or backdate pending ages.
        checkpoint = time.monotonic()
        self.pending.expire_pending(checkpoint)
        self.pending.observe_pending(checkpoint)

    def _try_process(self, pending):
        scan = pending.payload
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, pending.frame_id,
                Time.from_msg(scan.header.stamp, clock_type=ClockType.ROS_TIME),
                timeout=Duration(seconds=0.0))
        except TransformException:
            return ScanResult.WAITING
        start = time.monotonic()
        # The exact-time TF lookup can return after its waiting deadline even
        # with timeout=0. Reject before any map mutation, using this admission time.
        if start - pending.received_at >= self.pending.tf_wait_timeout:
            return ScanResult.EXPIRED
        try:
            translation = transform.transform.translation
            q = transform.transform.rotation
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError("TF rotation is invalid")
            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            stats = self.mapper.integrate_scan(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, translation.x, translation.y, yaw)
        except (ValueError, TypeError, OverflowError):
            self.invalid_scans += 1
            return ScanResult.INVALID  # Consume once without counting an integration.
        self.processing_ms.append((time.monotonic() - start) * 1000.0)
        budget_ms = float(scan.scan_time) * 1000 if scan.scan_time > 0 else (
            float(np.median(self.scan_period_ms)) if self.scan_period_ms else None)
        if budget_ms is not None and self.processing_ms[-1] > budget_ms:
            self.scan_overruns += 1
        self.waiting_ms.append((start - pending.received_at) * 1000.0)
        self.integrated_scans += 1
        self.invalid_beams += stats.invalid_beams
        return ScanResult.PROCESSED

    def _publish_map(self):
        start = time.monotonic()
        self.message.header.stamp = self._observe_clock().to_msg()
        # Signed-byte serialization avoids allocating a million Python ints.
        self.message.data = array("b", self.mapper.data.tobytes(order="C"))
        self.map_pub.publish(self.message)
        self.serialization_ms.append((time.monotonic() - start) * 1000)

    def _log_metrics(self):
        counts = self.pending.counters
        p50, p95 = (np.percentile(self.processing_ms, [50, 95])
                    if self.processing_ms else (0.0, 0.0))
        waiting = float(np.percentile(self.waiting_ms, 95)) if self.waiting_ms else 0.0
        self.get_logger().info(
            f"mapping queue={len(self.pending)} integrated={self.integrated_scans} "
            f"drops(tf={counts.expired}, overflow={counts.overflow}, "
            f"duplicate_or_old={counts.duplicate}, invalid={counts.invalid + self.invalid_scans}) "
            f"invalid_beams={self.invalid_beams} resets={counts.resets} "
            f"processing_ms(p50={p50:.2f}, p95={p95:.2f}) queue_ms_p95={waiting:.2f}")
        def summary(samples):
            return {'count': len(samples), **({
                'p50': float(np.percentile(samples, 50)), 'p95': float(np.percentile(samples, 95)),
                'p99': float(np.percentile(samples, 99)), 'max': max(samples)} if samples else {})}
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self.status_pub.publish(String(data=json.dumps({
            'stamp': self.get_clock().now().nanoseconds * 1e-9, 'queue_size': len(self.pending),
            'integrated_scans': self.integrated_scans, 'scan_overruns': self.scan_overruns,
            'queue_counters': vars(counts), 'invalid_scans': self.invalid_scans,
            'queue_observation_window': self.pending.observation_summary(time.monotonic()),
            'invalid_beams': self.invalid_beams,
            'processing_ms': summary(self.processing_ms), 'queue_ms': summary(self.waiting_ms),
            'serialization_and_publish_ms': summary(self.serialization_ms),
            'observed_scan_period_ms': summary(self.scan_period_ms),
            'process': {'pid': os.getpid(), 'cpu_seconds': usage.ru_utime + usage.ru_stime,
                        'peak_rss_kib': usage.ru_maxrss, 'monotonic': time.monotonic()}})))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MappingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.pending.reset()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
