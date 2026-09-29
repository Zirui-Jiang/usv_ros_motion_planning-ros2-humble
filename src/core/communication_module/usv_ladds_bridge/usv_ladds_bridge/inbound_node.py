"""LaDDS 接收节点 usv_ladds_rx：订 0211/0331/0212 任务，0411 解成内部 odom，0141 画成障碍栅格。"""

from __future__ import annotations

from dataclasses import asdict
import json
import time
from typing import Any, Callable, Dict, Optional, Sequence

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, UInt8MultiArray

from .config import (
    BridgeConfig,
    HeaderCache,
    decode_utf8,
    drain_received,
    packet_info_json,
    receive_channel,
)
from .ladds_client import LaDDSClient
from .obstacle_grid import ObstacleDisc, parse_obstacle_message, rasterize, target_discs
from .ownship import odometry_from_ownship_payload, parse_ownship_payload
from common.geodesy import GeoReference


_PARAMETER_DEFAULTS: Dict[str, Any] = asdict(BridgeConfig())


class LaDDSInboundNode(Node):
    """Subscribe to inbound LaDDS topics and republish ROS messages for the stack."""

    def __init__(
        self,
        *,
        native_client_factory: Callable[..., Any] = LaDDSClient,
        tcp_client_factory: Optional[Callable[..., Any]] = None,
        header_cache: Optional[HeaderCache] = None,
    ) -> None:
        super().__init__("usv_ladds_rx")
        self.client: Optional[Any] = None
        self.header_cache = header_cache or HeaderCache()
        self._receive_failures = 0
        self._utf8_decode_failures = 0
        self._last_dropped_messages = 0
        self._last_dropped_warning = 0
        self._ownship_decode_failures = 0
        self._obstacle_decode_failures = 0
        self._obstacle_target_warnings = 0
        # 最新 0411 船位（地图坐标），作为障碍栅格窗口的中心。
        self._ownship_xy: Optional[tuple[float, float]] = None
        self._last_obstacle_at: Optional[float] = None
        self._grid_publisher = None

        try:
            self.bridge_config = self._read_config()
            config = self.bridge_config
            self._origin = GeoReference(
                config.origin_latitude,
                config.origin_longitude,
                config.origin_altitude,
            )
            self.client = _create_client(
                config,
                native_client_factory=native_client_factory,
                tcp_client_factory=tcp_client_factory,
            )
            self.client.start()
            self._setup_io(config)
            self._log_start(config)
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def _read_config(self) -> BridgeConfig:
        from rcl_interfaces.msg import ParameterDescriptor

        values: Dict[str, Any] = {}
        for name, default in _PARAMETER_DEFAULTS.items():
            descriptor = ParameterDescriptor(
                description="Static LaDDS bridge setting; restart the node to change it.",
                read_only=True,
            )
            values[name] = self.declare_parameter(
                name, default, descriptor=descriptor
            ).value
        return BridgeConfig.from_mapping(values)

    def _setup_io(self, config: BridgeConfig) -> None:
        self._text_rx_publisher = self.create_publisher(
            String, "/ladds/rx", config.queue_depth
        )
        self._stop_rx_publisher = self.create_publisher(
            String, "/ladds/stop_rx", config.queue_depth
        )
        self._berth_rx_publisher = self.create_publisher(
            String, "/ladds/berth_rx", config.queue_depth
        )
        self._bytes_rx_publisher = self.create_publisher(
            UInt8MultiArray, "/ladds/rx_bytes", config.queue_depth
        )
        self._info_rx_publisher = self.create_publisher(
            String, "/ladds/rx_info", config.queue_depth
        )
        odom_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_publisher = None
        if config.ownship_output_topic:
            self._odom_publisher = self.create_publisher(
                Odometry, config.ownship_output_topic, odom_qos
            )
        self._targets_publisher = None
        if config.obstacle_output_topic:
            self._targets_publisher = self.create_publisher(
                String, config.obstacle_output_topic, config.queue_depth
            )
        if config.obstacle_grid_topic:
            # 与 costmap 静态层的 map_subscribe_transient_local 对应，后启动的订阅者也能拿到最新一张。
            grid_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._grid_publisher = self.create_publisher(
                OccupancyGrid, config.obstacle_grid_topic, grid_qos
            )
            # Give the static layer a valid empty map before the first 0141.
            self._publish_obstacle_grid(())
            if config.obstacle_timeout_sec > 0.0:
                self.create_timer(
                    min(0.5, config.obstacle_timeout_sec / 2.0),
                    self._expire_obstacles,
                    clock=Clock(clock_type=ClockType.STEADY_TIME),
                )
            self.get_logger().info(
                f"0141 obstacle grid: {config.obstacle_grid_topic} in "
                f"{config.obstacle_grid_frame}, resolution {config.obstacle_grid_resolution:g} m, "
                f"default radius {config.obstacle_default_radius:g} m, "
                f"threatingRadius scale {config.obstacle_radius_scale:g}, "
                + (
                    f"cleared after {config.obstacle_timeout_sec:g} s without 0141"
                    if config.obstacle_timeout_sec > 0.0
                    else "kept until the next 0141"
                )
            )
        if config.ownship_json_topic:
            self.create_subscription(
                String,
                config.ownship_json_topic,
                self._on_ownship_json,
                config.queue_depth,
            )
        if config.obstacle_json_topic:
            self.create_subscription(
                String,
                config.obstacle_json_topic,
                self._on_obstacle_json,
                config.queue_depth,
            )
        self._receive_limit = config.queue_depth
        self._poll_timer = self.create_timer(
            config.poll_interval_sec, self._poll_received
        )

    def _log_start(self, config: BridgeConfig) -> None:
        version = self.client.version if self.client is not None else "unknown"
        version = version or "unknown"
        if config.backend == "tcp":
            self.get_logger().info(
                "LaDDS bridge started "
                f"(rx; TCP relay {config.relay_host}:{config.relay_port}, "
                f"version {version}); subscribe 0211/0331/0212/0411/0141"
            )
        else:
            self.get_logger().info(
                f"LaDDS bridge started (rx; native SDK {version}, "
                f"domain {config.domain_id}, receive {config.receive_topic}, "
                f"berth {config.berth_receive_topic or '-'}, "
                f"ownship {config.ownship_receive_topic}, QoS {config.qos})"
            )

    def _on_ownship_json(self, message: String) -> None:
        """接收节点订 0411 JSON 后解成决策/航控使用的 odom。"""
        self._publish_odometry(message.data, use_cached_header=True)

    def _on_obstacle_json(self, message: String) -> None:
        self._publish_targets(message.data)

    def _publish_odometry(self, text: str, *, use_cached_header: bool) -> None:
        publisher = self._odom_publisher
        if publisher is None:
            return
        try:
            payload = parse_ownship_payload(text)
            state = odometry_from_ownship_payload(payload, self._origin)
        except (AttributeError, TypeError, ValueError, ArithmeticError, KeyError) as exc:
            self._ownship_decode_failures += 1
            if _should_log_count(self._ownship_decode_failures):
                self.get_logger().warning(
                    f"drop invalid 0411 for odometry: {exc}"
                )
            return
        self._ownship_xy = (state.x, state.y)
        message = Odometry()
        cached = self.header_cache.load() if use_cached_header else None
        if cached is not None:
            message.header.stamp, message.header.frame_id, message.child_frame_id = cached
        else:
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.bridge_config.ownship_frame_id
            message.child_frame_id = self.bridge_config.ownship_child_frame_id
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.orientation.x = state.qx
        message.pose.pose.orientation.y = state.qy
        message.pose.pose.orientation.z = state.qz
        message.pose.pose.orientation.w = state.qw
        message.twist.twist.linear.x = state.speed_mps
        message.twist.twist.linear.y = 0.0
        message.twist.twist.angular.z = state.angular_z
        publisher.publish(message)

    # 0141 剥外壳后转发业务对象，并按解 0411 的同一个原点画成障碍栅格；坏包不改动已发布的栅格。
    def _publish_targets(self, text: str) -> None:
        if self._targets_publisher is None and self._grid_publisher is None:
            return
        try:
            content, wrapped = parse_obstacle_message(text)
        except (AttributeError, TypeError, ValueError, UnicodeError) as exc:
            self._obstacle_decode_failures += 1
            if _should_log_count(self._obstacle_decode_failures):
                self.get_logger().warning(f"drop invalid 0141 JSON: {exc}")
            return
        if self._targets_publisher is not None:
            message = String()
            # Bare input is forwarded unchanged; wrapped input exposes only content.
            message.data = (
                json.dumps(content, ensure_ascii=False, separators=(",", ":"))
                if wrapped
                else text
            )
            self._targets_publisher.publish(message)
        if self._grid_publisher is None:
            return
        config = self.bridge_config
        discs, skipped = target_discs(
            content,
            self._origin,
            default_radius=config.obstacle_default_radius,
            radius_scale=config.obstacle_radius_scale,
        )
        dropped = self._publish_obstacle_grid(discs)
        self._last_obstacle_at = time.monotonic()
        if skipped or dropped:
            self._obstacle_target_warnings += 1
            if _should_log_count(self._obstacle_target_warnings):
                self.get_logger().warning(
                    f"0141 targets not drawn: {skipped} without valid latitude/longitude, "
                    f"{dropped} outside the {config.obstacle_grid_max_size:g} m grid window"
                )

    # 生成一张完整的障碍栅格并替换上一张；返回因超出窗口没有画出的目标数。
    def _publish_obstacle_grid(self, discs: Sequence[ObstacleDisc]) -> int:
        config = self.bridge_config
        grid = rasterize(
            discs,
            resolution=config.obstacle_grid_resolution,
            max_size=config.obstacle_grid_max_size,
            anchor=self._ownship_xy,
        )
        message = OccupancyGrid()
        stamp = self.get_clock().now().to_msg()
        message.header.stamp = stamp
        message.header.frame_id = config.obstacle_grid_frame
        message.info.map_load_time = stamp
        message.info.resolution = float(grid.resolution)
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = float(grid.origin_x)
        message.info.origin.position.y = float(grid.origin_y)
        message.info.origin.orientation.w = 1.0
        message.data = grid.data
        self._grid_publisher.publish(message)
        return grid.dropped

    # 超时仍没有新的 0141 时发一张空栅格；只清一次，等下一包到来再重新计时。
    def _expire_obstacles(self) -> None:
        last = self._last_obstacle_at
        timeout = self.bridge_config.obstacle_timeout_sec
        if last is None or time.monotonic() - last <= timeout:
            return
        self._last_obstacle_at = None
        self._publish_obstacle_grid(())
        self.get_logger().warning(f"no 0141 for {timeout:g} s; obstacle grid cleared")

    def _poll_received(self) -> None:
        client = self.client
        if client is None:
            return
        config = self.bridge_config
        try:
            packets = drain_received(client, self._receive_limit)
        except Exception as exc:
            self._receive_failures += 1
            if _should_log_count(self._receive_failures):
                self.get_logger().error(
                    "LaDDS receive failed "
                    f"(failure {self._receive_failures}): {_short_error(exc)}"
                )
            return
        finally:
            self._report_dropped_messages(client)

        for packet in packets:
            bytes_message = UInt8MultiArray()
            bytes_message.data = list(packet.payload)
            self._bytes_rx_publisher.publish(bytes_message)

            info_message = String()
            info_message.data = packet_info_json(packet)
            self._info_rx_publisher.publish(info_message)

            decoded = decode_utf8(packet.payload)
            if decoded is None:
                self._utf8_decode_failures += 1
                if _should_log_count(self._utf8_decode_failures):
                    self.get_logger().warning(
                        "LaDDS packet is not valid UTF-8; published on /ladds/rx_bytes only "
                        f"(decode failure {self._utf8_decode_failures})"
                    )
                continue

            text_message = String()
            text_message.data = decoded
            channel = receive_channel(
                packet.topic,
                config.receive_topic,
                config.stop_receive_topic,
                config.ownship_receive_topic,
                config.obstacle_receive_topic,
                config.berth_receive_topic,
            )
            if channel == "stop":
                self._stop_rx_publisher.publish(text_message)
            elif channel == "berth":
                self._berth_rx_publisher.publish(text_message)
            elif channel == "route":
                self._text_rx_publisher.publish(text_message)
            elif channel == "ownship":
                self._publish_odometry(decoded, use_cached_header=False)
            elif channel == "obstacle":
                self._publish_targets(decoded)
            else:
                self.get_logger().warning(
                    f"Ignoring unconfigured DDS text topic: {packet.topic!r}"
                )

    def _report_dropped_messages(self, client: Any) -> None:
        try:
            dropped = int(client.dropped_messages)
        except Exception:
            return
        if dropped <= self._last_dropped_messages:
            return

        self._last_dropped_messages = dropped
        if self._last_dropped_warning == 0 or dropped - self._last_dropped_warning >= 100:
            increase = dropped - self._last_dropped_warning
            self.get_logger().warning(
                f"LaDDS receive queue dropped {increase} additional message(s); "
                f"total dropped: {dropped}"
            )
            self._last_dropped_warning = dropped

    def close(self) -> None:
        """Release the native transport handle; safe to call more than once."""
        client, self.client = self.client, None
        if client is None:
            return
        try:
            client.close()
        except Exception as exc:
            self.get_logger().error(f"LaDDS close failed: {_short_error(exc)}")


def _create_client(
    config: BridgeConfig,
    *,
    native_client_factory: Callable[..., Any],
    tcp_client_factory: Optional[Callable[..., Any]],
) -> Any:
    if config.backend == "tcp":
        if tcp_client_factory is None:
            from .tcp_client import TcpRelayClient

            tcp_client_factory = TcpRelayClient
        return tcp_client_factory(
            host=config.relay_host,
            port=config.relay_port,
            timeout=config.relay_timeout,
            max_payload_bytes=config.max_payload_bytes,
            queue_depth=config.queue_depth,
        )
    return native_client_factory(
        library_path=config.library_path,
        domain_id=config.domain_id,
        bind_ip=config.bind_ip,
        ttl=config.ttl,
        qos=config.qos,
        send_topic=config.send_topic,
        ownship_send_topic=config.ownship_send_topic,
        obstacle_send_topic=config.obstacle_send_topic,
        status_send_topic=config.status_send_topic,
        progress_send_topic=config.progress_send_topic,
        thrust_send_topic=config.thrust_send_topic,
        receive_topic=config.receive_topic,
        stop_receive_topic=config.stop_receive_topic,
        berth_receive_topic=config.berth_receive_topic,
        ownship_receive_topic=config.ownship_receive_topic,
        obstacle_receive_topic=config.obstacle_receive_topic,
        src=config.src,
        dst=config.dst,
        max_payload_bytes=config.max_payload_bytes,
        queue_depth=config.queue_depth,
        encryption=config.encryption,
    )


def _should_log_count(count: int) -> bool:
    return count == 1 or count % 100 == 0


def _short_error(error: Exception, limit: int = 300) -> str:
    text = str(error).replace("\r", " ").replace("\n", " ")
    return text if len(text) <= limit else f"{text[:limit]}..."
