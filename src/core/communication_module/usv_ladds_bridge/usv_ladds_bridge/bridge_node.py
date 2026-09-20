"""ROS 2 node that bridges fixed local topics to one LaDDS transport client."""

from __future__ import annotations

import json
import sys
from dataclasses import asdict
from typing import Any, Callable, Dict, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8MultiArray

from .config import BridgeConfig, decode_utf8, drain_received, packet_info_json, receive_channel
from .ladds_client import LaDDSClient
from .ownship import ownship_payload_from_odometry
from common.geodesy import GeoReference


_PARAMETER_DEFAULTS: Dict[str, Any] = asdict(BridgeConfig())


class LaDDSBridgeNode(Node):
    """Translate ROS text/byte messages without changing their payload bytes."""

    def __init__(
        self,
        *,
        native_client_factory: Callable[..., Any] = LaDDSClient,
        tcp_client_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        super().__init__("usv_ladds_bridge")
        self._client: Optional[Any] = None
        self._send_failures = 0
        self._receive_failures = 0
        self._utf8_decode_failures = 0
        self._last_dropped_messages = 0
        self._last_dropped_warning = 0

        try:
            config = self._read_config()
            self._receive_topic = config.receive_topic
            self._stop_receive_topic = config.stop_receive_topic
            self._ownship_send_topic = config.ownship_send_topic
            self._ownship_input_topic = config.ownship_input_topic
            self._ownship_origin = GeoReference(
                config.origin_latitude,
                config.origin_longitude,
                config.origin_altitude,
            )
            if config.backend == "tcp":
                if tcp_client_factory is None:
                    from .tcp_client import TcpRelayClient

                    tcp_client_factory = TcpRelayClient
                self._client = tcp_client_factory(
                    host=config.relay_host,
                    port=config.relay_port,
                    timeout=config.relay_timeout,
                    max_payload_bytes=config.max_payload_bytes,
                    queue_depth=config.queue_depth,
                )
            else:
                self._client = native_client_factory(
                    library_path=config.library_path,
                    domain_id=config.domain_id,
                    bind_ip=config.bind_ip,
                    ttl=config.ttl,
                    qos=config.qos,
                    send_topic=config.send_topic,
                    ownship_send_topic=config.ownship_send_topic,
                    receive_topic=config.receive_topic,
                    stop_receive_topic=config.stop_receive_topic,
                    src=config.src,
                    dst=config.dst,
                    max_payload_bytes=config.max_payload_bytes,
                    queue_depth=config.queue_depth,
                    encryption=config.encryption,
                )
            self._client.start()

            self._text_rx_publisher = self.create_publisher(
                String, "/ladds/rx", config.queue_depth
            )
            self._stop_rx_publisher = self.create_publisher(
                String, '/ladds/stop_rx', config.queue_depth
            )
            self._bytes_rx_publisher = self.create_publisher(
                UInt8MultiArray, "/ladds/rx_bytes", config.queue_depth
            )
            self._info_rx_publisher = self.create_publisher(
                String, "/ladds/rx_info", config.queue_depth
            )
            self._text_tx_subscription = self.create_subscription(
                String, "/ladds/tx", self._on_text_tx, config.queue_depth
            )
            self._bytes_tx_subscription = self.create_subscription(
                UInt8MultiArray,
                "/ladds/tx_bytes",
                self._on_bytes_tx,
                config.queue_depth,
            )
            self._ownship_tx_subscription = None
            if config.ownship_input_topic:
                self._ownship_tx_subscription = self.create_subscription(
                    Odometry,
                    config.ownship_input_topic,
                    self._on_ownship,
                    config.queue_depth,
                )
            self._receive_limit = config.queue_depth
            self._poll_timer = self.create_timer(
                config.poll_interval_sec, self._poll_received
            )

            version = self._client.version or "unknown"
            if config.backend == "tcp":
                self.get_logger().info(
                    "LaDDS bridge started "
                    f"(TCP relay {config.relay_host}:{config.relay_port}, "
                    f"version {version}); DDS domain/topic/QoS are configured by the relay"
                )
            else:
                self.get_logger().info(
                    f"LaDDS bridge started (native SDK {version}, "
                    f"domain {config.domain_id}, topic {config.receive_topic}, "
                    f"QoS {config.qos})"
                )
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def _read_config(self) -> BridgeConfig:
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

    def _on_text_tx(self, message: String) -> None:
        self._send_payload(message.data.encode("utf-8"), source_topic="/ladds/tx")

    def _on_ownship(self, message: Odometry) -> None:
        """Convert the existing odometry stream into an outbound 0411 packet."""
        try:
            payload = ownship_payload_from_odometry(message, self._ownship_origin)
            encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self._send_payload(
                encoded.encode("utf-8"),
                source_topic=self._ownship_input_topic,
                transport_topic=self._ownship_send_topic,
            )
        except (AttributeError, TypeError, ValueError, ArithmeticError) as exc:
            self.get_logger().warning(f"drop invalid odometry for LaDDS 0411: {exc}")

    def _on_bytes_tx(self, message: UInt8MultiArray) -> None:
        self._send_payload(bytes(message.data), source_topic="/ladds/tx_bytes")

    def _send_payload(
        self,
        payload: bytes,
        *,
        source_topic: str,
        transport_topic: Optional[str] = None,
    ) -> None:
        client = self._client
        if client is None:
            return
        try:
            if transport_topic is None:
                result = client.send(payload)
            else:
                result = client.send(payload, topic=transport_topic)
            if result is False:
                raise RuntimeError("transport returned failure")
        except Exception as exc:
            self._send_failures += 1
            self.get_logger().error(
                f"LaDDS send failed from {source_topic} "
                f"(failure {self._send_failures}): {self._short_error(exc)}"
            )

    def _poll_received(self) -> None:
        client = self._client
        if client is None:
            return
        try:
            packets = drain_received(client, self._receive_limit)
        except Exception as exc:
            self._receive_failures += 1
            if self._should_log_count(self._receive_failures):
                self.get_logger().error(
                    "LaDDS receive failed "
                    f"(failure {self._receive_failures}): {self._short_error(exc)}"
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
                if self._should_log_count(self._utf8_decode_failures):
                    self.get_logger().warning(
                        "LaDDS packet is not valid UTF-8; published on /ladds/rx_bytes only "
                        f"(decode failure {self._utf8_decode_failures})"
                    )
                continue

            text_message = String()
            text_message.data = decoded
            channel = receive_channel(packet.topic, self._receive_topic, self._stop_receive_topic)
            if channel == 'stop':
                self._stop_rx_publisher.publish(text_message)
            elif channel == 'route':
                self._text_rx_publisher.publish(text_message)
            else:
                self.get_logger().warning(f'Ignoring unconfigured DDS text topic: {packet.topic!r}')

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

    @staticmethod
    def _should_log_count(count: int) -> bool:
        return count == 1 or count % 100 == 0

    @staticmethod
    def _short_error(error: Exception, limit: int = 300) -> str:
        text = str(error).replace("\r", " ").replace("\n", " ")
        return text if len(text) <= limit else f"{text[:limit]}..."

    def close(self) -> None:
        """Release the native transport handle; safe to call more than once."""
        client, self._client = self._client, None
        if client is None:
            return
        try:
            client.close()
        except Exception as exc:
            self.get_logger().error(
                f"LaDDS close failed: {self._short_error(exc)}"
            )


def main(args: Optional[list[str]] = None) -> None:
    """Initialize ROS, run the bridge, and release both ROS and LaDDS resources."""
    initialized = False
    node: Optional[LaDDSBridgeNode] = None
    try:
        rclpy.init(args=args)
        initialized = True
        node = LaDDSBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"LaDDS bridge stopped: {node._short_error(exc)}")
        else:
            print(f"LaDDS bridge failed to start: {exc}", file=sys.stderr)
        raise
    finally:
        try:
            if node is not None:
                node.close()
                node.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()
