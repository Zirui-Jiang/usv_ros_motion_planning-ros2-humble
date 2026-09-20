"""LaDDS 发布节点：把仿真 odom 编成 0411 发出，并发布内部 0411 JSON。"""

from __future__ import annotations

import json
from typing import Any, Optional

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8MultiArray

from .config import BridgeConfig, HeaderCache, validate_obstacle_payload
from .ownship import ownship_payload_from_odometry
from common.geodesy import GeoReference


class LaDDSOutboundNode(Node):
    """Encode ROS inputs to LaDDS 0411/0141/0052 without decoding odom."""

    def __init__(
        self,
        config: BridgeConfig,
        client: Any,
        header_cache: HeaderCache,
    ) -> None:
        super().__init__("usv_ladds_outbound", use_global_arguments=False)
        self._config = config
        self._client = client
        self._header_cache = header_cache
        self._send_failures = 0
        self._origin = GeoReference(
            config.origin_latitude,
            config.origin_longitude,
            config.origin_altitude,
        )
        self._ownship_json_publisher = None
        if config.ownship_json_topic:
            self._ownship_json_publisher = self.create_publisher(
                String, config.ownship_json_topic, config.queue_depth
            )
        self._obstacle_json_publisher = None
        if config.obstacle_json_topic:
            self._obstacle_json_publisher = self.create_publisher(
                String, config.obstacle_json_topic, config.queue_depth
            )
        self.create_subscription(
            String, "/ladds/tx", self._on_text_tx, config.queue_depth
        )
        self.create_subscription(
            UInt8MultiArray,
            "/ladds/tx_bytes",
            self._on_bytes_tx,
            config.queue_depth,
        )
        if config.ownship_input_topic:
            self.create_subscription(
                Odometry,
                config.ownship_input_topic,
                self._on_ownship,
                config.queue_depth,
            )
        if config.obstacle_input_topic:
            self.create_subscription(
                String,
                config.obstacle_input_topic,
                self._on_obstacle,
                config.queue_depth,
            )
        self.get_logger().info(
            "LaDDS outbound started "
            f"(0411 from {config.ownship_input_topic or 'disabled'}, "
            f"0141 from {config.obstacle_input_topic or 'disabled'})"
        )

    def _on_text_tx(self, message: String) -> None:
        self._send_payload(message.data.encode("utf-8"), source_topic="/ladds/tx")

    def _on_bytes_tx(self, message: UInt8MultiArray) -> None:
        self._send_payload(bytes(message.data), source_topic="/ladds/tx_bytes")

    def _on_ownship(self, message: Odometry) -> None:
        """仿真真值 odom 编成 0411 发到 LaDDS，并发布给接收节点订阅。"""
        try:
            payload = ownship_payload_from_odometry(message, self._origin)
            encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self._header_cache.store(
                message.header.stamp,
                message.header.frame_id,
                message.child_frame_id,
            )
            self._send_payload(
                encoded.encode("utf-8"),
                source_topic=self._config.ownship_input_topic,
                transport_topic=self._config.ownship_send_topic,
            )
            if self._ownship_json_publisher is not None:
                json_message = String()
                json_message.data = encoded
                self._ownship_json_publisher.publish(json_message)
        except (AttributeError, TypeError, ValueError, ArithmeticError) as exc:
            self.get_logger().warning(f"drop invalid odometry for LaDDS 0411: {exc}")

    def _on_obstacle(self, message: String) -> None:
        """校验 0141 JSON 后发到 LaDDS，并发布给接收节点订阅。"""
        try:
            validate_obstacle_payload(message.data)
            payload = message.data.encode("utf-8")
            self._send_payload(
                payload,
                source_topic=self._config.obstacle_input_topic,
                transport_topic=self._config.obstacle_send_topic,
            )
            if self._obstacle_json_publisher is not None:
                self._obstacle_json_publisher.publish(message)
        except (AttributeError, TypeError, ValueError, UnicodeError) as exc:
            self.get_logger().warning(
                f"drop invalid obstacle JSON for LaDDS {self._config.obstacle_send_topic}: {exc}"
            )

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
                f"(failure {self._send_failures}): {_short_error(exc)}"
            )


def _short_error(error: Exception, limit: int = 300) -> str:
    text = str(error).replace("\r", " ").replace("\n", " ")
    return text if len(text) <= limit else f"{text[:limit]}..."
