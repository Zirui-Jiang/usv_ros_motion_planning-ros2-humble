"""ROS-independent bridge configuration and payload conversion helpers."""

from dataclasses import dataclass
import json
import os
import threading
from typing import Any, Dict, List, Mapping, Optional, Tuple


@dataclass(frozen=True)
class BridgeConfig:
    """Validated, immutable settings used to construct a LaDDS client."""

    backend: str = "tcp"
    library_path: str = ""
    relay_host: str = "127.0.0.1"
    relay_port: int = 17880
    relay_timeout: float = 5.0
    domain_id: int = 1
    bind_ip: str = ""
    ttl: int = 1
    qos: str = "best_effort"
    send_topic: str = "0211"
    ownship_send_topic: str = "0411"
    obstacle_send_topic: str = "0141"
    receive_topic: str = "0211"
    stop_receive_topic: str = ""
    berth_receive_topic: str = ""
    ownship_receive_topic: str = "0411"
    obstacle_receive_topic: str = "0141"
    ownship_input_topic: str = ""
    obstacle_input_topic: str = "/ladds/obstacle_in"
    ownship_output_topic: str = "/otter/odom"
    obstacle_output_topic: str = "/ladds/targets"
    ownship_json_topic: str = "/ladds/ownship"
    obstacle_json_topic: str = "/ladds/obstacle"
    ownship_frame_id: str = "otter/odom"
    ownship_child_frame_id: str = "otter/base_link"
    origin_latitude: float = 30.5453
    origin_longitude: float = 114.392931
    origin_altitude: float = 0.0
    src: int = 0
    dst: int = 0
    encryption: bool = False
    max_payload_bytes: int = 1_048_576
    queue_depth: int = 100
    poll_interval_sec: float = 0.01
    status_send_topic: str = "0051"
    heartbeat_input_topic: str = "/otter/navigation/heartbeat"
    progress_send_topic: str = "0053"

    @classmethod
    def from_mapping(
        cls,
        values: Mapping[str, Any],
        *,
        environ: Optional[Mapping[str, str]] = None,
    ) -> "BridgeConfig":
        """Build settings from ROS-like values, resolving the SDK path from the environment."""
        environment = os.environ if environ is None else environ
        backend = str(values.get("backend", "tcp")).strip().lower()
        library_path = str(values.get("library_path", "")).strip()
        if backend == "native" and not library_path:
            library_path = environment.get("LADDS_LIBRARY_PATH", "").strip()
        if backend == "native" and not library_path:
            raise ValueError(
                "LaDDS SDK library is not configured: set the 'library_path' "
                "parameter or the LADDS_LIBRARY_PATH environment variable"
            )

        config = cls(
            backend=backend,
            library_path=library_path,
            relay_host=str(values.get("relay_host", "127.0.0.1")),
            relay_port=int(values.get("relay_port", 17880)),
            relay_timeout=float(values.get("relay_timeout", 5.0)),
            domain_id=int(values.get("domain_id", 1)),
            bind_ip=str(values.get("bind_ip", "")),
            ttl=int(values.get("ttl", 1)),
            qos=str(values.get("qos", "best_effort")),
            send_topic=str(values.get("send_topic", "0211")),
            ownship_send_topic=str(values.get("ownship_send_topic", "0411")),
            obstacle_send_topic=str(values.get("obstacle_send_topic", "0141")),
            receive_topic=str(values.get("receive_topic", "0211")),
            stop_receive_topic=str(values.get("stop_receive_topic", "")),
            berth_receive_topic=str(values.get("berth_receive_topic", "")),
            ownship_receive_topic=str(values.get("ownship_receive_topic", "0411")),
            obstacle_receive_topic=str(values.get("obstacle_receive_topic", "0141")),
            ownship_input_topic=str(values.get("ownship_input_topic", "")),
            obstacle_input_topic=str(
                values.get("obstacle_input_topic", "/ladds/obstacle_in")
            ),
            ownship_output_topic=str(values.get("ownship_output_topic", "/otter/odom")),
            obstacle_output_topic=str(
                values.get("obstacle_output_topic", "/ladds/targets")
            ),
            ownship_json_topic=str(values.get("ownship_json_topic", "/ladds/ownship")),
            obstacle_json_topic=str(values.get("obstacle_json_topic", "/ladds/obstacle")),
            ownship_frame_id=str(values.get("ownship_frame_id", "otter/odom")),
            ownship_child_frame_id=str(
                values.get("ownship_child_frame_id", "otter/base_link")
            ),
            origin_latitude=float(values.get("origin_latitude", 30.5453)),
            origin_longitude=float(values.get("origin_longitude", 114.392931)),
            origin_altitude=float(values.get("origin_altitude", 0.0)),
            src=int(values.get("src", 0)),
            dst=int(values.get("dst", 0)),
            encryption=bool(values.get("encryption", False)),
            max_payload_bytes=int(values.get("max_payload_bytes", 1_048_576)),
            queue_depth=int(values.get("queue_depth", 100)),
            poll_interval_sec=float(values.get("poll_interval_sec", 0.01)),
            status_send_topic=str(values.get("status_send_topic", "0051")),
            heartbeat_input_topic=str(
                values.get("heartbeat_input_topic", "/otter/navigation/heartbeat")
            ),
            progress_send_topic=str(values.get("progress_send_topic", "0053")),
        )
        config._validate()
        return config

    def _validate(self) -> None:
        if self.backend not in {"native", "tcp"}:
            raise ValueError("backend must be 'native' or 'tcp'")
        if not self.relay_host.strip():
            raise ValueError("relay_host must not be empty")
        if not 1 <= self.relay_port <= 65535:
            raise ValueError("relay_port must be between 1 and 65535")
        if self.relay_timeout <= 0.0:
            raise ValueError("relay_timeout must be greater than zero")
        if self.domain_id < 0:
            raise ValueError("domain_id must be non-negative")
        if not 0 <= self.ttl <= 255:
            raise ValueError("ttl must be between 0 and 255")
        if not self.qos.strip():
            raise ValueError("qos must not be empty")
        if not self.send_topic.strip():
            raise ValueError("send_topic must not be empty")
        if not self.ownship_send_topic.strip():
            raise ValueError("ownship_send_topic must not be empty")
        if not self.obstacle_send_topic.strip():
            raise ValueError("obstacle_send_topic must not be empty")
        if not self.receive_topic.strip():
            raise ValueError("receive_topic must not be empty")
        if self.stop_receive_topic and self.stop_receive_topic == self.receive_topic:
            raise ValueError('stop_receive_topic must be different from receive_topic')
        if self.stop_receive_topic and not self.stop_receive_topic.strip():
            raise ValueError('stop_receive_topic must be empty or a non-blank topic')
        if self.berth_receive_topic and not self.berth_receive_topic.strip():
            raise ValueError('berth_receive_topic must be empty or a non-blank topic')
        if self.berth_receive_topic and self.berth_receive_topic == self.receive_topic:
            raise ValueError('berth_receive_topic must be different from receive_topic')
        if (
            self.berth_receive_topic
            and self.stop_receive_topic
            and self.berth_receive_topic == self.stop_receive_topic
        ):
            raise ValueError('berth_receive_topic must be different from stop_receive_topic')
        for name, topic in (
            ("ownship_receive_topic", self.ownship_receive_topic),
            ("obstacle_receive_topic", self.obstacle_receive_topic),
        ):
            if topic and topic == self.receive_topic:
                raise ValueError(f"{name} must be different from receive_topic")
            if topic and self.stop_receive_topic and topic == self.stop_receive_topic:
                raise ValueError(f"{name} must be different from stop_receive_topic")
            if topic and self.berth_receive_topic and topic == self.berth_receive_topic:
                raise ValueError(f"{name} must be different from berth_receive_topic")
        if (
            self.ownship_receive_topic
            and self.obstacle_receive_topic
            and self.ownship_receive_topic == self.obstacle_receive_topic
        ):
            raise ValueError("ownship_receive_topic must be different from obstacle_receive_topic")
        if self.ownship_json_topic and not self.ownship_json_topic.strip():
            raise ValueError("ownship_json_topic must be empty or a non-blank topic")
        if self.obstacle_json_topic and not self.obstacle_json_topic.strip():
            raise ValueError("obstacle_json_topic must be empty or a non-blank topic")
        if self.ownship_output_topic and not self.ownship_output_topic.strip():
            raise ValueError("ownship_output_topic must be empty or a non-blank topic")
        if self.obstacle_output_topic and not self.obstacle_output_topic.strip():
            raise ValueError("obstacle_output_topic must be empty or a non-blank topic")
        for name, topic in (
            ("ownship_input_topic", self.ownship_input_topic),
            ("obstacle_input_topic", self.obstacle_input_topic),
            ("ownship_output_topic", self.ownship_output_topic),
            ("obstacle_output_topic", self.obstacle_output_topic),
            ("ownship_json_topic", self.ownship_json_topic),
            ("obstacle_json_topic", self.obstacle_json_topic),
        ):
            _reject_numeric_ros_token(name, topic)
        if not self.ownship_frame_id.strip():
            raise ValueError("ownship_frame_id must not be empty")
        if not self.ownship_child_frame_id.strip():
            raise ValueError("ownship_child_frame_id must not be empty")
        if not -90.0 <= self.origin_latitude <= 90.0:
            raise ValueError("origin_latitude must be between -90 and 90")
        if not -180.0 <= self.origin_longitude <= 180.0:
            raise ValueError("origin_longitude must be between -180 and 180")
        if self.src < 0:
            raise ValueError("src must be non-negative")
        if self.dst < 0:
            raise ValueError("dst must be non-negative")
        if self.max_payload_bytes <= 0:
            raise ValueError("max_payload_bytes must be greater than zero")
        if self.queue_depth <= 0:
            raise ValueError("queue_depth must be greater than zero")
        if self.poll_interval_sec <= 0.0:
            raise ValueError("poll_interval_sec must be greater than zero")
        if not self.status_send_topic.strip():
            raise ValueError("status_send_topic must not be empty")
        if self.status_send_topic in {self.ownship_send_topic, self.obstacle_send_topic}:
            raise ValueError(
                "status_send_topic must be different from ownship_send_topic "
                "and obstacle_send_topic"
            )
        if not self.heartbeat_input_topic.strip():
            raise ValueError("heartbeat_input_topic must not be empty")
        _reject_numeric_ros_token("heartbeat_input_topic", self.heartbeat_input_topic)
        if not self.progress_send_topic.strip():
            raise ValueError("progress_send_topic must not be empty")
        if self.progress_send_topic in {
            self.ownship_send_topic,
            self.obstacle_send_topic,
            self.status_send_topic,
        }:
            raise ValueError(
                "progress_send_topic must be different from ownship_send_topic, "
                "obstacle_send_topic and status_send_topic"
            )


# ROS 话题每一段不能以数字开头；LaDDS 主题 0411/0141 不能直接当 ROS 名。
def _reject_numeric_ros_token(name: str, value: str) -> None:
    if not value or not value.strip():
        return
    for token in value.strip().strip("/").split("/"):
        if token and token[0].isdigit():
            raise ValueError(
                f"{name} ROS topic token must not start with a number: {value}"
            )


def decode_utf8(payload: bytes) -> Optional[str]:
    """Return decoded text only when the complete payload is valid UTF-8."""
    try:
        return payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
        return None


def validate_obstacle_payload(text: str) -> Dict[str, Any]:
    """Validate the structural part of a 0141 obstacle JSON payload.

    The 0141 protocol defines the top-level ``count`` and ``targets`` fields.
    Simulation fills per-target attributes in ``usv_ladds_radar``; the bridge
    only checks that the list is unambiguous and leaves target fields untouched.
    The caller then forwards the original UTF-8 JSON without changing names,
    numeric precision, or optional values.
    """
    if not isinstance(text, str):
        raise ValueError("obstacle payload must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid obstacle JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("obstacle JSON root must be an object")

    if "count" not in root:
        raise ValueError("obstacle JSON is missing required field: count")
    count = root["count"]
    if type(count) is not int:
        raise ValueError("count must be an integer")
    if not 0 <= count <= 255:
        raise ValueError("count must be in [0, 255]")

    if "targets" not in root:
        raise ValueError("obstacle JSON is missing required field: targets")
    targets = root["targets"]
    if type(targets) is not list:
        raise ValueError("targets must be a list")
    if len(targets) != count:
        raise ValueError("count must match the number of targets")
    for index, target in enumerate(targets):
        if type(target) is not dict:
            raise ValueError(f"targets[{index}] must be an object")

    return root


def receive_channel(
    topic: str,
    receive_topic: str,
    stop_topic: str,
    ownship_topic: str = "",
    obstacle_topic: str = "",
    berth_topic: str = "",
) -> Optional[str]:
    """Route by transport metadata; preserve generic bridge when stop is disabled."""
    if ownship_topic and topic == ownship_topic:
        return "ownship"
    if obstacle_topic and topic == obstacle_topic:
        return "obstacle"
    if berth_topic and topic == berth_topic:
        return "berth"
    if not stop_topic:
        return "route"
    if topic == stop_topic:
        return "stop"
    if topic == receive_topic:
        return "route"
    return None


class HeaderCache:
    """跨收发节点保存源里程计时间戳，0411 JSON 本身没有时间字段。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Optional[Tuple[Any, str, str]] = None

    def store(self, stamp: Any, frame_id: str, child_frame_id: str) -> None:
        with self._lock:
            self._value = (stamp, frame_id, child_frame_id)

    def load(self) -> Optional[Tuple[Any, str, str]]:
        with self._lock:
            return self._value


def drain_received(client: Any, limit: int) -> List[Any]:
    """Read only the packets currently available, capped to keep the ROS executor responsive."""
    packets: List[Any] = []
    for _ in range(limit):
        packet = client.receive()
        if packet is None:
            break
        packets.append(packet)
    return packets


def packet_info_json(packet: Any) -> str:
    """Serialize the stable receive metadata exposed on ``/ladds/rx_info``."""
    return json.dumps(
        {
            "domain": packet.domain_id,
            "topic": packet.topic,
            "src": packet.src,
            "dst": packet.dst,
            "length": len(packet.payload),
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )
