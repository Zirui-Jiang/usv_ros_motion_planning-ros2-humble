"""ROS-independent bridge configuration and payload conversion helpers."""

from dataclasses import dataclass
import json
import os
from typing import Any, List, Mapping, Optional


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
    receive_topic: str = "0211"
    src: int = 0
    dst: int = 0
    encryption: bool = False
    max_payload_bytes: int = 1_048_576
    queue_depth: int = 100
    poll_interval_sec: float = 0.01

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
            receive_topic=str(values.get("receive_topic", "0211")),
            src=int(values.get("src", 0)),
            dst=int(values.get("dst", 0)),
            encryption=bool(values.get("encryption", False)),
            max_payload_bytes=int(values.get("max_payload_bytes", 1_048_576)),
            queue_depth=int(values.get("queue_depth", 100)),
            poll_interval_sec=float(values.get("poll_interval_sec", 0.01)),
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
        if not self.receive_topic.strip():
            raise ValueError("receive_topic must not be empty")
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


def decode_utf8(payload: bytes) -> Optional[str]:
    """Return decoded text only when the complete payload is valid UTF-8."""
    try:
        return payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
        return None


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
