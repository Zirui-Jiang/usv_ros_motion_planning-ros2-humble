"""Small bounded framing protocol shared by the TCP relay peers."""

from __future__ import annotations

import base64
import binascii
import json
import struct
from typing import Any, Dict, List


_HEADER = struct.Struct("!I")
_FRAME_OVERHEAD = 4096


class WireProtocolError(ValueError):
    """Raised when a relay peer sends an invalid or oversized frame."""


def max_frame_bytes(max_payload_bytes: int) -> int:
    """Return a bounded JSON frame size that includes base64 expansion."""
    return 4 * ((max_payload_bytes + 2) // 3) + _FRAME_OVERHEAD


def encode_frame(message: Dict[str, Any], max_payload_bytes: int) -> bytes:
    """Encode one message as a length-prefixed compact JSON frame."""
    encoded = dict(message)
    payload = encoded.pop("payload", None)
    if payload is not None:
        if not isinstance(payload, bytes):
            raise TypeError("payload must be bytes")
        if len(payload) > max_payload_bytes:
            raise WireProtocolError(
                f"payload exceeds {max_payload_bytes} byte limit"
            )
        encoded["payload_b64"] = base64.b64encode(payload).decode("ascii")
    body = json.dumps(encoded, ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )
    limit = max_frame_bytes(max_payload_bytes)
    if not body or len(body) > limit:
        raise WireProtocolError(f"frame exceeds {limit} byte limit")
    return _HEADER.pack(len(body)) + body


class FrameReader:
    """Incrementally decode frames without discarding partial socket reads."""

    def __init__(self, max_frame_size: int) -> None:
        if max_frame_size < 1:
            raise ValueError("max_frame_size must be positive")
        self._max_frame_size = max_frame_size
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Dict[str, Any]]:
        self._buffer.extend(data)
        messages: List[Dict[str, Any]] = []
        while len(self._buffer) >= _HEADER.size:
            size = _HEADER.unpack_from(self._buffer)[0]
            if size < 1 or size > self._max_frame_size:
                self._buffer.clear()
                raise WireProtocolError(
                    f"frame size {size} is outside 1..{self._max_frame_size}"
                )
            end = _HEADER.size + size
            if len(self._buffer) < end:
                break
            body = bytes(self._buffer[_HEADER.size:end])
            del self._buffer[:end]
            messages.append(_decode_body(body))
        return messages


def _decode_body(body: bytes) -> Dict[str, Any]:
    try:
        message = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise WireProtocolError("frame is not valid UTF-8 JSON") from exc
    if not isinstance(message, dict) or not isinstance(message.get("type"), str):
        raise WireProtocolError("frame must be an object with a string type")
    if "payload_b64" in message:
        value = message.pop("payload_b64")
        if not isinstance(value, str):
            raise WireProtocolError("payload_b64 must be a string")
        try:
            message["payload"] = base64.b64decode(value, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise WireProtocolError("payload_b64 is not valid base64") from exc
    return message
