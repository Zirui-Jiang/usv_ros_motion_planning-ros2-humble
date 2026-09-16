"""ROS-side client for a LaDDS instance hosted by the Windows relay."""

from __future__ import annotations

import queue
import socket
import threading
import time
from typing import Dict, Optional

from .ladds_client import LaDDSError, ReceivedPacket
from .wire import FrameReader, WireProtocolError, encode_frame, max_frame_bytes


class TcpRelayClient:
    """Expose the LaDDSClient interface over one bounded TCP connection."""

    def __init__(
        self,
        host: str,
        port: int = 17880,
        timeout: float = 5.0,
        max_payload_bytes: int = 1_048_576,
        queue_depth: int = 100,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        if not isinstance(timeout, (int, float)) or timeout <= 0:
            raise ValueError("timeout must be positive")
        if max_payload_bytes < 1 or queue_depth < 1:
            raise ValueError("payload and queue limits must be positive")
        self._host = host
        self._port = port
        self._timeout = float(timeout)
        self._max_payload_bytes = max_payload_bytes
        self._received: queue.Queue[ReceivedPacket] = queue.Queue(queue_depth)
        self._socket: Optional[socket.socket] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: Dict[int, tuple[threading.Event, list]] = {}
        self._next_id = 1
        self._version: Optional[str] = None
        self._dropped_messages = 0
        self._disconnect_error: Optional[str] = None

    @property
    def version(self) -> Optional[str]:
        return self._version

    @property
    def dropped_messages(self) -> int:
        return self._dropped_messages

    def start(self) -> None:
        if self._socket is not None and self._disconnect_error is None:
            return
        self._stop.clear()
        self._disconnect_error = None
        try:
            sock = socket.create_connection((self._host, self._port), self._timeout)
        except OSError as exc:
            raise LaDDSError(
                f"could not connect to relay {self._host}:{self._port}: {exc}"
            ) from exc
        sock.settimeout(min(self._timeout, 0.2))
        reader = FrameReader(max_frame_bytes(self._max_payload_bytes))
        try:
            hello, queued = self._read_hello(sock, reader)
            if hello.get("type") == "error":
                raise LaDDSError(str(hello.get("message", "relay rejected connection")))
            if hello.get("type") != "hello":
                raise LaDDSError("relay did not send a hello frame")
            version = hello.get("version")
            self._version = version if isinstance(version, str) else ""
        except Exception as exc:
            sock.close()
            if isinstance(exc, LaDDSError):
                raise
            raise LaDDSError(f"relay handshake failed: {exc}") from exc
        self._socket = sock
        for message in queued:
            self._dispatch(message)
        self._reader_thread = threading.Thread(
            target=self._read_loop, args=(sock, reader), daemon=True
        )
        self._reader_thread.start()

    def send(self, payload: bytes) -> None:
        if not isinstance(payload, bytes):
            raise TypeError("payload must be bytes")
        if len(payload) > self._max_payload_bytes:
            raise LaDDSError(
                f"payload exceeds {self._max_payload_bytes} byte limit"
            )
        with self._request_lock:
            sock = self._require_connection()
            request_id = self._next_id
            self._next_id += 1
            event = threading.Event()
            outcome: list = []
            with self._pending_lock:
                self._pending[request_id] = (event, outcome)
            try:
                frame = encode_frame(
                    {"type": "send", "id": request_id, "payload": payload},
                    self._max_payload_bytes,
                )
                with self._send_lock:
                    sock.sendall(frame)
            except (OSError, WireProtocolError) as exc:
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                self._mark_disconnected(f"relay send failed: {exc}")
                self._discard_socket(sock)
                raise LaDDSError(self._disconnect_error) from exc
            if not event.wait(self._timeout):
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                raise LaDDSError("timed out waiting for LaDDS Send acknowledgement")
            if outcome and outcome[0] is not None:
                raise LaDDSError(outcome[0])

    def receive(self) -> Optional[ReceivedPacket]:
        try:
            return self._received.get_nowait()
        except queue.Empty:
            if self._disconnect_error is not None:
                raise LaDDSError(self._disconnect_error)
            return None

    def close(self) -> None:
        self._stop.set()
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        self._mark_disconnected("relay client is closed")
        thread = self._reader_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self._timeout)
        self._reader_thread = None

    def _read_hello(self, sock: socket.socket, reader: FrameReader):
        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                raise LaDDSError("relay disconnected before hello")
            messages = reader.feed(data)
            if messages:
                return messages[0], messages[1:]
        raise LaDDSError("timed out waiting for relay hello")

    def _read_loop(self, sock: socket.socket, reader: FrameReader) -> None:
        try:
            while not self._stop.is_set():
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("relay closed the connection")
                for message in reader.feed(data):
                    self._dispatch(message)
        except (OSError, ConnectionError, WireProtocolError) as exc:
            if not self._stop.is_set():
                self._mark_disconnected(str(exc))
        finally:
            self._discard_socket(sock)

    def _dispatch(self, message: dict) -> None:
        kind = message.get("type")
        if kind == "rx":
            payload = message.get("payload")
            if not isinstance(payload, bytes) or len(payload) > self._max_payload_bytes:
                raise WireProtocolError("invalid rx payload")
            try:
                packet = ReceivedPacket(
                    domain_id=int(message["domain_id"]),
                    topic=str(message["topic"]),
                    payload=payload,
                    src=int(message["src"]),
                    dst=int(message["dst"]),
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise WireProtocolError("invalid rx metadata") from exc
            if self._received.full():
                self._dropped_messages += 1
                return
            self._received.put_nowait(packet)
            return
        if kind in {"sent", "error"}:
            request_id = message.get("id")
            if not isinstance(request_id, int):
                raise WireProtocolError("acknowledgement is missing integer id")
            with self._pending_lock:
                pending = self._pending.pop(request_id, None)
            if pending is not None:
                event, outcome = pending
                error = message.get("message", "LaDDS Send failed") if kind == "error" else None
                outcome.append(str(error) if error is not None else None)
                event.set()
            return
        raise WireProtocolError(f"unexpected relay message type: {kind!r}")

    def _require_connection(self) -> socket.socket:
        if self._disconnect_error is not None or self._socket is None:
            raise LaDDSError(self._disconnect_error or "relay client is not started")
        return self._socket

    def _mark_disconnected(self, reason: str) -> None:
        if self._disconnect_error is None:
            self._disconnect_error = reason
        with self._pending_lock:
            pending = list(self._pending.values())
            self._pending.clear()
        for event, outcome in pending:
            outcome.append(self._disconnect_error)
            event.set()

    def _discard_socket(self, sock: socket.socket) -> None:
        if self._socket is sock:
            self._socket = None
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()
