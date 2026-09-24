"""Windows-side LaDDS to TCP relay."""

from __future__ import annotations

import argparse
import select
import socket
import threading
from typing import Optional

from .ladds_client import LaDDSClient
from .wire import FrameReader, WireProtocolError, encode_frame, max_frame_bytes


class RelayServer:
    """Serve one TCP client at a time while keeping one LaDDS handle alive."""

    def __init__(
        self,
        dds_client,
        listen_host: str = "127.0.0.1",
        port: int = 17880,
        max_payload_bytes: int = 1_048_576,
        queue_depth: int = 100,
        io_timeout: float = 0.1,
    ) -> None:
        if not isinstance(listen_host, str) or not listen_host:
            raise ValueError("listen_host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        if max_payload_bytes < 1 or queue_depth < 1 or io_timeout <= 0:
            raise ValueError("relay limits and io_timeout must be positive")
        self._dds = dds_client
        self._listen_host = listen_host
        self._port = port
        self._max_payload_bytes = max_payload_bytes
        self._queue_depth = queue_depth
        self._io_timeout = io_timeout
        self._listener: Optional[socket.socket] = None
        self._connection: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._started = False
        self._address = (listen_host, port)

    @property
    def address(self):
        return self._address

    def start(self) -> None:
        if self._started:
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Windows SO_REUSEADDR permits a second listener on the same port.
            # Reserve it exclusively before opening another licensed DDS handle.
            address_option = getattr(socket, "SO_EXCLUSIVEADDRUSE", socket.SO_REUSEADDR)
            listener.setsockopt(socket.SOL_SOCKET, address_option, 1)
            listener.bind((self._listen_host, self._port))
            listener.listen(1)
            listener.settimeout(self._io_timeout)
        except Exception:
            listener.close()
            self._dds.close()
            raise
        try:
            self._dds.start()
        except Exception:
            listener.close()
            self._dds.close()
            raise
        self._listener = listener
        self._address = listener.getsockname()
        self._started = True

    def serve_forever(self) -> None:
        if not self._started:
            self.start()
        try:
            while not self._stop.is_set():
                try:
                    connection, _peer = self._listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self._connection = connection
                try:
                    self._serve_client(connection)
                except (OSError, ConnectionError, WireProtocolError):
                    if self._stop.is_set():
                        break
                finally:
                    self._connection = None
                    connection.close()
        finally:
            self.close()

    def close(self) -> None:
        self._stop.set()
        connection, self._connection = self._connection, None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        listener, self._listener = self._listener, None
        if listener is not None:
            listener.close()
        if self._started:
            self._started = False
            self._dds.close()

    def _serve_client(self, connection: socket.socket) -> None:
        connection.settimeout(self._io_timeout)
        reader = FrameReader(max_frame_bytes(self._max_payload_bytes))
        hello = {"type": "hello", "version": self._dds.version or ""}
        # Let the ROS side verify its send topics were registered at SDK start.
        send_topics = getattr(self._dds, "send_topics", None)
        if send_topics is not None:
            hello["send_topics"] = list(send_topics)
        self._send_frame(connection, hello)
        while not self._stop.is_set():
            for _ in range(self._queue_depth):
                packet = self._dds.receive()
                if packet is None:
                    break
                self._send_frame(
                    connection,
                    {
                        "type": "rx",
                        "domain_id": packet.domain_id,
                        "topic": packet.topic,
                        "payload": packet.payload,
                        "src": packet.src,
                        "dst": packet.dst,
                    },
                )
            try:
                data = connection.recv(65536)
            except socket.timeout:
                self._reject_extra_client()
                continue
            except OSError:
                return
            if not data:
                return
            try:
                messages = reader.feed(data)
            except WireProtocolError as exc:
                self._send_error(connection, None, str(exc))
                return
            for message in messages:
                if not self._handle_message(connection, message):
                    return
            self._reject_extra_client()

    def _reject_extra_client(self) -> None:
        """Explain single-client ownership instead of leaving a peer in backlog."""
        listener = self._listener
        if listener is None or self._stop.is_set():
            return
        try:
            ready, _, _ = select.select([listener], [], [], 0)
            if not ready:
                return
            extra, _peer = listener.accept()
        except (OSError, ValueError):
            return  # The listener may have been closed by shutdown.
        with extra:
            extra.settimeout(self._io_timeout)
            try:
                self._send_error(
                    extra, None,
                    "relay busy: another ROS bridge is already connected; "
                    "stop the existing bridge before starting another one",
                )
            except OSError:
                pass  # A rejected peer must not interrupt the active client.

    def _handle_message(self, connection: socket.socket, message: dict) -> bool:
        request_id = message.get("id")
        if message.get("type") != "send" or not isinstance(request_id, int):
            self._send_error(connection, request_id, "expected send with integer id")
            return False
        payload = message.get("payload")
        if not isinstance(payload, bytes) or len(payload) > self._max_payload_bytes:
            self._send_error(connection, request_id, "invalid or oversized payload")
            return True
        topic = message.get("topic")
        if topic is not None and (not isinstance(topic, str) or not topic):
            self._send_error(connection, request_id, "topic must be a non-empty string")
            return True
        try:
            if topic is None:
                self._dds.send(payload)
            else:
                self._dds.send(payload, topic=topic)
        except Exception as exc:
            self._send_error(connection, request_id, str(exc) or type(exc).__name__)
        else:
            self._send_frame(connection, {"type": "sent", "id": request_id})
        return True

    def _send_error(self, connection, request_id, message: str) -> None:
        frame = {"type": "error", "message": message}
        if isinstance(request_id, int):
            frame["id"] = request_id
        self._send_frame(connection, frame)

    def _send_frame(self, connection: socket.socket, message: dict) -> None:
        connection.sendall(encode_frame(message, self._max_payload_bytes))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Relay Windows LaDDS over TCP")
    parser.add_argument("--library", required=True, help="path to LaDDS.dll")
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=17880)
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--bind-ip", default="")
    parser.add_argument("--ttl", type=int, default=1)
    parser.add_argument("--qos", choices=("reliable", "best_effort"), default="best_effort")
    parser.add_argument("--send-topic", default="0211")
    parser.add_argument("--ownship-send-topic", default="0411")
    parser.add_argument("--obstacle-send-topic", default="0141")
    parser.add_argument("--status-send-topic", default="0051")
    parser.add_argument("--progress-send-topic", default="0053")
    parser.add_argument(
        "--thrust-send-topic",
        default="",
        help="optional thruster command publication, e.g. 0054",
    )
    parser.add_argument("--receive-topic", default="0211")
    parser.add_argument('--stop-receive-topic', default='', help='optional extra stop subscription, e.g. 0331')
    parser.add_argument(
        '--berth-receive-topic',
        default='',
        help='optional extra berth subscription, e.g. 0212',
    )
    parser.add_argument(
        "--ownship-receive-topic",
        default="0411",
        help="LaDDS subscription for ownship 0411 JSON",
    )
    parser.add_argument(
        "--obstacle-receive-topic",
        default="0141",
        help="LaDDS subscription for obstacle 0141 JSON",
    )
    parser.add_argument("--src", type=int, default=0)
    parser.add_argument("--dst", type=int, default=0)
    parser.add_argument("--max-payload-bytes", type=int, default=1_048_576)
    parser.add_argument("--queue-depth", type=int, default=100)
    parser.add_argument("--encryption", action="store_true")
    return parser


def main() -> None:
    args = _parser().parse_args()
    dds = LaDDSClient(
        args.library,
        domain_id=args.domain_id,
        bind_ip=args.bind_ip,
        ttl=args.ttl,
        qos=args.qos,
        send_topic=args.send_topic,
        ownship_send_topic=args.ownship_send_topic,
        obstacle_send_topic=args.obstacle_send_topic,
        status_send_topic=args.status_send_topic,
        progress_send_topic=args.progress_send_topic,
        thrust_send_topic=args.thrust_send_topic,
        receive_topic=args.receive_topic,
        stop_receive_topic=args.stop_receive_topic,
        berth_receive_topic=args.berth_receive_topic,
        ownship_receive_topic=args.ownship_receive_topic,
        obstacle_receive_topic=args.obstacle_receive_topic,
        src=args.src,
        dst=args.dst,
        max_payload_bytes=args.max_payload_bytes,
        queue_depth=args.queue_depth,
        encryption=args.encryption,
    )
    server = RelayServer(
        dds,
        listen_host=args.listen,
        port=args.port,
        max_payload_bytes=args.max_payload_bytes,
        queue_depth=args.queue_depth,
    )
    server.start()
    print(
        f"LaDDS relay listening on {server.address[0]}:{server.address[1]} "
        f"(SDK {dds.version or 'unknown'}; send topics {', '.join(dds.send_topics)})",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.close()


if __name__ == "__main__":
    main()
