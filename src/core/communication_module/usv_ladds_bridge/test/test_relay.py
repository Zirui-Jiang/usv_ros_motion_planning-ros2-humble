import os
import queue
import socket
import struct
import threading
import time

import pytest

from usv_ladds_bridge.ladds_client import LaDDSError, ReceivedPacket
from usv_ladds_bridge.relay import RelayServer
from usv_ladds_bridge.tcp_client import TcpRelayClient
from usv_ladds_bridge.wire import FrameReader, WireProtocolError, encode_frame


class FakeDDS:
    """DDS boundary fake; this does not exercise the vendor SDK or network."""

    version = "FakeDDS 1.0"

    def __init__(self):
        self.incoming = queue.Queue()
        self.sent = []
        self.send_error = None
        self.start_calls = 0
        self.closed = False
        self.dropped_messages = 0

    def start(self):
        self.start_calls += 1

    def send(self, payload, topic=None):
        if self.send_error is not None:
            raise self.send_error
        self.sent.append(payload if topic is None else (topic, payload))

    def receive(self):
        try:
            return self.incoming.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


@pytest.fixture
def relay_pair():
    dds = FakeDDS()
    server = RelayServer(
        dds, listen_host="127.0.0.1", port=0, max_payload_bytes=1024,
        io_timeout=0.02,
    )
    server.start()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = TcpRelayClient(
        "127.0.0.1", port=server.address[1], timeout=0.5,
        max_payload_bytes=1024,
    )
    client.start()
    try:
        yield dds, server, client
    finally:
        client.close()
        server.close()
        thread.join(timeout=1)


def _receive_until(client, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        packet = client.receive()
        if packet is not None:
            return packet
        time.sleep(0.005)
    return None


def test_tcp_relay_keeps_stop_topic_separate_from_route(relay_pair):
    dds, _, client = relay_pair
    for topic in ('0211', '0331'):
        dds.incoming.put(ReceivedPacket(1, topic, b'{}', 1000, 1001))
        received = _receive_until(client)
        assert received.topic == topic
        assert received.payload == b'{}'


def test_tcp_relay_forwards_selected_publish_topic(relay_pair):
    dds, _, client = relay_pair

    client.send(b'0411-json', topic='0411')

    assert dds.sent == [('0411', b'0411-json')]


def test_tcp_relay_forwards_obstacle_publish_topic(relay_pair):
    dds, _, client = relay_pair

    client.send(b'0141-json', topic='0141')

    assert dds.sent == [('0141', b'0141-json')]


def test_client_keeps_rx_coalesced_in_same_tcp_read_as_hello():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    frames = encode_frame({"type": "hello", "version": "coalesced"}, 64)
    frames += encode_frame(
        {
            "type": "rx", "domain_id": 1, "topic": "0211",
            "payload": b"\x00\xff", "src": 7, "dst": 8,
        },
        64,
    )

    def serve_once():
        connection, _ = listener.accept()
        try:
            connection.sendall(frames)
            time.sleep(0.05)
        finally:
            connection.close()

    thread = threading.Thread(target=serve_once, daemon=True)
    thread.start()
    client = TcpRelayClient("127.0.0.1", listener.getsockname()[1], timeout=0.5)
    try:
        client.start()
        assert client.receive() == ReceivedPacket(1, "0211", b"\x00\xff", 7, 8)
    finally:
        client.close()
        listener.close()
        thread.join(timeout=1)


def test_frame_reader_keeps_partial_binary_frame_until_complete():
    frame = encode_frame({"type": "send", "payload": b"\x00\xff\x80data"}, 64)
    reader = FrameReader(512)

    assert reader.feed(frame[:2]) == []
    assert reader.feed(frame[2:9]) == []
    assert reader.feed(frame[9:]) == [
        {"type": "send", "payload": b"\x00\xff\x80data"}
    ]


def test_frame_reader_rejects_announced_frame_larger_than_limit():
    reader = FrameReader(32)

    with pytest.raises(WireProtocolError, match="frame.*32"):
        reader.feed(struct.pack("!I", 33))


def test_connection_failure_uses_same_observable_ladds_error_contract():
    unused = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    unused.bind(("127.0.0.1", 0))
    port = unused.getsockname()[1]
    unused.close()

    client = TcpRelayClient("127.0.0.1", port=port, timeout=0.1)
    with pytest.raises(LaDDSError, match="connect"):
        client.start()


def test_loopback_relay_preserves_bidirectional_binary_payload_and_metadata(relay_pair):
    dds, _server, client = relay_pair
    from_ros = b"\x00\xff\x80ros-to-dds"
    from_dds = b"\x00\xfe\x81dds-to-ros"

    dds.incoming.put(ReceivedPacket(1, "0211", from_dds, 41, 52))
    client.send(from_ros)
    packet = _receive_until(client)

    assert dds.sent == [from_ros]
    assert packet == ReceivedPacket(1, "0211", from_dds, 41, 52)
    assert client.version == "FakeDDS 1.0"


def test_sdk_send_failure_is_returned_to_requesting_client(relay_pair):
    dds, _server, client = relay_pair
    dds.send_error = LaDDSError("SDK SendEx returned 0")

    with pytest.raises(LaDDSError, match="SDK SendEx returned 0"):
        client.send(b"command")

    assert dds.sent == []


def test_second_client_gets_busy_error_and_first_client_still_works(relay_pair):
    dds, server, first = relay_pair
    second = TcpRelayClient("127.0.0.1", server.address[1], timeout=0.5)
    try:
        with pytest.raises(LaDDSError, match="relay busy.*already connected"):
            second.start()
        first.send(b"original client remains connected")
        assert dds.sent == [b"original client remains connected"]
    finally:
        second.close()


def test_duplicate_listener_is_rejected_before_starting_another_dds(relay_pair):
    _dds, server, first = relay_pair
    other_dds = FakeDDS()
    duplicate = RelayServer(other_dds, "127.0.0.1", server.address[1])
    try:
        with pytest.raises(OSError):
            duplicate.start()
        assert other_dds.start_calls == 0
        first.send(b"original listener remains usable")
    finally:
        duplicate.close()


def test_sdk_start_failure_releases_listener():
    class FailingDDS(FakeDDS):
        def start(self):
            raise LaDDSError("SDK unavailable")

    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    dds = FailingDDS()
    server = RelayServer(dds, "127.0.0.1", port)
    with pytest.raises(LaDDSError, match="SDK unavailable"):
        server.start()
    assert dds.closed is True
    replacement = RelayServer(FakeDDS(), "127.0.0.1", port)
    try:
        replacement.start()
    finally:
        replacement.close()


def test_send_ack_wait_is_bounded_when_sdk_call_stalls():
    class SlowDDS(FakeDDS):
        def send(self, payload):
            time.sleep(0.2)
            super().send(payload)

    dds = SlowDDS()
    server = RelayServer(dds, "127.0.0.1", 0, io_timeout=0.01)
    server.start()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = TcpRelayClient("127.0.0.1", server.address[1], timeout=0.05)
    client.start()
    started = time.monotonic()
    try:
        with pytest.raises(LaDDSError, match="timed out"):
            client.send(b"command")
        assert time.monotonic() - started < 0.15
    finally:
        client.close()
        server.close()
        thread.join(timeout=1)


def test_server_keeps_dds_alive_and_accepts_a_new_client_after_disconnect(relay_pair):
    dds, server, first = relay_pair
    first.close()
    second = TcpRelayClient("127.0.0.1", server.address[1], timeout=0.5)
    try:
        second.start()
        second.send(b"after reconnect")
    finally:
        second.close()

    assert dds.start_calls == 1
    assert dds.closed is False
    assert dds.sent == [b"after reconnect"]


def test_client_observes_server_disconnect_instead_of_reporting_send_success(relay_pair):
    _dds, server, client = relay_pair
    server.close()

    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        try:
            client.send(b"must fail")
        except LaDDSError:
            break
        time.sleep(0.005)
    else:
        pytest.fail("client continued reporting success after relay closed")


def test_receive_raises_after_queue_is_drained_when_relay_disconnects(relay_pair):
    _dds, server, client = relay_pair
    server.close()

    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        try:
            client.receive()
        except LaDDSError as exc:
            assert "closed" in str(exc) or "connection" in str(exc)
            break
        time.sleep(0.005)
    else:
        pytest.fail("receive() hid the relay disconnect")


def test_client_rejects_payload_above_configured_limit(relay_pair):
    _dds, server, current = relay_pair
    current.close()
    limited = TcpRelayClient(
        "127.0.0.1", server.address[1], timeout=0.5, max_payload_bytes=3
    )
    try:
        limited.start()
        with pytest.raises(LaDDSError, match="3 byte limit"):
            limited.send(b"four")
    finally:
        limited.close()


def test_full_receive_queue_drops_new_packet_like_native_client(relay_pair):
    dds, server, current = relay_pair
    current.close()
    limited = TcpRelayClient(
        "127.0.0.1", server.address[1], timeout=0.5,
        max_payload_bytes=1024, queue_depth=2,
    )
    limited.start()
    try:
        for value in range(3):
            dds.incoming.put(ReceivedPacket(1, "0211", bytes([value]), 0, 0))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and limited.dropped_messages == 0:
            time.sleep(0.005)

        assert limited.dropped_messages == 1
        assert limited.receive().payload == b"\x00"
        assert limited.receive().payload == b"\x01"
    finally:
        limited.close()


def test_send_failure_on_reset_peer_does_not_stop_server_or_close_dds():
    class GatedDDS(FakeDDS):
        def __init__(self):
            super().__init__()
            self.packet = None
            self.delivery_started = threading.Event()
            self.allow_delivery = threading.Event()

        def receive(self):
            if self.packet is None:
                return super().receive()
            packet, self.packet = self.packet, None
            self.delivery_started.set()
            self.allow_delivery.wait(timeout=0.5)
            return packet

    dds = GatedDDS()
    server = RelayServer(dds, "127.0.0.1", 0, io_timeout=0.01)
    server.start()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    reset_peer = None
    replacement = None
    try:
        reset_peer = socket.create_connection(server.address, timeout=0.5)
        reset_peer.recv(4096)  # hello
        dds.packet = ReceivedPacket(1, "0211", b"pending", 1, 2)
        assert dds.delivery_started.wait(timeout=0.5)
        linger_format = "hh" if os.name == "nt" else "ii"
        reset_peer.setsockopt(
            socket.SOL_SOCKET, socket.SO_LINGER,
            struct.pack(linger_format, 1, 0),
        )
        reset_peer.close()
        reset_peer = None
        time.sleep(0.02)
        dds.allow_delivery.set()

        replacement = TcpRelayClient(
            "127.0.0.1", server.address[1], timeout=0.5
        )
        replacement.start()
        replacement.send(b"still alive")
        assert dds.sent == [b"still alive"]
        assert dds.closed is False
    finally:
        dds.allow_delivery.set()
        if reset_peer is not None:
            reset_peer.close()
        if replacement is not None:
            replacement.close()
        server.close()
        thread.join(timeout=1)
