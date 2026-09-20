import ctypes
from dataclasses import FrozenInstanceError
import os
import pytest
import threading

from usv_ladds_bridge.ladds_client import LaDDSClient, LaDDSError, ReceivedPacket


class FakeFunction:
    def __init__(self, implementation):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class FakeLibrary:
    def __init__(self, *, handle=2**40 + 7):
        self.calls = []
        self.callback = None
        self.LaDDS_GetVersion = FakeFunction(lambda: b"LaDDS 1.0.3")
        self.LaDDS_Create = FakeFunction(
            lambda domain_id, ttl, bind_ip: self._record_create(
                handle, domain_id, ttl, bind_ip
            )
        )
        self.LaDDS_Destroy = FakeFunction(
            lambda native_handle: self.calls.append(("destroy", native_handle))
        )
        self.LaDDS_EnableEncryption = FakeFunction(
            lambda native_handle, on: self._record_success("encryption", native_handle, on)
        )
        self.LaDDS_RegPubTopic = FakeFunction(
            lambda native_handle, topic, qos: self._record_success(
                "pub", native_handle, topic, qos
            )
        )
        self.LaDDS_RegSubTopic = FakeFunction(
            lambda native_handle, topic, qos: self._record_success(
                "sub", native_handle, topic, qos
            )
        )
        self.LaDDS_SetRecvCallback = FakeFunction(self._set_callback)
        self.LaDDS_SendEx = FakeFunction(self._record_send)

    def _record_create(self, handle, domain_id, ttl, bind_ip):
        self.calls.append(("create", domain_id, ttl, bind_ip))
        return handle

    def _record_success(self, name, *args):
        self.calls.append((name, *args))
        return 1

    def _set_callback(self, native_handle, callback):
        self.calls.append(("callback", native_handle))
        self.callback = callback

    def _record_send(self, native_handle, topic, payload, size, src, dst):
        copied_payload = ctypes.string_at(payload, size)
        return self._record_success(
            "send", native_handle, topic, copied_payload, size, src, dst
        )

    def emit(self, payload, *, domain_id=17, topic=b"in", src=23, dst=42):
        buffer = ctypes.create_string_buffer(payload, len(payload))
        self.callback(
            2**40 + 7,
            domain_id,
            topic,
            ctypes.cast(buffer, ctypes.c_void_p),
            len(payload),
            src,
            dst,
        )


def test_constructor_does_not_load_the_native_library():
    load_calls = []

    client = LaDDSClient("libLaDDS.so", library_loader=lambda path: load_calls.append(path))

    assert load_calls == []
    assert client.version is None
    assert client.receive() is None
    assert client.dropped_messages == 0


def test_optional_stop_topic_is_registered_and_original_topic_preserved():
    library = FakeLibrary()
    client = LaDDSClient('fake.dll', receive_topic='0211', stop_receive_topic='0331',
                        library_loader=lambda path: library)
    client.start()
    try:
        assert ('sub', 2**40 + 7, b'0211', 2) in library.calls
        assert ('sub', 2**40 + 7, b'0331', 2) in library.calls
        library.emit(b'{}', topic=b'0331')
        assert client.receive().topic == '0331'
    finally:
        client.close()


def test_start_loads_sdk_and_registers_both_topics_with_64_bit_handle():
    library = FakeLibrary()
    loaded_paths = []
    client = LaDDSClient(
        "libLaDDS.so",
        domain_id=17,
        bind_ip="192.168.1.51",
        ttl=3,
        qos="best_effort",
        send_topic="out",
        receive_topic="in",
        encryption=True,
        library_loader=lambda path: (loaded_paths.append(path), library)[1],
    )

    client.start()

    assert loaded_paths == ["libLaDDS.so"]
    assert client.version == "LaDDS 1.0.3"
    assert library.calls == [
        ("create", 17, 3, b"192.168.1.51"),
        ("callback", 2**40 + 7),
        ("encryption", 2**40 + 7, 1),
        ("pub", 2**40 + 7, b"out", 2),
        ("sub", 2**40 + 7, b"in", 2),
    ]
    assert ctypes.sizeof(library.LaDDS_Create.restype) == 8


def test_start_registers_ownship_publish_topic_and_send_can_select_it():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so",
        send_topic="0052",
        ownship_send_topic="0411",
        library_loader=lambda _path: library,
    )
    client.start()

    try:
        assert ("pub", 2**40 + 7, b"0052", 2) in library.calls
        assert ("pub", 2**40 + 7, b"0411", 2) in library.calls
        client.send(b"ownship", topic="0411")
        assert library.calls[-1] == (
            "send", 2**40 + 7, b"0411", b"ownship", 7, 0, 0
        )
    finally:
        client.close()


def test_start_registers_obstacle_publish_topic_and_send_can_select_it():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so",
        send_topic="0211",
        ownship_send_topic="0411",
        obstacle_send_topic="0141",
        library_loader=lambda _path: library,
    )
    client.start()

    try:
        assert ("pub", 2**40 + 7, b"0211", 2) in library.calls
        assert ("pub", 2**40 + 7, b"0411", 2) in library.calls
        assert ("pub", 2**40 + 7, b"0141", 2) in library.calls
        client.send(b"obstacle-json", topic="0141")
        assert library.calls[-1] == (
            "send", 2**40 + 7, b"0141", b"obstacle-json", 13, 0, 0
        )
    finally:
        client.close()


def test_start_registers_ownship_and_obstacle_subscribe_topics():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so",
        receive_topic="0211",
        stop_receive_topic="0331",
        ownship_receive_topic="0411",
        obstacle_receive_topic="0141",
        library_loader=lambda _path: library,
    )
    client.start()
    try:
        assert ("sub", 2**40 + 7, b"0211", 2) in library.calls
        assert ("sub", 2**40 + 7, b"0331", 2) in library.calls
        assert ("sub", 2**40 + 7, b"0411", 2) in library.calls
        assert ("sub", 2**40 + 7, b"0141", 2) in library.calls
        library.emit(b'{"latitude":30.0}', topic=b"0411")
        assert client.receive().topic == "0411"
    finally:
        client.close()


def test_send_rejects_unregistered_topic():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)
    client.start()
    try:
        with pytest.raises(ValueError, match="registered"):
            client.send(b"data", topic="0411")
    finally:
        client.close()


@pytest.mark.parametrize(
    ("overrides", "match"),
    [
        ({"library_path": ""}, "library_path"),
        ({"domain_id": -1}, "domain_id"),
        ({"ttl": 256}, "ttl"),
        ({"qos": "fast"}, "qos"),
        ({"send_topic": ""}, "send_topic"),
        ({"receive_topic": "bad\x00topic"}, "receive_topic"),
        ({"src": -1}, "src"),
        ({"dst": 2**32}, "dst"),
        ({"max_payload_bytes": 0}, "max_payload_bytes"),
        ({"max_payload_bytes": 2**31}, "max_payload_bytes"),
        ({"queue_depth": 0}, "queue_depth"),
        ({"encryption": 1}, "encryption"),
    ],
)
def test_constructor_rejects_values_that_cannot_be_passed_safely_to_sdk(
    overrides, match
):
    arguments = {"library_path": "libLaDDS.so"}
    arguments.update(overrides)

    with pytest.raises((TypeError, ValueError), match=match):
        LaDDSClient(**arguments)


def test_callback_copies_binary_payload_and_receive_is_non_blocking():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)
    client.start()

    library.emit(b"before\x00after")

    assert client.receive() == ReceivedPacket(
        domain_id=17,
        topic="in",
        payload=b"before\x00after",
        src=23,
        dst=42,
    )
    assert client.receive() is None


def test_send_uses_explicit_length_and_source_destination_fields():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so",
        send_topic="0211",
        src=12,
        dst=34,
        library_loader=lambda _path: library,
    )
    client.start()

    client.send(b"abc\x00def")

    assert library.calls[-1] == (
        "send",
        2**40 + 7,
        b"0211",
        b"abc\x00def",
        7,
        12,
        34,
    )


def test_close_is_idempotent_and_disables_enqueue_before_destroy():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)
    client.start()
    callback = library.callback

    client.close()
    client.close()
    buffer = ctypes.create_string_buffer(b"late", 4)
    callback(2**40 + 7, 1, b"0211", ctypes.cast(buffer, ctypes.c_void_p), 4, 0, 0)

    assert [call for call in library.calls if call[0] == "destroy"] == [
        ("destroy", 2**40 + 7)
    ]
    assert client.receive() is None


def test_start_destroys_created_handle_when_callback_setup_raises():
    library = FakeLibrary()

    def fail_callback_setup(_handle, _callback):
        raise OSError("callback setup failed")

    library.LaDDS_SetRecvCallback = FakeFunction(fail_callback_setup)
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)

    with pytest.raises(LaDDSError, match="LaDDS_SetRecvCallback"):
        client.start()

    assert library.calls[-1] == ("destroy", 2**40 + 7)


@pytest.mark.skipif(os.name != "nt", reason="os.add_dll_directory is Windows-only")
def test_close_releases_dependency_search_directory_after_native_handle(
    tmp_path, monkeypatch
):
    library = FakeLibrary()

    class DirectoryHandle:
        def close(self):
            library.calls.append(("dll_dir_close",))

    added_directories = []
    monkeypatch.setattr(
        os,
        "add_dll_directory",
        lambda path: (added_directories.append(path), DirectoryHandle())[1],
    )
    dll_path = tmp_path / "LaDDS.dll"
    client = LaDDSClient(str(dll_path), library_loader=lambda _path: library)

    client.start()
    client.close()

    assert added_directories == [str(tmp_path)]
    assert library.calls[-2:] == [
        ("destroy", 2**40 + 7),
        ("dll_dir_close",),
    ]


def test_default_transport_configuration_is_topic_0211_best_effort():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)

    client.start()

    assert ("pub", 2**40 + 7, b"0211", 2) in library.calls
    assert ("sub", 2**40 + 7, b"0211", 2) in library.calls


def test_receive_queue_keeps_oldest_packet_and_counts_new_packet_when_full():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so", queue_depth=1, library_loader=lambda _path: library
    )
    client.start()

    library.emit(b"old")
    library.emit(b"new")

    assert client.receive().payload == b"old"
    assert client.receive() is None
    assert client.dropped_messages == 1


def test_callback_discards_oversized_native_packet_without_raising():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so", max_payload_bytes=4, library_loader=lambda _path: library
    )
    client.start()

    library.emit(b"12345")

    assert client.receive() is None
    assert client.dropped_messages == 1


def test_received_packet_is_immutable():
    packet = ReceivedPacket(1, "0211", b"data", 0, 0)

    with pytest.raises(FrozenInstanceError):
        packet.payload = b"changed"


@pytest.mark.parametrize("payload", [b"", bytearray(b"data"), "data"])
def test_send_rejects_empty_or_non_bytes_payload(payload):
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: FakeLibrary())

    with pytest.raises((TypeError, ValueError), match="payload"):
        client.send(payload)


def test_send_rejects_payload_over_configured_limit_before_native_call():
    library = FakeLibrary()
    client = LaDDSClient(
        "libLaDDS.so", max_payload_bytes=3, library_loader=lambda _path: library
    )
    client.start()

    with pytest.raises(ValueError, match="maximum"):
        client.send(b"four")

    assert not any(call[0] == "send" for call in library.calls)


def test_send_requires_started_client_and_reports_native_failure():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)

    with pytest.raises(LaDDSError, match="start"):
        client.send(b"data")

    client.start()
    library.LaDDS_SendEx = FakeFunction(lambda *_args: 0)
    with pytest.raises(LaDDSError, match="LaDDS_SendEx"):
        client.send(b"data")


@pytest.mark.parametrize(
    ("failed_function", "expected_operation"),
    [
        ("LaDDS_EnableEncryption", "LaDDS_EnableEncryption"),
        ("LaDDS_RegPubTopic", "LaDDS_RegPubTopic"),
        ("LaDDS_RegSubTopic", "LaDDS_RegSubTopic"),
    ],
)
def test_start_checks_every_status_return_and_destroys_handle(
    failed_function, expected_operation
):
    library = FakeLibrary()
    setattr(library, failed_function, FakeFunction(lambda *_args: 0))
    client = LaDDSClient(
        "libLaDDS.so", encryption=True, library_loader=lambda _path: library
    )

    with pytest.raises(LaDDSError, match=expected_operation):
        client.start()

    assert library.calls[-1] == ("destroy", 2**40 + 7)


def test_start_wraps_loader_failure_with_dependency_diagnostics():
    def failed_loader(_path):
        raise OSError("undefined symbol: SSL_CTX_free")

    client = LaDDSClient("libLaDDS.so", library_loader=failed_loader)

    with pytest.raises(LaDDSError, match="依赖库"):
        client.start()


def test_start_reports_zero_handle_with_dependency_and_authorization_hints():
    library = FakeLibrary(handle=0)
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)

    with pytest.raises(LaDDSError, match="依赖.*授权.*网络"):
        client.start()


def test_start_is_idempotent():
    library = FakeLibrary()
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)

    client.start()
    client.start()

    assert len([call for call in library.calls if call[0] == "create"]) == 1


def test_close_waits_for_in_flight_send_before_destroying_handle():
    library = FakeLibrary()
    send_entered = threading.Event()
    release_send = threading.Event()
    close_finished = threading.Event()

    def blocking_send(*_args):
        library.calls.append(("send_entered",))
        send_entered.set()
        assert release_send.wait(timeout=2.0)
        library.calls.append(("send_returned",))
        return 1

    library.LaDDS_SendEx = FakeFunction(blocking_send)
    client = LaDDSClient("libLaDDS.so", library_loader=lambda _path: library)
    client.start()
    send_thread = threading.Thread(target=client.send, args=(b"data",))
    close_thread = threading.Thread(
        target=lambda: (client.close(), close_finished.set())
    )

    send_thread.start()
    assert send_entered.wait(timeout=1.0)
    close_thread.start()
    assert not close_finished.wait(timeout=0.05)
    release_send.set()
    send_thread.join(timeout=1.0)
    close_thread.join(timeout=1.0)

    assert not send_thread.is_alive()
    assert not close_thread.is_alive()
    assert library.calls[-2:] == [
        ("send_returned",),
        ("destroy", 2**40 + 7),
    ]
