"""Small ctypes adapter for the vendor LaDDS C SDK."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
import queue
import threading
from typing import Callable, Optional


class LaDDSError(RuntimeError):
    """Raised when the LaDDS SDK cannot complete an operation."""


@dataclass(frozen=True)
class ReceivedPacket:
    domain_id: int
    topic: str
    payload: bytes
    src: int
    dst: int


class LaDDSClient:
    """Lifecycle wrapper around one LaDDS native handle."""

    def __init__(
        self,
        library_path: str,
        domain_id: int = 1,
        bind_ip: str = "",
        ttl: int = 1,
        qos: str = "best_effort",
        send_topic: str = "0211",
        receive_topic: str = "0211",
        src: int = 0,
        dst: int = 0,
        max_payload_bytes: int = 1_048_576,
        queue_depth: int = 100,
        encryption: bool = False,
        library_loader: Optional[Callable[[str], object]] = None,
    ) -> None:
        _require_c_string("library_path", library_path, allow_empty=False)
        _require_integer("domain_id", domain_id, 0, 2**31 - 1)
        _require_c_string("bind_ip", bind_ip, allow_empty=True)
        _require_integer("ttl", ttl, 0, 255)
        if qos not in {"reliable", "best_effort"}:
            raise ValueError("qos must be 'reliable' or 'best_effort'")
        _require_c_string("send_topic", send_topic, allow_empty=False)
        _require_c_string("receive_topic", receive_topic, allow_empty=False)
        _require_integer("src", src, 0, 2**32 - 1)
        _require_integer("dst", dst, 0, 2**32 - 1)
        _require_integer("max_payload_bytes", max_payload_bytes, 1, 2**31 - 1)
        _require_integer("queue_depth", queue_depth, 1, 2**31 - 1)
        if not isinstance(encryption, bool):
            raise TypeError("encryption must be a bool")
        if library_loader is not None and not callable(library_loader):
            raise TypeError("library_loader must be callable")

        self._library_path = library_path
        self._library_loader = library_loader or ctypes.CDLL
        self._domain_id = domain_id
        self._bind_ip = bind_ip.encode("utf-8") if bind_ip else None
        self._ttl = ttl
        self._qos = {"reliable": 1, "best_effort": 2}[qos]
        self._send_topic = send_topic.encode("utf-8")
        self._receive_topic = receive_topic.encode("utf-8")
        self._src = src
        self._dst = dst
        self._max_payload_bytes = max_payload_bytes
        self._queue_depth = queue_depth
        self._encryption = encryption
        self._version: Optional[str] = None
        self._dropped_messages = 0
        self._library = None
        self._handle = 0
        self._callback = None
        self._dll_directory_handle = None
        self._packets: queue.Queue[ReceivedPacket] = queue.Queue(maxsize=queue_depth)
        self._operation_lock = threading.RLock()
        self._callback_lock = threading.Lock()
        self._accept_callbacks = False

    @property
    def version(self) -> Optional[str]:
        return self._version

    @property
    def dropped_messages(self) -> int:
        return self._dropped_messages

    def receive(self) -> Optional[ReceivedPacket]:
        try:
            return self._packets.get_nowait()
        except queue.Empty:
            return None

    def start(self) -> None:
        with self._operation_lock:
            self._start_locked()

    def _start_locked(self) -> None:
        if self._handle:
            return

        self._open_dll_directory()
        try:
            library = self._library_loader(self._library_path)
        except Exception as exc:
            self._release_dll_directory()
            raise LaDDSError(
                f"无法加载 LaDDS 动态库 {self._library_path!r}: {exc}. "
                "请检查文件路径、位数和依赖库"
            ) from exc

        try:
            self._configure_abi(library)
            version = library.LaDDS_GetVersion()
        except Exception as exc:
            self._release_dll_directory()
            raise LaDDSError(
                f"LaDDS SDK 符号或版本读取失败: {exc}; 请核对 SDK 版本和依赖库"
            ) from exc
        if not version:
            self._release_dll_directory()
            raise LaDDSError("LaDDS_GetVersion 返回空指针；请核对 SDK 动态库")

        handle = 0
        callback = _RECV_CALLBACK(self._on_receive)
        try:
            try:
                handle = library.LaDDS_Create(self._domain_id, self._ttl, self._bind_ip)
            except Exception as exc:
                raise LaDDSError(f"LaDDS_Create 调用失败: {exc}") from exc
            if not handle:
                raise LaDDSError(
                    "LaDDS_Create 返回空句柄；请检查动态库依赖、运行授权与网络配置"
                )

            try:
                library.LaDDS_SetRecvCallback(handle, callback)
            except Exception as exc:
                raise LaDDSError(f"LaDDS_SetRecvCallback 调用失败: {exc}") from exc
            if self._encryption and not library.LaDDS_EnableEncryption(handle, 1):
                raise LaDDSError(
                    "LaDDS_EnableEncryption 失败；请检查 SDK 加密支持与运行授权"
                )
            if not library.LaDDS_RegPubTopic(handle, self._send_topic, self._qos):
                raise LaDDSError(
                    "LaDDS_RegPubTopic 失败；请检查发布主题、QoS 与运行授权"
                )
            if not library.LaDDS_RegSubTopic(handle, self._receive_topic, self._qos):
                raise LaDDSError(
                    "LaDDS_RegSubTopic 失败；请检查订阅主题、QoS 与运行授权"
                )
        except Exception as exc:
            if handle:
                try:
                    library.LaDDS_Destroy(handle)
                except Exception:
                    pass
            self._release_dll_directory()
            if isinstance(exc, LaDDSError):
                raise
            raise LaDDSError(f"LaDDS 启动失败: {exc}") from exc

        self._library = library
        self._handle = handle
        self._callback = callback
        self._version = version.decode("utf-8", errors="replace") if version else ""
        with self._callback_lock:
            self._accept_callbacks = True

    def send(self, payload: bytes) -> None:
        if not isinstance(payload, bytes):
            raise TypeError("payload must be bytes")
        if not payload:
            raise ValueError("payload must not be empty")
        if len(payload) > self._max_payload_bytes:
            raise ValueError(
                f"payload is {len(payload)} bytes; maximum is {self._max_payload_bytes}"
            )

        with self._operation_lock:
            if not self._handle or self._library is None:
                raise LaDDSError("LaDDSClient 尚未启动；请先调用 start()")
            buffer = ctypes.create_string_buffer(payload, len(payload))
            result = self._library.LaDDS_SendEx(
                self._handle,
                self._send_topic,
                ctypes.cast(buffer, ctypes.c_void_p),
                len(payload),
                self._src,
                self._dst,
            )
            if not result:
                raise LaDDSError("LaDDS_SendEx 失败；请检查主题注册、网络与授权状态")

    def close(self) -> None:
        with self._operation_lock:
            if not self._handle:
                return

            with self._callback_lock:
                self._accept_callbacks = False

            handle = self._handle
            library = self._library
            try:
                if library is not None:
                    library.LaDDS_Destroy(handle)
            except Exception as exc:
                raise LaDDSError(f"LaDDS_Destroy 调用失败: {exc}") from exc
            finally:
                self._handle = 0
                self._library = None
                # Keep the ctypes callback alive until Destroy has returned.
                self._callback = None
                self._release_dll_directory()

    def _open_dll_directory(self) -> None:
        if os.name != "nt" or not hasattr(os, "add_dll_directory"):
            return
        configured_directory = os.path.dirname(self._library_path)
        if not configured_directory:
            return
        directory = os.path.abspath(configured_directory)
        try:
            self._dll_directory_handle = os.add_dll_directory(directory)
        except OSError as exc:
            raise LaDDSError(f"无法加入 DLL 依赖目录 {directory!r}: {exc}") from exc

    def _release_dll_directory(self) -> None:
        handle = self._dll_directory_handle
        self._dll_directory_handle = None
        if handle is not None:
            handle.close()

    def _on_receive(
        self,
        _handle: int,
        domain_id: int,
        topic: bytes,
        buffer: int,
        size: int,
        src: int,
        dst: int,
    ) -> None:
        try:
            if size < 0 or size > self._max_payload_bytes or (size and not buffer):
                self._record_drop()
                return
            payload = ctypes.string_at(buffer, size) if size else b""
            packet = ReceivedPacket(
                domain_id=domain_id,
                topic=topic.decode("utf-8", errors="replace") if topic else "",
                payload=payload,
                src=src,
                dst=dst,
            )
            with self._callback_lock:
                if not self._accept_callbacks:
                    return
                try:
                    self._packets.put_nowait(packet)
                except queue.Full:
                    self._dropped_messages += 1
        except BaseException:
            # ctypes reports callback exceptions to stderr and otherwise loses them.
            # The callback boundary therefore absorbs malformed native input.
            self._record_drop()

    def _record_drop(self) -> None:
        with self._callback_lock:
            self._dropped_messages += 1

    @staticmethod
    def _configure_abi(library: object) -> None:
        library.LaDDS_GetVersion.argtypes = []
        library.LaDDS_GetVersion.restype = ctypes.c_char_p
        library.LaDDS_Create.argtypes = [ctypes.c_int, ctypes.c_uint8, ctypes.c_char_p]
        library.LaDDS_Create.restype = ctypes.c_uint64
        library.LaDDS_Destroy.argtypes = [ctypes.c_uint64]
        library.LaDDS_Destroy.restype = None
        library.LaDDS_EnableEncryption.argtypes = [ctypes.c_uint64, ctypes.c_int]
        library.LaDDS_EnableEncryption.restype = ctypes.c_int
        topic_args = [ctypes.c_uint64, ctypes.c_char_p, ctypes.c_int]
        library.LaDDS_RegPubTopic.argtypes = topic_args
        library.LaDDS_RegPubTopic.restype = ctypes.c_int
        library.LaDDS_RegSubTopic.argtypes = topic_args
        library.LaDDS_RegSubTopic.restype = ctypes.c_int
        library.LaDDS_SetRecvCallback.argtypes = [ctypes.c_uint64, _RECV_CALLBACK]
        library.LaDDS_SetRecvCallback.restype = None
        library.LaDDS_SendEx.argtypes = [
            ctypes.c_uint64,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_uint32,
            ctypes.c_uint32,
        ]
        library.LaDDS_SendEx.restype = ctypes.c_int


_RECV_CALLBACK = ctypes.CFUNCTYPE(
    None,
    ctypes.c_uint64,
    ctypes.c_int,
    ctypes.c_char_p,
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_uint32,
    ctypes.c_uint32,
)


def _require_integer(name: str, value: int, minimum: int, maximum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")


def _require_c_string(name: str, value: str, *, allow_empty: bool) -> None:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string")
    if not allow_empty and not value:
        raise ValueError(f"{name} must not be empty")
    if "\x00" in value:
        raise ValueError(f"{name} must not contain NUL")
