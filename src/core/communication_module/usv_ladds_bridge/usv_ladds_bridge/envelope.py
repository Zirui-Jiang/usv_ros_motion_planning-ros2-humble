"""给 0141 / 0411 / 0052 的发出报文套上联调要求的 head 与 content。"""

from __future__ import annotations

import json
from datetime import datetime
from typing import Optional


# 与联调样本一致：本侧 sendPlatform=1000，对端 recvPlatform=1001，packageType=0。
SEND_PLATFORM = 1000
RECV_PLATFORM = 1001
PACKAGE_TYPE = 0
ENVELOPE_TOPICS = frozenset({"0141", "0411", "0052"})


def prepare_outbound_payload(
    payload: bytes,
    topic: str,
    package_seq: int,
    now: Optional[datetime] = None,
) -> tuple[bytes, int]:
    """0141、0411、0052 的 JSON 对象在发出前套外壳；其它主题原样转发。

    已经同时带有对象 ``head`` 和 ``content`` 的报文不再套一层，序号也不前进。
    """
    if topic not in ENVELOPE_TOPICS:
        return payload, package_seq
    if type(package_seq) is not int or package_seq < 1:
        raise ValueError("packageSeq must be a positive integer")
    wrapped = _wrap(payload, package_seq, datetime.now() if now is None else now)
    if wrapped is payload:
        return payload, package_seq
    return wrapped, package_seq + 1


def _wrap(payload: bytes, package_seq: int, moment: datetime) -> bytes:
    if not isinstance(payload, bytes) or not payload:
        raise ValueError("outbound payload must be non-empty bytes")
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("outbound payload must be UTF-8 JSON") from exc
    try:
        root = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"outbound payload is not JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("outbound JSON root must be an object")
    content = root.get("content")
    head = root.get("head")
    if type(content) is dict and type(head) is dict:
        return payload
    envelope = {
        "head": {
            "packageSeq": package_seq,
            "packageType": PACKAGE_TYPE,
            "recvPlatform": RECV_PLATFORM,
            "sendPlatform": SEND_PLATFORM,
            "time": _format_time(moment),
        },
        "content": root,
    }
    return json.dumps(envelope, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _format_time(moment: datetime) -> str:
    """联调时间格式：``2026-09-01 09:48:43.950``。"""
    return moment.strftime("%Y-%m-%d %H:%M:%S.") + f"{moment.microsecond // 1000:03d}"
