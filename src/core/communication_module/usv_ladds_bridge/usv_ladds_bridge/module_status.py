# 把感知、决策、航控的存活编成 0051 业务对象，外壳仍由发布节点套上。
"""Module liveness as a 0051 content object; the outbound node adds the head."""

from __future__ import annotations

import math
from typing import Optional


# 与导航心跳角色名一致；0 表示未启动或未正常工作，1 表示已启动且正常工作。
MODULE_CODES = ("perception", "decision", "control")
STATUS_PERIOD = 1.0
STATUS_TIMEOUT = 1.5
STATUS_CHECK_PERIOD = 0.2


def status_content(code: str, working: bool) -> dict:
    """Return the bare 0052 content object for one module."""
    if code not in MODULE_CODES:
        raise ValueError(f"module code must be one of {MODULE_CODES}")
    if not isinstance(working, bool):
        raise TypeError("working must be a bool")
    return {"state": 1 if working else 0, "code": code}


# 按模块记住最近一次 ROS 心跳，超时或未出现都报 state 0，状态变化立即发出。
class ModuleStatusTracker:
    def __init__(
        self,
        period: float = STATUS_PERIOD,
        timeout: float = STATUS_TIMEOUT,
    ) -> None:
        if period <= 0.0 or timeout <= 0.0:
            raise ValueError("status period and timeout must be greater than zero")
        self.period = period
        self.timeout = timeout
        self._started_at: Optional[float] = None
        self._seen: dict[str, tuple[bool, float]] = {}
        self._sent: dict[str, tuple[int, float]] = {}

    def observe(self, code: str, ready: bool, now: float) -> Optional[dict]:
        """Record one heartbeat and return content when its reported state changes."""
        if code not in MODULE_CODES or not isinstance(ready, bool) or not math.isfinite(now):
            return None
        self._arm(now)
        self._seen[code] = (ready, now)
        return self._emit_if_due(code, now, force_change=True)

    def due(self, now: float) -> list[dict]:
        """Return each module whose periodic status is due, including modules not yet seen."""
        if not math.isfinite(now):
            return []
        self._arm(now)
        payloads = []
        for code in MODULE_CODES:
            content = self._emit_if_due(code, now, force_change=False)
            if content is not None:
                payloads.append(content)
        return payloads

    def _arm(self, now: float) -> None:
        if self._started_at is None:
            self._started_at = now

    def _working(self, code: str, now: float) -> bool:
        seen = self._seen.get(code)
        if seen is None:
            return False
        ready, seen_at = seen
        if (
            not math.isfinite(now)
            or not math.isfinite(seen_at)
            or now < seen_at
            or now - seen_at > self.timeout
        ):
            return False
        return ready

    def _emit_if_due(self, code: str, now: float, *, force_change: bool) -> Optional[dict]:
        if code not in self._seen:
            # 未见过的模块先留一个超时窗口，避免桥启动瞬间把已在运行的模块误报为 0。
            if self._started_at is None or now - self._started_at < self.timeout:
                return None
            state = 0
        else:
            state = 1 if self._working(code, now) else 0
        previous = self._sent.get(code)
        if previous is not None:
            last_state, sent_at = previous
            if last_state == state and (force_change or now - sent_at < self.period):
                return None
        self._sent[code] = (state, now)
        return status_content(code, state == 1)
