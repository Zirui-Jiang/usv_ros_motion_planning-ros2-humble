"""航控桨角/转速命令编成东湖 0054 报文内容，并决定转发最新命令还是由看门狗发零推力。"""

from __future__ import annotations

import math
from typing import Any, Optional


# 0054 暂定内容：字段名沿用航控接口，桨角单位弧度（0 指向船艏），转速单位 rpm（带符号）。
THRUST_FIELDS = ("left_angle", "left_rpm", "right_angle", "right_rpm")
# 看门狗检查及断流后发零推力的周期，秒；与航控 10 Hz 输出一致。
WATCHDOG_PERIOD = 0.1


def zero_thrust_content() -> dict[str, Any]:
    """Zero rpm with forward-pointing thrusters."""
    return {"mode": 0, **{name: 0.0 for name in THRUST_FIELDS}}


def thrust_content_from_command(message: Any) -> dict[str, Any]:
    """把 ``ThrusterCommand`` 转成 0054 content；非有限数或未知 mode 直接拒绝。"""
    mode = message.mode
    if isinstance(mode, bool) or not isinstance(mode, int) or mode not in (0, 1):
        raise ValueError("thrust mode must be 0 or 1")
    content: dict[str, Any] = {"mode": mode}
    for name in THRUST_FIELDS:
        value = float(getattr(message, name))
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        content[name] = value
    return content


class ThrustForwarder:
    """Forward fresh commands immediately; send zero while the stream is silent.

    Times are local monotonic seconds at reception. Message stamps are not
    compared because the control node may run on simulation time.
    """

    def __init__(self, timeout: float) -> None:
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("thrust timeout must be finite and positive")
        self._timeout = timeout
        self._last_command_time: Optional[float] = None
        # 启动时尚未收到航控命令，按断流处理：看门狗先发零推力。
        self.stale = True

    # 有效命令刷新接收时间并返回待发内容；无效命令抛出，由调用方改发零推力。
    def on_command(self, message: Any, now: float) -> dict[str, Any]:
        content = thrust_content_from_command(message)
        self._last_command_time = now
        self.stale = False
        return content

    # 超过 timeout 未收到有效命令（含从未收到）时返回零推力，否则返回 None 表示无需补发。
    def watchdog(self, now: float) -> Optional[dict[str, Any]]:
        last = self._last_command_time
        if last is not None and now - last <= self._timeout:
            return None
        self.stale = True
        return zero_thrust_content()
