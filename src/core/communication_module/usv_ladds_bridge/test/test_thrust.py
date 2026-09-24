"""0054 推力内容编码与看门狗策略：新鲜命令立即转发，断流或尚未收到时发零推力。"""

import json
import math
from datetime import datetime
from types import SimpleNamespace

import pytest

from usv_ladds_bridge.envelope import ENVELOPE_TOPICS, prepare_outbound_payload
from usv_ladds_bridge.thrust import (
    ThrustForwarder,
    thrust_content_from_command,
    zero_thrust_content,
)


def command(**overrides):
    values = dict(mode=0, left_angle=0.0, left_rpm=283.4, right_angle=0.0, right_rpm=-12.5)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_content_keeps_field_names_units_and_signs():
    assert thrust_content_from_command(command()) == {
        "mode": 0,
        "left_angle": 0.0,
        "left_rpm": 283.4,
        "right_angle": 0.0,
        "right_rpm": -12.5,
    }
    assert zero_thrust_content() == {
        "mode": 0, "left_angle": 0.0, "left_rpm": 0.0, "right_angle": 0.0, "right_rpm": 0.0,
    }


@pytest.mark.parametrize("overrides", [
    {"mode": 2}, {"mode": True}, {"mode": 0.0},
    {"left_rpm": math.nan}, {"right_angle": math.inf},
])
def test_invalid_commands_are_rejected(overrides):
    with pytest.raises(ValueError):
        thrust_content_from_command(command(**overrides))


# 尚未收到命令即视为断流：看门狗从启动起就发零推力。
def test_watchdog_sends_zero_before_the_first_command():
    forwarder = ThrustForwarder(0.5)
    assert forwarder.stale
    assert forwarder.watchdog(10.0) == zero_thrust_content()


# 0.5 s 内有命令时看门狗不补发；超过 0.5 s 后每次检查都发零推力，直到命令恢复。
def test_watchdog_zeroes_only_after_the_timeout():
    forwarder = ThrustForwarder(0.5)
    assert forwarder.on_command(command(), 10.0)["left_rpm"] == 283.4
    assert not forwarder.stale
    assert forwarder.watchdog(10.5) is None
    assert forwarder.watchdog(10.51) == zero_thrust_content()
    assert forwarder.stale
    assert forwarder.watchdog(10.61) == zero_thrust_content()
    forwarder.on_command(command(), 10.7)
    assert forwarder.watchdog(10.8) is None


# 无效命令不刷新接收时间，持续无效时看门狗照样接管。
def test_invalid_command_does_not_refresh_the_stream():
    forwarder = ThrustForwarder(0.5)
    forwarder.on_command(command(), 10.0)
    with pytest.raises(ValueError):
        forwarder.on_command(command(left_rpm=math.nan), 10.4)
    assert forwarder.watchdog(10.6) == zero_thrust_content()


@pytest.mark.parametrize("timeout", [0.0, -1.0, math.nan, math.inf])
def test_timeout_must_be_finite_and_positive(timeout):
    with pytest.raises(ValueError):
        ThrustForwarder(timeout)


# 0054 由配置并入 head 集合后，与 0052 等使用同一外壳和递增序号。
def test_configured_thrust_topic_gets_the_same_head():
    when = datetime(2026, 9, 24, 10, 15, 3, 250000)
    body = json.dumps(zero_thrust_content()).encode("utf-8")
    bare, seq = prepare_outbound_payload(body, "0054", 7, now=when)
    assert bare is body and seq == 7
    wrapped, seq = prepare_outbound_payload(
        body, "0054", 7, now=when, envelope_topics=ENVELOPE_TOPICS | {"0054"}
    )
    root = json.loads(wrapped)
    assert seq == 8
    assert root["content"] == zero_thrust_content()
    assert root["head"] == {
        "packageSeq": 7,
        "packageType": 0,
        "recvPlatform": 1001,
        "sendPlatform": 1000,
        "time": "2026-09-24 10:15:03.250",
    }
