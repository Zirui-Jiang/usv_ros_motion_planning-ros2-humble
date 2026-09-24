"""0052 模块心跳：未启动或未就绪为 0，正常工作为 1。"""

import json
import math
from datetime import datetime

from usv_ladds_bridge.envelope import prepare_outbound_payload
from usv_ladds_bridge.module_status import (
    MODULE_CODES,
    ModuleStatusTracker,
    status_content,
)


WHEN = datetime(2026, 9, 1, 9, 48, 43, 950000)


def test_status_content_uses_service_state_and_module_code():
    assert status_content("perception", False) == {"state": 0, "code": "perception"}
    assert status_content("decision", True) == {"state": 1, "code": "decision"}
    assert status_content("control", True) == {"state": 1, "code": "control"}
    assert MODULE_CODES == ("perception", "decision", "control")


def test_unseen_modules_are_reported_as_not_started_after_the_timeout():
    tracker = ModuleStatusTracker(timeout=1.5)

    assert tracker.due(0.0) == []
    assert tracker.due(1.5) == [
        {"state": 0, "code": "perception"},
        {"state": 0, "code": "decision"},
        {"state": 0, "code": "control"},
    ]
    assert tracker.due(1.6) == []


def test_ready_false_stays_down_and_ready_true_is_sent_immediately():
    tracker = ModuleStatusTracker(period=10.0, timeout=1.5)

    assert tracker.observe("perception", False, 1.0) == {"state": 0, "code": "perception"}
    assert tracker.observe("perception", False, 1.1) is None
    assert tracker.observe("decision", True, 1.1) == {"state": 1, "code": "decision"}
    assert tracker.observe("decision", True, 1.2) is None
    assert tracker.due(1.2) == []
    assert tracker.due(2.5) == [{"state": 0, "code": "control"}]


def test_state_change_is_sent_without_waiting_for_the_period():
    tracker = ModuleStatusTracker(period=10.0, timeout=1.5)
    tracker.observe("control", True, 0.0)

    assert tracker.observe("control", False, 0.2) == {"state": 0, "code": "control"}
    assert tracker.due(0.4) == []
    assert tracker.due(1.5) == [
        {"state": 0, "code": "perception"},
        {"state": 0, "code": "decision"},
    ]


def test_unchanged_status_repeats_only_after_the_period():
    tracker = ModuleStatusTracker(period=1.0, timeout=1.5)
    tracker.observe("decision", True, 0.0)

    assert tracker.due(0.9) == []
    assert tracker.due(1.0) == [{"state": 1, "code": "decision"}]
    assert tracker.due(1.5) == [
        {"state": 0, "code": "perception"},
        {"state": 0, "code": "control"},
    ]


def test_stale_or_missing_heartbeat_returns_to_state_zero():
    tracker = ModuleStatusTracker(timeout=1.5)
    tracker.observe("perception", True, 10.0)

    assert tracker.due(11.5) == [
        {"state": 1, "code": "perception"},
        {"state": 0, "code": "decision"},
        {"state": 0, "code": "control"},
    ]
    assert tracker.due(11.6) == [{"state": 0, "code": "perception"}]


def test_unknown_module_and_non_finite_time_do_not_replace_a_live_status():
    tracker = ModuleStatusTracker(period=10.0)
    tracker.observe("decision", True, 1.0)

    assert tracker.observe("planner", True, 1.1) is None
    assert tracker.observe("decision", True, math.nan) is None
    assert tracker.due(math.inf) == []
    assert tracker.due(1.2) == []
    assert tracker.due(2.5) == [
        {"state": 0, "code": "perception"},
        {"state": 0, "code": "control"},
    ]


def test_module_status_is_wrapped_with_the_0051_head():
    payload, sequence = prepare_outbound_payload(
        json.dumps(status_content("perception", False), separators=(",", ":")).encode("utf-8"),
        "0051",
        2,
        now=WHEN,
    )

    assert sequence == 3
    assert json.loads(payload) == {
        "head": {
            "packageSeq": 2,
            "packageType": 0,
            "recvPlatform": 1001,
            "sendPlatform": 1000,
            "time": "2026-09-01 09:48:43.950",
        },
        "content": {"state": 0, "code": "perception"},
    }
