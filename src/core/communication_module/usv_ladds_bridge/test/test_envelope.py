"""0141 / 0411 / 0052 发出前的 head 与 content 外壳。"""

import json
from datetime import datetime

import pytest

from usv_ladds_bridge.envelope import prepare_outbound_payload


WHEN = datetime(2026, 9, 1, 9, 48, 43, 950000)


def test_0411_0141_and_0052_are_wrapped_with_the_same_head():
    cases = {
        "0411": {"longitude": 114.7, "latitude": 22.1, "speed": 8, "heading": 16, "angularZ": 26},
        "0141": {"count": 0, "targets": []},
        "0052": {"travelId": 53, "taskFinish": 1},
    }
    seq = 1
    for topic, business in cases.items():
        payload, seq = prepare_outbound_payload(
            json.dumps(business).encode("utf-8"), topic, seq, now=WHEN
        )
        root = json.loads(payload)
        assert root["content"] == business
        assert root["head"] == {
            "packageSeq": seq - 1,
            "packageType": 0,
            "recvPlatform": 1001,
            "sendPlatform": 1000,
            "time": "2026-09-01 09:48:43.950",
        }
    assert seq == 4


def test_other_topics_and_already_wrapped_payloads_pass_through():
    bare = b'{"travelId":53,"taskFinish":1}'
    unchanged, seq = prepare_outbound_payload(bare, "0211", 3, now=WHEN)
    assert unchanged is bare
    assert seq == 3

    wrapped = json.dumps(
        {"head": {"sendPlatform": 1000}, "content": {"travelId": 53}}
    ).encode("utf-8")
    again, seq = prepare_outbound_payload(wrapped, "0052", 3, now=WHEN)
    assert again is wrapped
    assert seq == 3


@pytest.mark.parametrize(
    "payload",
    [b"", b"\xff", b"[]", b"null", b'"text"'],
)
def test_envelope_topics_reject_payloads_that_are_not_json_objects(payload):
    with pytest.raises(ValueError):
        prepare_outbound_payload(payload, "0141", 1, now=WHEN)
