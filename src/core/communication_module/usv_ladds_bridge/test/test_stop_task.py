import pytest

from usv_ladds_bridge.stop_task import parse_stop


@pytest.mark.parametrize('text', ['{}', '{"head":{},"content":{}}'])
def test_empty_body_stop_requires_no_navigation_fields(text):
    assert parse_stop(text) == ()


def test_legacy_position_fields_are_not_used_as_stop_target():
    assert '忽略' in parse_stop('{"longitude":114,"latitude":30,"heading":0,"duration":1}')[0]


@pytest.mark.parametrize('text', ['', 'null', '[]', '{"content":{}}',
    '{"head":{},"content":null}', '{"taskType":16}', '{"stop":true}'])
def test_malformed_or_other_protocol_body_is_not_a_stop(text):
    with pytest.raises(ValueError):
        parse_stop(text)
