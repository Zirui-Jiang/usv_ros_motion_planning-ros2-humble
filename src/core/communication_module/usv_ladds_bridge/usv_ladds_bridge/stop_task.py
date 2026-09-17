"""0331 stop command validation; routing uses DDS topic, never JSON guessing."""

import json
from dataclasses import dataclass, field
from uuid import uuid4


@dataclass
class StopExecution:
    """One idempotent stop, kept separate from ordinary voyage completions."""

    superseded_request_ids: tuple = ()
    request_id: str = field(default_factory=lambda: 'ladds-stop-' + uuid4().hex)
    planner_session: str = ''
    task_id: int = -1
    accepted: bool = False
    stopped: bool = False
    attempts: int = 0
    state: str = 'STOP_PENDING'


def parse_stop(text: str) -> tuple:
    """Accept an empty object, optionally wrapped, and report legacy fields.

    No coordinate, heading or duration influences active braking. Unknown
    business fields are rejected to prevent a misrouted voyage becoming a stop.
    """
    try:
        root = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'invalid 0331 JSON: {exc}') from exc
    if type(root) is not dict:
        raise ValueError('0331 root must be an object')
    content = root
    if 'content' in root:
        if type(root.get('head')) is not dict:
            raise ValueError('wrapped 0331 requires object head and content')
        content = root['content']
    if type(content) is not dict:
        raise ValueError('0331 content must be an object')
    legacy_fields = {'longitude', 'latitude', 'heading', 'duration'}
    if set(content) - legacy_fields:
        raise ValueError('0331 accepts empty content or legacy positioning fields only')
    if content:
        return ('0331 已忽略旧经纬度、艏向和时长字段；仅主动刹停，停稳后不保持原地',)
    return ()
