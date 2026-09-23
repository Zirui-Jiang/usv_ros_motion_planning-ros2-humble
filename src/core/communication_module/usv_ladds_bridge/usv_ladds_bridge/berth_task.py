"""0212 靠离泊解析：本阶段只取一个导航点，当成普通单点航路。"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any

from common.geodesy import GeoReference

from .route_task import RouteTask, RouteWaypoint, _required_point
from .single_leg import (
    KNOT_TO_METRES_PER_SECOND,
    _required,
    _require_exact_integer,
    _require_finite_number,
)


_IGNORED_BERTH_FIELDS = (
    'heading',
    'berthHeading',
    'berthType',
    'berthingType',
    'dockId',
    'berthId',
    'pierId',
    'berthMode',
    'mode',
    'areas',
    'areasCount',
    'guardingHeading',
    'guardingRadius',
)


# 0212 单点靠离泊。联调报文用 outBerthLat/outBerthLon 和 berthSpeed 组成一个航路点；
# 没有这两个坐标时，仍接受旧的 points 单点。taskType 1/3/4 都按普通单点航路执行。
def parse_berth(text: str, origin: GeoReference) -> RouteTask:
    """Parse one 0212 waypoint as an ordinary single-point RouteTask."""
    if not isinstance(text, str):
        raise ValueError("berth task must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid berth JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("berth JSON root must be an object")

    wrapped = "content" in root
    if wrapped:
        if "head" not in root or type(root["head"]) is not dict:
            raise ValueError("wrapped task must contain object head and content")
        content = root["content"]
        head = root["head"]
    else:
        content = root
        head = None
    if type(content) is not dict:
        raise ValueError("task content must be an object")

    sender = head.get("sendPlatform") if head is not None else None
    if sender is not None:
        _require_exact_integer("head.sendPlatform", sender)

    travel_id = _required(content, "travelId")
    _require_exact_integer("travelId", travel_id)
    task_type = _required(content, "taskType")
    _require_exact_integer("taskType", task_type)

    if "outBerthLat" in content or "outBerthLon" in content:
        if task_type not in (1, 3, 4):
            raise ValueError("0212 taskType must be 1, 3, or 4")
        latitude = _require_finite_number(
            "outBerthLat", _required(content, "outBerthLat")
        )
        longitude = _require_finite_number(
            "outBerthLon", _required(content, "outBerthLon")
        )
        speed_knots = _require_finite_number(
            "berthSpeed", _required(content, "berthSpeed")
        )
        point_type = 1
        lat_name, lon_name, speed_name = "outBerthLat", "outBerthLon", "berthSpeed"
    else:
        if task_type not in (3, 4):
            raise ValueError("0212 taskType must be 3 (departure) or 4 (berthing)")
        latitude, longitude, speed_knots, point_type = _legacy_single_point(content)
        lat_name, lon_name, speed_name = (
            "points[0].latitude",
            "points[0].longitude",
            "points[0].speed",
        )
    if not -90.0 <= latitude <= 90.0:
        raise ValueError(f"{lat_name} must be in [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError(f"{lon_name} must be in [-180, 180]")
    if speed_knots <= 0.0:
        raise ValueError(f"{speed_name} must be positive")

    try:
        x, y, _ = origin.to_local(latitude, longitude)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot convert points[0] to ENU: {exc}") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("cannot convert points[0] to finite ENU coordinates")

    warnings = list(_unused_berth_field_warnings(content))

    summary = {
        "point": {
            "latitude": latitude,
            "longitude": longitude,
            "speed": speed_knots,
            "type": point_type,
        },
        "protocol": "0212",
        "taskType": task_type,
        "travelId": travel_id,
    }
    canonical = json.dumps(
        summary, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return RouteTask(
        travel_id=travel_id,
        sender=sender,
        points=(RouteWaypoint(
            x=x,
            y=y,
            speed_knots=speed_knots,
            speed_mps=speed_knots * KNOT_TO_METRES_PER_SECOND,
        ),),
        fingerprint=hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
        wrapped=wrapped,
        task_type=task_type,
        protocol="0212",
        task_finish=3,
        warnings=tuple(warnings),
    )


def _legacy_single_point(content: dict[str, Any]) -> tuple[float, float, float, int]:
    """旧 0212：count=1 且 points 里只有一个 type=1 的导航点。"""
    count = _required(content, "count")
    _require_exact_integer("count", count)
    if count != 1:
        raise ValueError("0212 count must be exactly 1")
    point_objects = _required(content, "points")
    if type(point_objects) is not list:
        raise ValueError("points must be a list")
    if len(point_objects) != 1:
        raise ValueError("0212 points must contain exactly one point")

    point = point_objects[0]
    if type(point) is not dict:
        raise ValueError("points[0] must be an object")
    point_type = _required_point(point, "type", "points[0]")
    _require_exact_integer("points[0].type", point_type)
    if point_type != 1:
        raise ValueError("points[0].type must be 1")
    latitude = _require_finite_number(
        "points[0].latitude", _required_point(point, "latitude", "points[0]")
    )
    longitude = _require_finite_number(
        "points[0].longitude", _required_point(point, "longitude", "points[0]")
    )
    speed_knots = _require_finite_number(
        "points[0].speed", _required_point(point, "speed", "points[0]")
    )
    return latitude, longitude, speed_knots, point_type


def _unused_berth_field_warnings(content: dict[str, Any]) -> tuple[str, ...]:
    warnings = []
    for name in _IGNORED_BERTH_FIELDS:
        if name not in content:
            continue
        value = content[name]
        if _is_absent_constraint(value):
            continue
        warnings.append(
            f'未应用靠泊字段 {name}：本版本将 0212 作为单点航路执行，不做真正靠离泊'
        )
    return tuple(warnings)


def _is_absent_constraint(value: object) -> bool:
    if value is None:
        return True
    if type(value) is int and value == 0:
        return True
    if type(value) is list and not value:
        return True
    return False
