"""Pure parsing for an ordered LaDDS multi-waypoint route."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Optional, Tuple

from common.geodesy import GeoReference, heading_to_yaw

from .single_leg import (
    KNOT_TO_METRES_PER_SECOND,
    _required,
    _require_exact_integer,
    _require_finite_number,
)


@dataclass(frozen=True)
class RouteWaypoint:
    """One validated required point in ENU and its inbound segment speed."""

    x: float
    y: float
    speed_knots: float
    speed_mps: float


@dataclass(frozen=True)
class RouteTask:
    """One immutable, ordered route received in a single 0211 packet."""

    travel_id: int
    sender: Optional[int]
    points: Tuple[RouteWaypoint, ...]
    fingerprint: str
    wrapped: bool
    task_type: int = 16
    guard_heading: Optional[float] = None
    warnings: Tuple[str, ...] = ()

    @property
    def guard_on_arrival(self) -> bool:
        return self.task_type == 80

    @property
    def identity(self) -> Tuple[Optional[int], int]:
        """Route identity only; each received command gets its own execution ID."""
        return (self.sender, self.travel_id)

    # LegBook and status formatting historically consume a single target through
    # these properties. Keep that boundary compatible while the record holds the
    # full immutable route in ``points``.
    @property
    def x(self) -> float:
        return self.points[0].x

    @property
    def y(self) -> float:
        return self.points[0].y

    @property
    def speed_knots(self) -> float:
        return self.points[0].speed_knots

    @property
    def speed_mps(self) -> float:
        return self.points[0].speed_mps


def parse_route(text: str, origin: GeoReference) -> RouteTask:
    """Parse and atomically validate all required points in one 0211 route."""
    if not isinstance(text, str):
        raise ValueError("route task must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid route JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("route JSON root must be an object")

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
    if task_type not in (16, 80):
        raise ValueError("taskType must be 16 (route) or 80 (route then guard)")

    count = _required(content, "count")
    _require_exact_integer("count", count)
    if not 1 <= count <= 255:
        raise ValueError("count must be in [1, 255]")
    point_objects = _required(content, "points")
    if type(point_objects) is not list:
        raise ValueError("points must be a list")
    if len(point_objects) != count:
        raise ValueError("count must match the number of points")

    guard_heading = None
    warnings = []
    if task_type == 16:
        _validate_optional_constraints(content)
    else:
        heading = content.get('guardingHeading')
        if heading is not None:
            heading = _require_finite_number('guardingHeading', heading)
            if not 0 <= heading <= 360:
                raise ValueError('guardingHeading must be true-north clockwise degrees in [0, 360]')
            guard_heading = heading_to_yaw(heading)
        if content.get('areas') or content.get('areasCount'):
            warnings.append('未应用区域约束：本版本仅执行航路终点定点值守，areas/areasCount 未适配')
        if content.get('guardingRadius') is not None:
            warnings.append('guardingRadius 未适配：不会用作允许漂移半径或定点控制精度')

    # Validate every protocol field before any coordinate conversion. A bad
    # later point rejects the packet without producing a partially usable task.
    normalized = []
    for index, point in enumerate(point_objects):
        label = f"points[{index}]"
        if type(point) is not dict:
            raise ValueError(f"{label} must be an object")
        point_type = _required_point(point, "type", label)
        _require_exact_integer(f"{label}.type", point_type)
        if point_type != 1:
            raise ValueError(f"{label}.type must be 1")
        latitude = _require_finite_number(
            f"{label}.latitude", _required_point(point, "latitude", label)
        )
        longitude = _require_finite_number(
            f"{label}.longitude", _required_point(point, "longitude", label)
        )
        speed_knots = _require_finite_number(
            f"{label}.speed", _required_point(point, "speed", label)
        )
        if not -90.0 <= latitude <= 90.0:
            raise ValueError(f"{label}.latitude must be in [-90, 90]")
        if not -180.0 <= longitude <= 180.0:
            raise ValueError(f"{label}.longitude must be in [-180, 180]")
        if speed_knots <= 0.0:
            raise ValueError(f"{label}.speed must be positive")
        normalized.append((latitude, longitude, speed_knots, point_type))

    waypoints = []
    for index, (latitude, longitude, speed_knots, _) in enumerate(normalized):
        try:
            x, y, _ = origin.to_local(latitude, longitude)
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(f"cannot convert points[{index}] to ENU: {exc}") from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"cannot convert points[{index}] to finite ENU coordinates")
        waypoints.append(RouteWaypoint(
            x=x,
            y=y,
            speed_knots=speed_knots,
            speed_mps=speed_knots * KNOT_TO_METRES_PER_SECOND,
        ))

    point_summaries = [
        {
            "latitude": latitude,
            "longitude": longitude,
            "speed": speed_knots,
            "type": point_type,
        }
        for latitude, longitude, speed_knots, point_type in normalized
    ]
    # Preserve the established one-point fingerprint contract. Multi-point
    # fingerprints cover the complete ordered route.
    summary = {
        "point": point_summaries[0],
        "taskType": task_type,
        "travelId": travel_id,
    } if count == 1 else {
        "points": point_summaries,
        "taskType": task_type,
        "travelId": travel_id,
    }
    if task_type == 80:
        summary['guard_heading_yaw'] = guard_heading
    canonical = json.dumps(
        summary, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return RouteTask(
        travel_id=travel_id,
        sender=sender,
        points=tuple(waypoints),
        fingerprint=hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
        wrapped=wrapped,
        task_type=task_type,
        guard_heading=guard_heading,
        warnings=tuple(warnings),
    )


def _required_point(point: dict, name: str, label: str):
    try:
        return point[name]
    except KeyError as exc:
        raise ValueError(f"missing required field: {label}.{name}") from exc


def _validate_optional_constraints(content: dict) -> None:
    areas_count = content.get("areasCount")
    if areas_count is not None:
        _require_exact_integer("areasCount", areas_count)
        if areas_count != 0:
            raise ValueError("only tasks without areas are supported")
    areas = content.get("areas")
    if areas is not None and (type(areas) is not list or areas):
        raise ValueError("areas must be null or an empty list")
    if content.get("guardingHeading") is not None:
        raise ValueError("guardingHeading is not supported")
    if content.get("guardingRadius") is not None:
        raise ValueError("guardingRadius is not supported")
