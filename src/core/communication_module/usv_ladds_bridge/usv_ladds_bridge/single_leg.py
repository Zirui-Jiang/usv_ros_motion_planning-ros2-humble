"""Pure parsing for the supported one-waypoint LaDDS task message."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Optional, Tuple

from common.geodesy import GeoReference


KNOT_TO_METRES_PER_SECOND = 1852.0 / 3600.0


@dataclass(frozen=True)
class SingleLeg:
    """Validated single waypoint expressed in the simulation ENU frame."""

    travel_id: int
    sender: Optional[int]
    x: float
    y: float
    speed_knots: float
    speed_mps: float
    fingerprint: str
    wrapped: bool

    @property
    def identity(self) -> Tuple[Optional[int], int]:
        """Route identity only; each received command gets its own execution ID."""
        return (self.sender, self.travel_id)


def parse_single_leg(text: str, origin: GeoReference) -> SingleLeg:
    """Parse one supported task without retaining or changing parser state."""
    if not isinstance(text, str):
        raise ValueError("single-leg task must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid single-leg JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("single-leg JSON root must be an object")

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
    if task_type != 16:
        raise ValueError("taskType must be 16 for a single-leg task")

    count = _required(content, "count")
    _require_exact_integer("count", count)
    if count != 1:
        raise ValueError("count must be exactly 1")
    points = _required(content, "points")
    if type(points) is not list or len(points) != 1:
        raise ValueError("points must be a list containing exactly one point")
    point = points[0]
    if type(point) is not dict:
        raise ValueError("the single point must be an object")

    point_type = _required(point, "type")
    _require_exact_integer("points[0].type", point_type)
    if point_type != 1:
        raise ValueError("points[0].type must be 1")
    latitude = _require_finite_number(
        "points[0].latitude", _required(point, "latitude")
    )
    longitude = _require_finite_number(
        "points[0].longitude", _required(point, "longitude")
    )
    speed_knots = _require_finite_number(
        "points[0].speed", _required(point, "speed")
    )
    if not -90.0 <= latitude <= 90.0:
        raise ValueError("points[0].latitude must be in [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("points[0].longitude must be in [-180, 180]")
    if speed_knots <= 0.0:
        raise ValueError("points[0].speed must be positive")

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

    try:
        x, y, _ = origin.to_local(latitude, longitude)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot convert waypoint to ENU: {exc}") from exc

    summary = json.dumps(
        {
            "point": {
                "latitude": latitude,
                "longitude": longitude,
                "speed": speed_knots,
                "type": point_type,
            },
            "taskType": task_type,
            "travelId": travel_id,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return SingleLeg(
        travel_id=travel_id,
        sender=sender,
        x=x,
        y=y,
        speed_knots=speed_knots,
        speed_mps=speed_knots * KNOT_TO_METRES_PER_SECOND,
        fingerprint=hashlib.sha256(summary.encode("utf-8")).hexdigest(),
        wrapped=wrapped,
    )


# 完成回报：普通航路只含航线号和完成码；值守与靠离泊额外回传任务类型。
def make_completion(leg: SingleLeg) -> dict:
    """Build the documented bare completion business object.

    0212 echoes its task type: 3 is departure, 4 is berthing. 0211 taskType 80
    echoes 80. Ordinary routes stay travelId plus taskFinish only.
    """
    payload = {
        "travelId": leg.travel_id,
        "taskFinish": getattr(leg, "task_finish", 1),
    }
    task_type = getattr(leg, "task_type", None)
    if getattr(leg, "protocol", "0211") == "0212" or task_type == 80:
        payload = {
            "travelId": leg.travel_id,
            "taskType": task_type,
            "taskFinish": payload["taskFinish"],
        }
    return payload


def _required(mapping: dict, name: str):
    try:
        return mapping[name]
    except KeyError as exc:
        raise ValueError(f"missing required field: {name}") from exc


def _require_exact_integer(name: str, value: object) -> None:
    if type(value) is not int:
        raise ValueError(f"{name} must be an integer")


def _require_finite_number(name: str, value: object) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a number")
    try:
        converted = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted
