"""Convert one polar LaserScan observation into 0141 target JSON.

The simulator currently exposes obstacles as ``sensor_msgs/LaserScan``.  A scan
is a set of measured points rather than a tracked-vessel list, so this module
groups neighbouring valid beams into temporary geometric targets.  It has no
ROS dependency; the ROS node is responsible for extracting the message fields
and supplying the vehicle pose.
"""

from __future__ import annotations

import json
import math
from numbers import Real
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from common.geodesy import GeoReference


DEFAULT_THREAT_RADIUS = 5.0
DEFAULT_MAX_TARGETS = 255


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite number")
    return converted


def _validate_inputs(
    ranges: Iterable[Any],
    angle_min: Any,
    angle_increment: Any,
    range_min: Any,
    range_max: Any,
    vehicle_x: Any,
    vehicle_y: Any,
    vehicle_yaw: Any,
    origin: GeoReference,
    cluster_range_jump: Optional[Any],
    threat_radius: Any,
    max_targets: Any,
) -> Tuple[List[Any], float, float, float, float, float, float, float, Optional[float], float, int]:
    if isinstance(ranges, (str, bytes)):
        raise ValueError("ranges must be a non-empty sequence of numbers")
    try:
        values = list(ranges)
    except TypeError as exc:
        raise ValueError("ranges must be a non-empty sequence of numbers") from exc
    if not values:
        raise ValueError("ranges must be a non-empty sequence of numbers")

    amin = _finite_number(angle_min, "angle_min")
    ainc = _finite_number(angle_increment, "angle_increment")
    rmin = _finite_number(range_min, "range_min")
    rmax = _finite_number(range_max, "range_max")
    x = _finite_number(vehicle_x, "vehicle_x")
    y = _finite_number(vehicle_y, "vehicle_y")
    yaw = _finite_number(vehicle_yaw, "vehicle_yaw")
    radius = _finite_number(threat_radius, "threat_radius")
    if ainc <= 0.0:
        raise ValueError("angle_increment must be greater than zero")
    if rmin < 0.0:
        raise ValueError("range_min must be non-negative")
    if rmax <= rmin:
        raise ValueError("range_max must be greater than range_min")
    if radius < 0.0:
        raise ValueError("threat_radius must be non-negative")
    if not isinstance(origin, GeoReference):
        raise ValueError("origin must be a GeoReference")

    jump = None
    if cluster_range_jump is not None:
        jump = _finite_number(cluster_range_jump, "cluster_range_jump")
        if jump <= 0.0:
            raise ValueError("cluster_range_jump must be greater than zero")

    if isinstance(max_targets, bool) or not isinstance(max_targets, int):
        raise ValueError("max_targets must be an integer")
    if not 1 <= max_targets <= 255:
        raise ValueError("max_targets must be in [1, 255]")
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, Real):
            raise ValueError(f"ranges[{index}] must be a number")
    return values, amin, ainc, rmin, rmax, x, y, yaw, jump, radius, max_targets


def _valid_range(value: Any, range_min: float, range_max: float) -> bool:
    number = float(value)
    return math.isfinite(number) and range_min <= number <= range_max


def _clusters(
    values: Sequence[Any],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    cluster_range_jump: Optional[float],
) -> List[List[Tuple[float, float, float]]]:
    clusters: List[List[Tuple[float, float, float]]] = []
    current: List[Tuple[float, float, float]] = []
    previous_range: Optional[float] = None
    for index, raw_range in enumerate(values):
        if not _valid_range(raw_range, range_min, range_max):
            if current:
                clusters.append(current)
                current = []
            previous_range = None
            continue

        measured_range = float(raw_range)
        if (
            current
            and cluster_range_jump is not None
            and previous_range is not None
            and abs(measured_range - previous_range) > cluster_range_jump
        ):
            clusters.append(current)
            current = []
        angle = angle_min + index * angle_increment
        current.append((measured_range, math.cos(angle), math.sin(angle)))
        previous_range = measured_range
    if current:
        clusters.append(current)
    return clusters


def _signed_degrees(radians: float) -> float:
    degrees = math.degrees(radians)
    return (degrees + 180.0) % 360.0 - 180.0


def _bearing_degrees(dx: float, dy: float) -> float:
    """Return true-north clockwise bearing for an ENU vector."""

    return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0


def scan_to_0141_dict(
    ranges: Iterable[Any],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
    origin: GeoReference,
    sequence: int = 1,
    cluster_range_jump: Optional[float] = None,
    threat_radius: float = DEFAULT_THREAT_RADIUS,
    max_targets: int = DEFAULT_MAX_TARGETS,
) -> Dict[str, Any]:
    """Build a JSON-compatible 0141 payload from one LaserScan.

    ``vehicle_x`` and ``vehicle_y`` are local ENU metres relative to ``origin``;
    ``vehicle_yaw`` follows ROS conventions (radians from east, positive CCW).
    Invalid beams (NaN, infinity, or outside the sensor range) delimit clusters.
    ``cluster_range_jump`` optionally splits adjacent valid beams when their
    measured ranges differ by more than the supplied metres.
    """

    (
        values,
        amin,
        ainc,
        rmin,
        rmax,
        vehicle_east,
        vehicle_north,
        yaw,
        jump,
        radius,
        target_limit,
    ) = _validate_inputs(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        vehicle_x,
        vehicle_y,
        vehicle_yaw,
        origin,
        cluster_range_jump,
        threat_radius,
        max_targets,
    )
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError("sequence must be an integer")
    # Keep the same 16-bit modulo convention as the previous 0141 adapter.
    batch = sequence % 65535
    cosine_yaw = math.cos(yaw)
    sine_yaw = math.sin(yaw)
    targets: List[Dict[str, Any]] = []

    for cluster in _clusters(
        values,
        angle_min=amin,
        angle_increment=ainc,
        range_min=rmin,
        range_max=rmax,
        cluster_range_jump=jump,
    )[:target_limit]:
        # Average Cartesian points so the reported bearing is the centre of
        # the measured obstacle, rather than the first beam in the run.
        local_x = sum(measured * cos_angle for measured, cos_angle, _ in cluster) / len(cluster)
        local_y = sum(measured * sin_angle for measured, _, sin_angle in cluster) / len(cluster)
        distance = sum(measured for measured, _, _ in cluster) / len(cluster)
        world_dx = cosine_yaw * local_x - sine_yaw * local_y
        world_dy = sine_yaw * local_x + cosine_yaw * local_y
        target_x = vehicle_east + world_dx
        target_y = vehicle_north + world_dy
        latitude, longitude, _ = origin.to_geographic(target_x, target_y)
        target = {
            "batch": batch,
            "absSpeed": -1.0,
            "absCourse": -1.0,
            "heading": -1.0,
            "longitude": longitude,
            "latitude": latitude,
            "distance": distance,
            "absAzimuth": _bearing_degrees(world_dx, world_dy),
            "relAzimuth": _signed_degrees(math.atan2(local_y, local_x)),
            "sign": 0,
            "type": 0,
            "color": 0,
            "state": 0,
            "threatingRadius": radius,
            "code": "",
            "model": 0,
            "name": "",
            "shipLength": -1.0,
            "shipWidth": -1.0,
            "shipHeight": -1.0,
            "MMSI": 0,
            "source": 1,
            "areaType": 1,
            "localX": local_x,
            "localY": local_y,
            "pointCount": len(cluster),
        }
        targets.append(target)

    return {
        "format": "lidar_targets_v1",
        "source": "laser_scan",
        "count": len(targets),
        "targets": targets,
    }


def scan_to_0141_json(*args: Any, **kwargs: Any) -> str:
    """Serialize :func:`scan_to_0141_dict` as compact JSON text."""

    return json.dumps(
        scan_to_0141_dict(*args, **kwargs),
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    )
