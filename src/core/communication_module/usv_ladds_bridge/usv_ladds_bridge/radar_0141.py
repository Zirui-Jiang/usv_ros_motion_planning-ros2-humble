# 将激光雷达扫描聚类为本船坐标系目标，再换成 0141 经纬度列表。
"""Cluster a planar radar/LiDAR scan into 0141 tracked-target JSON.

The simulation sensor is a 360° Gazebo ray scan published as LaserScan.
0141 expects a list of vessel-like objects with WGS84 coordinates, so this
module groups contiguous hits and reports one centroid per cluster.  It has
no ROS dependency.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Dict, List, Mapping, Sequence

from common.geodesy import GeoReference
from common.math_helper import quaternion_yaw


DEFAULT_CLUSTER_GAP_M = 3.0
DEFAULT_MIN_POINTS = 3
DEFAULT_MAX_TARGETS = 255
DEFAULT_LIDAR_X = 0.0
DEFAULT_LIDAR_Y = 0.0
# 东湖 0141 样例把 source=1 用作雷达/观测来源。
RADAR_SOURCE = 1


@dataclass(frozen=True)
class OwnshipPose:
    """局部 ENU 位姿：x 东、y 北，ROS yaw 为东向起算逆时针弧度。"""

    x: float
    y: float
    yaw: float


def _finite(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _field(value: Any, name: str, default: Any = None) -> Any:
    if value is None:
        return default
    if isinstance(value, Mapping):
        return value.get(name, default)
    return getattr(value, name, default)


def yaw_to_heading(yaw: float) -> float:
    """ROS yaw（东向逆时针弧度）转为真北顺时针航向角，单位度。"""
    if not _finite(yaw):
        raise ValueError("ownship yaw must be finite")
    return (90.0 - math.degrees(yaw)) % 360.0


def wrap_degrees_180(value: float) -> float:
    """规整到 (-180, 180]。"""
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def east_north_to_azimuth(east: float, north: float) -> float:
    """ENU 位移转为真北顺时针方位角，单位度。"""
    if not (_finite(east) and _finite(north)):
        raise ValueError("target offset must be finite")
    if east == 0.0 and north == 0.0:
        return 0.0
    return (90.0 - math.degrees(math.atan2(north, east))) % 360.0


def _valid_range(distance: float, range_min: float, range_max: float) -> bool:
    return (
        _finite(distance)
        and range_min <= distance < range_max
        and distance > 0.0
    )


def cluster_scan_hits(
    ranges: Sequence[Any],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    cluster_gap_m: float = DEFAULT_CLUSTER_GAP_M,
    min_points: int = DEFAULT_MIN_POINTS,
) -> List[Dict[str, float]]:
    """把相邻有效回波聚成雷达坐标系中的质心。"""
    if not _finite(angle_min) or not _finite(angle_increment):
        raise ValueError("scan angles must be finite")
    if not _finite(range_min) or not _finite(range_max) or range_max <= range_min:
        raise ValueError("scan range_min must be less than range_max")
    if not _finite(cluster_gap_m) or cluster_gap_m <= 0.0:
        raise ValueError("cluster_gap_m must be a positive finite number")
    points_required = int(min_points)
    if points_required <= 0:
        raise ValueError("min_points must be greater than zero")

    clusters: List[Dict[str, float]] = []
    current: List[tuple[float, float, float, float]] = []
    previous = None

    def flush() -> None:
        nonlocal previous
        if len(current) >= points_required:
            east = sum(item[0] for item in current) / len(current)
            north = sum(item[1] for item in current) / len(current)
            clusters.append(
                {
                    "x": east,
                    "y": north,
                    "range": math.hypot(east, north),
                    "bearing": math.atan2(north, east),
                    "point_count": float(len(current)),
                }
            )
        current.clear()
        previous = None

    for index, raw in enumerate(ranges):
        distance = float(raw) if _finite(raw) else math.nan
        if not _valid_range(distance, range_min, range_max):
            flush()
            continue
        angle = angle_min + index * angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if previous is not None and math.hypot(x - previous[0], y - previous[1]) > cluster_gap_m:
            flush()
        current.append((x, y, distance, angle))
        previous = (x, y)
    flush()
    return clusters


def _lidar_to_world(
    cluster: Mapping[str, float],
    pose: OwnshipPose,
    lidar_x: float,
    lidar_y: float,
) -> tuple[float, float]:
    x_base = lidar_x + float(cluster["x"])
    y_base = lidar_y + float(cluster["y"])
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    east = pose.x + x_base * cos_yaw - y_base * sin_yaw
    north = pose.y + x_base * sin_yaw + y_base * cos_yaw
    return east, north


def scan_to_0141_targets(
    scan: Any,
    pose: OwnshipPose,
    origin: GeoReference,
    *,
    cluster_gap_m: float = DEFAULT_CLUSTER_GAP_M,
    min_points: int = DEFAULT_MIN_POINTS,
    max_targets: int = DEFAULT_MAX_TARGETS,
    lidar_x: float = DEFAULT_LIDAR_X,
    lidar_y: float = DEFAULT_LIDAR_Y,
) -> List[Dict[str, Any]]:
    """由一帧扫描和本船位姿生成 0141 ``targets`` 数组。"""
    if not all(_finite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError("ownship pose must be finite")
    if not all(_finite(value) for value in (lidar_x, lidar_y)):
        raise ValueError("lidar offset must be finite")
    limit = int(max_targets)
    if not 1 <= limit <= 255:
        raise ValueError("max_targets must be in [1, 255]")

    clusters = cluster_scan_hits(
        list(_field(scan, "ranges", ()) or ()),
        angle_min=float(_field(scan, "angle_min", 0.0)),
        angle_increment=float(_field(scan, "angle_increment", 0.0)),
        range_min=float(_field(scan, "range_min", 0.0)),
        range_max=float(_field(scan, "range_max", 0.0)),
        cluster_gap_m=cluster_gap_m,
        min_points=min_points,
    )
    heading = yaw_to_heading(pose.yaw)
    targets: List[Dict[str, Any]] = []
    for cluster in clusters:
        east, north = _lidar_to_world(cluster, pose, lidar_x, lidar_y)
        latitude, longitude, _ = origin.to_geographic(east, north)
        offset_east = east - pose.x
        offset_north = north - pose.y
        distance = math.hypot(offset_east, offset_north)
        abs_azimuth = east_north_to_azimuth(offset_east, offset_north)
        targets.append(
            {
                "batch": 0,
                "absSpeed": 0.0,
                "absCourse": 0.0,
                "heading": -1.0,
                "longitude": round(float(longitude), 10),
                "latitude": round(float(latitude), 10),
                "distance": round(distance, 3),
                "absAzimuth": round(abs_azimuth, 3),
                "relAzimuth": round(wrap_degrees_180(abs_azimuth - heading), 3),
                "sign": 0,
                "type": 0,
                "color": 0,
                "state": 0,
                "threatingRadius": 5.0,
                "code": "",
                "model": 0,
                "name": "",
                "shipLength": -1.0,
                "shipWidth": -1.0,
                "shipHeight": -1.0,
                "MMSI": 0,
                "source": RADAR_SOURCE,
                "areaType": 1,
            }
        )
    targets.sort(key=lambda item: float(item["distance"]))
    trimmed = targets[:limit]
    for index, target in enumerate(trimmed, start=1):
        target["batch"] = index
    return trimmed


def scan_to_0141_json(
    scan: Any,
    pose: OwnshipPose,
    origin: GeoReference,
    **kwargs: Any,
) -> str:
    """把一帧雷达扫描编成桥接层可转发的 0141 JSON 文本。"""
    targets = scan_to_0141_targets(scan, pose, origin, **kwargs)
    payload = {"count": len(targets), "targets": targets}
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def pose_from_odometry(message: Any) -> OwnshipPose:
    """从里程计取出平面位置和 ROS yaw。"""
    position = _field(_field(message, "pose", None), "pose", None)
    point = _field(position, "position", None)
    orientation = _field(position, "orientation", None)
    x = float(_field(point, "x", math.nan))
    y = float(_field(point, "y", math.nan))
    yaw = quaternion_yaw(
        float(_field(orientation, "x", 0.0)),
        float(_field(orientation, "y", 0.0)),
        float(_field(orientation, "z", 0.0)),
        float(_field(orientation, "w", 1.0)),
    )
    return OwnshipPose(x=x, y=y, yaw=yaw)


__all__ = [
    "DEFAULT_CLUSTER_GAP_M",
    "DEFAULT_LIDAR_X",
    "DEFAULT_LIDAR_Y",
    "DEFAULT_MAX_TARGETS",
    "DEFAULT_MIN_POINTS",
    "OwnshipPose",
    "cluster_scan_hits",
    "east_north_to_azimuth",
    "pose_from_odometry",
    "scan_to_0141_json",
    "scan_to_0141_targets",
    "yaw_to_heading",
]
