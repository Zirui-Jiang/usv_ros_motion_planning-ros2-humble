"""将现有 ROS 2 里程计与外部 LaDDS 0411 JSON 互相转换。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping

from common.geodesy import GeoReference, heading_to_yaw
from common.math_helper import quaternion_yaw


KNOTS_PER_METRE_PER_SECOND = 3600.0 / 1852.0
METRES_PER_SECOND_PER_KNOT = 1852.0 / 3600.0


@dataclass(frozen=True)
class OwnshipOdometry:
    """0411 JSON 解出的平面位姿与体轴速度；不含 ROS 时间戳与坐标系。"""

    x: float
    y: float
    yaw: float
    speed_mps: float
    angular_z: float
    qx: float
    qy: float
    qz: float
    qw: float


def ownship_payload_from_odometry(message, origin: GeoReference) -> dict[str, float]:
    """把 ``nav_msgs/Odometry`` 转成 0411 的实际物理量 JSON 字段。

    0411 的航向采用真北顺时针角度，速度采用节，角速度采用度每秒。
    ``Odometry`` 的线速度保留纵向速度符号，以便反向航行时上报负航速。
    """
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    linear = message.twist.twist.linear
    angular = message.twist.twist.angular

    values = [
        float(position.x),
        float(position.y),
        float(linear.x),
        float(linear.y),
        float(angular.z),
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry position, velocity and angular velocity must be finite")

    latitude, longitude, _ = origin.to_geographic(values[0], values[1])
    speed_mps = math.hypot(values[2], values[3])
    if values[2] < 0.0:
        speed_mps = -speed_mps
    ros_yaw = quaternion_yaw(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    heading = (90.0 - math.degrees(ros_yaw)) % 360.0
    speed_knots = speed_mps * KNOTS_PER_METRE_PER_SECOND
    if not -20.0 <= speed_knots <= 200.0:
        raise ValueError("0411 speed is outside the documented -20..200 knot range")
    if not -180.0 <= longitude <= 180.0 or not -90.0 <= latitude <= 90.0:
        raise ValueError("0411 coordinates are outside WGS84 ranges")
    return {
        "longitude": round(float(longitude), 10),
        "latitude": round(float(latitude), 10),
        "speed": speed_knots,
        "heading": heading,
        "angularZ": math.degrees(values[4]),
    }


def parse_ownship_payload(text: str) -> dict[str, Any]:
    """解析 0411 JSON；若外层带 ``content`` 则使用内层业务对象。"""
    if not isinstance(text, str):
        raise ValueError("0411 payload must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid 0411 JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("0411 JSON root must be an object")
    content = root.get("content")
    if type(content) is dict:
        root = content
    return root


def odometry_from_ownship_payload(
    payload: Mapping[str, Any], origin: GeoReference
) -> OwnshipOdometry:
    """把 0411 物理量转回局部 ENU 位姿；横漂速度在协议中不存在，固定为 0。"""
    longitude = _finite_number("longitude", payload.get("longitude"))
    latitude = _finite_number("latitude", payload.get("latitude"))
    speed_knots = _finite_number("speed", payload.get("speed"))
    heading = _finite_number("heading", payload.get("heading"))
    angular_z_deg = _finite_number("angularZ", payload.get("angularZ"))
    if not -20.0 <= speed_knots <= 200.0:
        raise ValueError("0411 speed is outside the documented -20..200 knot range")
    if not -180.0 <= longitude <= 180.0 or not -90.0 <= latitude <= 90.0:
        raise ValueError("0411 coordinates are outside WGS84 ranges")
    x, y, _ = origin.to_local(latitude, longitude)
    yaw = heading_to_yaw(heading)
    half = yaw / 2.0
    return OwnshipOdometry(
        x=float(x),
        y=float(y),
        yaw=float(yaw),
        speed_mps=speed_knots * METRES_PER_SECOND_PER_KNOT,
        angular_z=math.radians(angular_z_deg),
        qx=0.0,
        qy=0.0,
        qz=math.sin(half),
        qw=math.cos(half),
    )


def _finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


__all__ = [
    "OwnshipOdometry",
    "odometry_from_ownship_payload",
    "ownship_payload_from_odometry",
    "parse_ownship_payload",
]
