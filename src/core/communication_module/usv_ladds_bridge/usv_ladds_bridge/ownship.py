"""将现有 ROS 2 里程计转换为外部 LaDDS 0411 JSON。"""

from __future__ import annotations

import math

from common.geodesy import GeoReference
from common.math_helper import quaternion_yaw


KNOTS_PER_METRE_PER_SECOND = 3600.0 / 1852.0


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


__all__ = ["ownship_payload_from_odometry"]
