"""本船 0411 JSON 编码的纯函数回归。"""

from types import SimpleNamespace
import json
import math

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.ownship import (
    odometry_from_ownship_payload,
    ownship_payload_from_odometry,
    parse_ownship_payload,
)


def test_ownship_payload_uses_wgs84_knots_true_heading_and_deg_per_second():
    odom = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=2.0, y=0.0),
                angular=SimpleNamespace(z=0.1),
            )
        ),
    )

    payload = ownship_payload_from_odometry(odom, GeoReference(30.0, 120.0))

    assert payload["longitude"] == pytest.approx(120.0)
    assert payload["latitude"] == pytest.approx(30.0)
    assert payload["speed"] == pytest.approx(2.0 * 3600.0 / 1852.0)
    assert payload["heading"] == pytest.approx(90.0)
    assert payload["angularZ"] == pytest.approx(0.1 * 180.0 / 3.141592653589793)


def test_ownship_payload_preserves_reverse_longitudinal_speed_sign():
    odom = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=-1.0, y=0.0),
                angular=SimpleNamespace(z=0.0),
            )
        ),
    )

    payload = ownship_payload_from_odometry(odom, GeoReference(30.0, 120.0))

    assert payload["speed"] == pytest.approx(-3600.0 / 1852.0)


def _odom(x=0.0, y=0.0, yaw=0.0, u=2.0, v=0.0, r=0.1):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=u, y=v),
                angular=SimpleNamespace(z=r),
            )
        ),
    )


def test_ownship_payload_round_trip_restores_enu_pose_and_longitudinal_speed():
    origin = GeoReference(30.0, 120.0)
    payload = ownship_payload_from_odometry(_odom(x=10.0, y=-4.0, yaw=0.3, u=-1.5, r=-0.2), origin)
    restored = odometry_from_ownship_payload(payload, origin)

    assert restored.x == pytest.approx(10.0, abs=1e-3)
    assert restored.y == pytest.approx(-4.0, abs=1e-3)
    assert restored.yaw == pytest.approx(0.3, abs=1e-6)
    assert restored.speed_mps == pytest.approx(-1.5, abs=1e-9)
    assert restored.angular_z == pytest.approx(-0.2, abs=1e-9)


def test_ownship_decode_drops_lateral_velocity_that_0411_cannot_carry():
    origin = GeoReference(30.0, 120.0)
    payload = ownship_payload_from_odometry(_odom(u=3.0, v=4.0, r=0.0), origin)
    restored = odometry_from_ownship_payload(payload, origin)

    assert restored.speed_mps == pytest.approx(5.0)
    assert payload["speed"] == pytest.approx(5.0 * 3600.0 / 1852.0)


def test_parse_ownship_payload_accepts_wrapped_content_object():
    origin = GeoReference(30.0, 120.0)
    payload = ownship_payload_from_odometry(_odom(), origin)
    wrapped = parse_ownship_payload(
        json.dumps({"head": {"topic": "0411"}, "content": payload})
    )
    restored = odometry_from_ownship_payload(wrapped, origin)
    assert restored.x == pytest.approx(0.0, abs=1e-6)
    assert restored.speed_mps == pytest.approx(2.0)
