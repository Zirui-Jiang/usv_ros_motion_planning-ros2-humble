"""本船 0411 JSON 编码的纯函数回归。"""

from types import SimpleNamespace

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.ownship import ownship_payload_from_odometry


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
