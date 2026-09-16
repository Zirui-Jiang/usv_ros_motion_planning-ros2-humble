# 验证 WGS84/ENU 解析基准、跨半球极区往返、实际世界原点以及海事艏向约定。
import math
from pathlib import Path

import pytest

from common.geodesy import GeoReference, WGS84_A, WGS84_F, heading_to_yaw


# 赤道格林尼治原点与已知 ECEF 轴关系一致。
def test_wgs84_equatorial_axis_has_known_absolute_ecef():
    reference = GeoReference(0.0, 0.0)
    # At equator/Greenwich ENU is exactly (ECEF Y, ECEF Z, ECEF X - a).
    assert reference.to_local(0.0, 90.0) == pytest.approx((WGS84_A, 0.0, -WGS84_A), abs=1e-8)
    assert reference.to_local(90.0, 0.0) == pytest.approx((0.0, WGS84_A * (1.0 - WGS84_F), -WGS84_A), abs=1e-8)


# 不同半球和极区局部坐标往返误差满足容限。
@pytest.mark.parametrize("origin", [(-33.7242, 150.68, 0.0), (51.5, -0.12, 14.0),
                                   (89.99, 179.99, -10.0), (-90.0, -180.0, 20.0)])
@pytest.mark.parametrize("point", [(0.0, 0.0, 0.0), (23.0, -10.0, 0.0), (-400.0, 300.0, 5.0)])
def test_enu_roundtrip_at_different_hemispheres_and_poles(origin, point):
    reference = GeoReference(*origin)
    latitude, longitude, altitude = reference.to_geographic(*point)
    assert reference.to_local(latitude, longitude, altitude) == pytest.approx(point, abs=2e-7)


# 实际世界地理参考与已知示例点转换一致。
def test_world_reference_matches_current_world_and_example_point():
    world = Path(__file__).resolve().parents[4] / "sim_env/worlds/ocean.world"
    reference = GeoReference.from_world(world)
    assert reference == GeoReference(-33.7242, 150.68, 0.0)
    assert reference.to_local(-33.72429015691839, 150.6802481598667)[:2] == pytest.approx((23.0, -10.0), abs=1e-6)


# 真北顺时针度正确转换为 ROS 东向逆时针弧度。
@pytest.mark.parametrize("degrees,yaw", [(0, math.pi / 2), (90, 0), (180, -math.pi / 2),
                                       (270, -math.pi), (360, math.pi / 2), (-90, -math.pi)])
def test_heading_convention(degrees, yaw):
    assert heading_to_yaw(degrees) == pytest.approx(yaw)


# 越界、非有限或布尔经纬高必须拒绝。
@pytest.mark.parametrize("coordinates", [(91, 0, 0), (0, -181, 0), (float("nan"), 0, 0),
                                       (0, 0, float("inf")), (True, 0, 0)])
def test_invalid_reference_rejected(coordinates):
    with pytest.raises(ValueError):
        GeoReference(*coordinates)


# 世界缺参考、错误坐标系/表面/航向或缺字段时明确报错。
@pytest.mark.parametrize("change", ["absent", "NED", "heading", "surface", "latitude", "elevation"])
def test_unsupported_or_incomplete_world_rejected(tmp_path, change):
    world = Path(__file__).resolve().parents[4] / "sim_env/worlds/ocean.world"
    text = world.read_text(encoding="utf-8")
    if change == "absent":
        text = "<sdf><world name='missing'/></sdf>"
    elif change == "NED":
        text = text.replace(">ENU<", ">NED<")
    elif change == "heading":
        text = text.replace("<heading_deg>0</heading_deg>", "<heading_deg>90</heading_deg>")
    elif change == "surface":
        text = text.replace("EARTH_WGS84", "MOON_SCS")
    elif change == "latitude":
        text = text.replace("-33.7242", "nan")
    else:
        text = text.replace("<elevation>0</elevation>", "")
    path = tmp_path / "world.world"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError):
        GeoReference.from_world(path)
