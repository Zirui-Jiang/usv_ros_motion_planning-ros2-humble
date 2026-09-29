"""0141 接收侧纯函数：剥外壳、与 0411 同原点换算、画圆和栅格窗口。"""

import json
import math

import pytest

from common.geodesy import GeoReference
from usv_ladds_bridge.obstacle_grid import (
    FREE,
    OCCUPIED,
    ObstacleDisc,
    parse_obstacle_message,
    rasterize,
    target_discs,
)
from usv_ladds_bridge.ownship import odometry_from_ownship_payload


ORIGIN = GeoReference(30.5472, 114.3891, 0.0)


def _target(east, north, **fields):
    latitude, longitude, _ = ORIGIN.to_geographic(east, north)
    return {"batch": 1, "latitude": latitude, "longitude": longitude, **fields}


def _cell(grid, x, y):
    column = math.floor((x - grid.origin_x) / grid.resolution)
    row = math.floor((y - grid.origin_y) / grid.resolution)
    assert 0 <= column < grid.width and 0 <= row < grid.height
    return grid.data[row * grid.width + column]


# 验证：带 head/content 外壳时取 content，不带外壳的业务对象照常接受。
def test_wrapped_and_bare_messages_yield_the_same_content():
    content = {"count": 1, "targets": [{"batch": 1, "latitude": 30.5, "longitude": 114.4}]}
    wrapped = json.dumps({"head": {"packageSeq": 3, "sendPlatform": 1001}, "content": content})
    assert parse_obstacle_message(wrapped) == (content, True)
    assert parse_obstacle_message(json.dumps(content)) == (content, False)


@pytest.mark.parametrize(("text", "message"), [
    ('{"content": {"count": 0, "targets": []}}', "head and content"),
    ('{"head": {}, "content": []}', "head and content"),
    ('{"head": {}, "content": {"count": 1, "targets": []}}', "count must match"),
    ('[]', "root must be an object"),
    ('not-json', "invalid obstacle JSON"),
])
def test_malformed_messages_are_rejected(text, message):
    with pytest.raises(ValueError, match=message):
        parse_obstacle_message(text)


# 验证：障碍与 0411 解出的船位使用同一原点，同一经纬度落在同一个地图坐标。
def test_target_uses_the_same_conversion_as_0411():
    target = _target(123.4, -56.7)
    discs, skipped = target_discs(
        {"count": 1, "targets": [target]}, ORIGIN, default_radius=5.0)
    ownship = odometry_from_ownship_payload({
        "latitude": target["latitude"], "longitude": target["longitude"],
        "speed": 0.0, "heading": 0.0, "angularZ": 0.0,
    }, ORIGIN)
    assert skipped == 0
    assert discs[0].x == pytest.approx(ownship.x, abs=1e-6)
    assert discs[0].y == pytest.approx(ownship.y, abs=1e-6)
    assert (discs[0].x, discs[0].y) == pytest.approx((123.4, -56.7), abs=1e-3)


# 验证：threatingRadius 乘系数得到米，缺失、非正或布尔值时改用默认半径。
@pytest.mark.parametrize(("fields", "scale", "radius"), [
    ({"threatingRadius": 8.0}, 1.0, 8.0),
    ({"threatingRadius": 50}, 0.1, 5.0),
    ({}, 1.0, 3.0),
    ({"threatingRadius": 0}, 1.0, 3.0),
    ({"threatingRadius": -2.0}, 1.0, 3.0),
    ({"threatingRadius": True}, 1.0, 3.0),
])
def test_radius_comes_from_threating_radius_or_default(fields, scale, radius):
    discs, _ = target_discs(
        {"count": 1, "targets": [_target(0.0, 0.0, **fields)]},
        ORIGIN, default_radius=3.0, radius_scale=scale)
    assert discs[0].radius == pytest.approx(radius)


# 验证：单个目标经纬度无效只跳过该目标，其余目标照常换算。
def test_invalid_coordinates_skip_only_that_target():
    targets = [
        _target(10.0, 0.0),
        {"batch": 2, "latitude": "30.5", "longitude": 114.4},
        {"batch": 3, "latitude": 95.0, "longitude": 114.4},
        {"batch": 4, "longitude": 114.4},
    ]
    discs, skipped = target_discs(
        {"count": len(targets), "targets": targets}, ORIGIN, default_radius=5.0)
    assert skipped == 3
    assert [(round(d.x, 3), round(d.y, 3)) for d in discs] == [(10.0, 0.0)]


# 验证：圆内格子占用、圆外空闲，栅格刚好覆盖所有圆并对齐整格。
def test_discs_are_filled_inside_a_snapped_bounding_grid():
    discs = [ObstacleDisc(40.3, 10.2, 5.0), ObstacleDisc(-20.0, 30.0, 2.0)]
    grid = rasterize(discs, resolution=1.0, max_size=1500.0, anchor=(0.0, 0.0))
    assert grid.dropped == 0
    assert grid.origin_x == pytest.approx(-22.0) and grid.origin_y == pytest.approx(5.0)
    assert grid.origin_x + grid.width >= 45.3 and grid.origin_y + grid.height >= 32.0
    assert len(grid.data) == grid.width * grid.height
    assert _cell(grid, 40.3, 10.2) == OCCUPIED
    assert _cell(grid, 44.0, 10.2) == OCCUPIED
    assert _cell(grid, 40.3, 14.8) == OCCUPIED
    assert _cell(grid, 36.0, 6.0) == FREE
    assert _cell(grid, -20.0, 30.0) == OCCUPIED
    assert _cell(grid, 10.0, 20.0) == FREE


# 验证：小于一格的目标仍占住圆心所在的格子。
def test_disc_smaller_than_a_cell_still_marks_its_cell():
    grid = rasterize([ObstacleDisc(3.9, 7.95, 0.1)], resolution=1.0, max_size=100.0)
    assert _cell(grid, 3.9, 7.95) == OCCUPIED
    assert sum(1 for value in grid.data if value == OCCUPIED) == 1


# 验证：没有目标时仍给一格空闲栅格，用来整张清掉上一包的障碍。
def test_empty_list_gives_one_free_cell_at_the_anchor():
    grid = rasterize([], resolution=1.0, max_size=1500.0, anchor=(23.4, -10.6))
    assert (grid.width, grid.height, list(grid.data)) == (1, 1, [FREE])
    assert (grid.origin_x, grid.origin_y) == (23.0, -11.0)


# 验证：窗口以锚点为中心限制边长，完全在窗口外的圆不画并计数。
def test_window_around_anchor_drops_far_discs():
    discs = [ObstacleDisc(10.0, 0.0, 5.0), ObstacleDisc(900.0, 0.0, 5.0)]
    grid = rasterize(discs, resolution=1.0, max_size=200.0, anchor=(0.0, 0.0))
    assert grid.dropped == 1
    assert grid.width <= 201 and grid.height <= 201
    assert _cell(grid, 10.0, 0.0) == OCCUPIED
