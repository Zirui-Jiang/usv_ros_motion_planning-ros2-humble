# 验证完整矩形船体与地图单元面积、平移/旋转扫掠、角度跨界及地图越界的碰撞判定。
import math
import numpy as np
import pytest
from common.collision import footprint_path_is_clear
from common.models import GridSnapshot


def grid_with_cell(x, y, cost):
    values = np.zeros((80, 80), np.uint8)
    values[y, x] = cost
    return GridSnapshot(values, .25, -10., -10.)


# 软膨胀成本不被重复当作物理障碍。
@pytest.mark.parametrize('cost', [1, 127, 162, 252, 253])
def test_soft_inflation_cost_is_not_a_second_physical_obstacle(cost):
    assert footprint_path_is_clear(grid_with_cell(40, 40, cost), [[0., 0., 0.]])


# 中心可通行但船艏接触致命或未知单元时必须拒绝。
@pytest.mark.parametrize('cost', [254, 255])
def test_lethal_and_unknown_cell_area_at_bow_rejects_clear_centre(cost):
    grid = grid_with_cell(44, 40, cost)  # Square starts x=1m; centre is outside bow.
    assert grid.data[40, 40] == 0
    assert not footprint_path_is_clear(grid, [[0., 0., 0.]])


# 侧向间距随船体航向变化，检测完整单元方块。
def test_side_clearance_depends_on_heading_and_full_cell_square():
    grid = grid_with_cell(40, 44, 254)
    assert footprint_path_is_clear(grid, [[0., 0., 0.]])
    assert not footprint_path_is_clear(grid, [[0., 0., math.pi / 2]])


# 平移端点均安全时仍检测中间障碍。
def test_sweep_detects_obstacle_between_clear_endpoints():
    grid = grid_with_cell(40, 40, 254)
    assert footprint_path_is_clear(grid, [[-2., 0., 0.]])
    assert footprint_path_is_clear(grid, [[2., 0., 0.]])
    assert not footprint_path_is_clear(grid, [[-2., 0., 0.], [2., 0., 0.]])


# 原地转动端点均安全时仍检测中间船角碰撞。
def test_rotation_sweep_detects_corner_collision_with_clear_endpoints():
    grid = grid_with_cell(43, 43, 254)
    assert footprint_path_is_clear(grid, [[0., 0., 0.]])
    assert footprint_path_is_clear(grid, [[0., 0., math.pi / 2]])
    assert not footprint_path_is_clear(grid, [[0., 0., 0.], [0., 0., math.pi / 2]])


# 跨 ±π 使用短弧，不扫过无关障碍。
def test_short_angle_wrap_does_not_rotate_through_unrelated_obstacle():
    grid = grid_with_cell(40, 44, 254)
    assert footprint_path_is_clear(grid, [[0., 0., math.pi - .02], [0., 0., -math.pi + .02]])


# 畸形、非有限及部分船体越界均拒绝。
@pytest.mark.parametrize('poses', [[], [[0., 0.]], [[float('nan'), 0., 0.]], [[-9.5, 0., 0.]]])
def test_invalid_or_partially_outside_map_is_rejected(poses):
    assert not footprint_path_is_clear(grid_with_cell(0, 0, 0), poses)
