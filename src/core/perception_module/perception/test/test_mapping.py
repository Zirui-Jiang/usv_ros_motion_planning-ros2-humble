# 整数证据建图内核回归：使用小地图精确检查光束顺序、量程、边界裁剪和坐标变换。
import math

import numpy as np
import pytest

from perception.mapping import GridConfig, GridMapper


def mapper(**kwargs):
    return GridMapper(GridConfig(width=8, resolution=1.0, **kwargs), backend="python")


def scan(grid, ranges, **kwargs):
    parameters = dict(angle_min=0.0, angle_increment=0.0, range_min=0.1,
                      range_max=20.0, pose_x=0.0, pose_y=0.0, pose_yaw=0.0)
    parameters.update(kwargs)
    return grid.integrate_scan(ranges, **parameters)


# 验证：默认地图尺寸、int8 类型和世界原点正确，复位后全为未知。
def test_metadata_unknown_and_reset():
    grid = GridMapper()
    assert grid.data.shape == (1024, 1024)
    assert grid.data.dtype == np.int8
    assert grid.origin_x == grid.origin_y == -256.0
    assert np.all(grid.data == -1)
    scan(grid, [1.0])
    grid.reset()
    assert np.all(grid.data == -1)


# 验证：首个命中为 54，重复命中饱和到 100，穿越可逐步清空。
def test_first_hit_saturation_and_clearing_keep_sequential_evidence():
    grid = mapper()
    scan(grid, [2.0])
    assert grid.data[4, :].tolist() == [-1, -1, -1, -1, 0, 0, 54, -1]
    scan(grid, [2.0])
    assert grid.data[4, 6] == 100
    scan(grid, [3.0, 3.0, 3.0, 3.0])
    assert grid.data[4, 6] == 0
    assert grid.data[4, 7] == 100


# 验证：共享栅格按光束输入顺序更新，因此交换次序产生不同证据。
def test_shared_cells_are_updated_in_beam_order():
    first = mapper()
    second = mapper()
    scan(first, [2.0, 3.0])
    scan(second, [3.0, 2.0])
    assert first.data[4, 6] == 29
    assert second.data[4, 6] == 55


# 验证：NaN、负无穷、负距离和小于最小量程的输入不清空地图。
def test_bad_ranges_do_not_clear_the_map():
    grid = mapper()
    before = grid.data.copy()
    result = scan(grid, [math.nan, -math.inf, -2.0, 0.0, 0.05])
    np.testing.assert_array_equal(grid.data, before)
    assert result.invalid_beams == 5
    assert result.valid_beams == 0


# 验证：正无穷或超量程返回只清空沿途，不标记虚假障碍。
@pytest.mark.parametrize("value", [math.inf, 20.0, 25.0])
def test_valid_no_return_clears_without_marking(value):
    grid = mapper()
    scan(grid, [value])
    assert grid.data[4].tolist() == [-1, -1, -1, -1, 0, 0, 0, 0]
    assert not np.any(grid.data > 0)


# 验证：无回波清空距离最多 30 米，图内未裁剪端点不被额外清空。
def test_clear_range_is_capped_at_30_m_and_excludes_unclipped_endpoint():
    grid = GridMapper(GridConfig(width=100, resolution=1.0), backend="python")
    scan(grid, [math.inf], range_max=80.0)
    assert grid.data[50, 79] == 0
    assert grid.data[50, 80] == -1


# 验证：二维边界裁剪不会跨行索引或把边界当命中。
def test_two_dimensional_clipping_never_wraps_a_row_or_marks_border_hit():
    grid = mapper()
    scan(grid, [15.0])
    assert np.count_nonzero(grid.data != -1) == 4
    assert grid.data[4, 7] == 0
    assert grid.data[5, 0] == -1
    assert grid.data[7, 7] == -1


# 验证：图外起点射线可进入地图，完全不相交射线保持地图不变。
def test_ray_entering_from_outside_and_ray_missing_map():
    grid = mapper()
    scan(grid, [12.0], pose_x=-10.0)
    assert grid.data[4].tolist() == [0, 0, 0, 0, 0, 0, 54, -1]
    before = grid.data.copy()
    scan(grid, [2.0], pose_x=-10.0, pose_y=-10.0)
    np.testing.assert_array_equal(grid.data, before)


# 验证：含小数原点偏移时坐标使用真实原点和 floor 得到行列。
def test_floor_coordinates_use_actual_origin_with_fractional_offset():
    grid = mapper(offset_x=0.25, offset_y=-0.25)
    assert grid.world_to_grid(0.0, 0.0) == (3, 4)
    scan(grid, [0.5])
    assert grid.data[4, 3] == 0
    assert grid.data[4, 4] == 54


# 验证：传感器 yaw 旋转与地图边界行为一致。
def test_yaw_rotation_and_map_boundary_are_consistent():
    grid = mapper()
    scan(grid, [2.0], pose_yaw=math.pi / 2)
    assert grid.data[6, 4] == 54
    assert grid.data[5, 4] == 0
    before = grid.data.copy()
    scan(grid, [1.0], pose_x=4.0)
    np.testing.assert_array_equal(grid.data, before)


# 验证：极大有限量程先裁剪再转整数索引，避免越界溢出。
def test_huge_valid_range_is_clipped_before_integer_conversion():
    grid = mapper()
    scan(grid, [1e20], range_max=1e21)
    assert np.count_nonzero(grid.data == 0) == 4
    assert not np.any(grid.data > 0)


# 验证：非法地图宽度、分辨率和偏移被构造器拒绝。
@pytest.mark.parametrize("config", [dict(width=0), dict(width=2.5),
                                      dict(resolution=0.0), dict(offset_x=math.nan)])
def test_invalid_map_geometry_is_rejected(config):
    with pytest.raises(ValueError):
        GridConfig(**config)


# 验证：非法扫描元数据抛错，空扫描不改变网格。
def test_bad_scan_metadata_is_rejected_and_empty_scan_is_noop():
    grid = mapper()
    with pytest.raises(ValueError):
        scan(grid, [1.0], range_max=math.inf)
    with pytest.raises(ValueError):
        scan(grid, [1.0], pose_yaw=math.nan)
    scan(grid, [])
    assert np.all(grid.data == -1)


# 验证：对角 Bresenham 只更新预期经过格与命中格。
def test_diagonal_bresenham_visits_expected_cells():
    grid = mapper()
    scan(grid, [math.sqrt(8)], pose_x=0.25, pose_y=0.25, angle_min=math.pi / 4)
    expected = np.full((8, 8), -1, dtype=np.int8)
    expected[4, 4] = expected[5, 5] = 0
    expected[6, 6] = 54
    np.testing.assert_array_equal(grid.data, expected)
