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


def test_metadata_unknown_and_reset():
    grid = GridMapper()
    assert grid.data.shape == (1024, 1024)
    assert grid.data.dtype == np.int8
    assert grid.origin_x == grid.origin_y == -256.0
    assert np.all(grid.data == -1)
    scan(grid, [1.0])
    grid.reset()
    assert np.all(grid.data == -1)


def test_first_hit_saturation_and_clearing_keep_sequential_evidence():
    grid = mapper()
    scan(grid, [2.0])
    assert grid.data[4, :].tolist() == [-1, -1, -1, -1, 0, 0, 54, -1]
    scan(grid, [2.0])
    assert grid.data[4, 6] == 100
    scan(grid, [3.0, 3.0, 3.0, 3.0])
    assert grid.data[4, 6] == 0
    assert grid.data[4, 7] == 100


def test_shared_cells_are_updated_in_beam_order():
    first = mapper()
    second = mapper()
    scan(first, [2.0, 3.0])
    scan(second, [3.0, 2.0])
    assert first.data[4, 6] == 29
    assert second.data[4, 6] == 55


def test_bad_ranges_do_not_clear_the_map():
    grid = mapper()
    before = grid.data.copy()
    result = scan(grid, [math.nan, -math.inf, -2.0, 0.0, 0.05])
    np.testing.assert_array_equal(grid.data, before)
    assert result.invalid_beams == 5
    assert result.valid_beams == 0


@pytest.mark.parametrize("value", [math.inf, 20.0, 25.0])
def test_valid_no_return_clears_without_marking(value):
    grid = mapper()
    scan(grid, [value])
    assert grid.data[4].tolist() == [-1, -1, -1, -1, 0, 0, 0, 0]
    assert not np.any(grid.data > 0)


def test_clear_range_is_capped_at_30_m_and_excludes_unclipped_endpoint():
    grid = GridMapper(GridConfig(width=100, resolution=1.0), backend="python")
    scan(grid, [math.inf], range_max=80.0)
    assert grid.data[50, 79] == 0
    assert grid.data[50, 80] == -1


def test_two_dimensional_clipping_never_wraps_a_row_or_marks_border_hit():
    grid = mapper()
    scan(grid, [15.0])
    assert np.count_nonzero(grid.data != -1) == 4
    assert grid.data[4, 7] == 0
    assert grid.data[5, 0] == -1
    assert grid.data[7, 7] == -1


def test_ray_entering_from_outside_and_ray_missing_map():
    grid = mapper()
    scan(grid, [12.0], pose_x=-10.0)
    assert grid.data[4].tolist() == [0, 0, 0, 0, 0, 0, 54, -1]
    before = grid.data.copy()
    scan(grid, [2.0], pose_x=-10.0, pose_y=-10.0)
    np.testing.assert_array_equal(grid.data, before)


def test_floor_coordinates_use_actual_origin_with_fractional_offset():
    grid = mapper(offset_x=0.25, offset_y=-0.25)
    assert grid.world_to_grid(0.0, 0.0) == (3, 4)
    scan(grid, [0.5])
    assert grid.data[4, 3] == 0
    assert grid.data[4, 4] == 54


def test_yaw_rotation_and_map_boundary_are_consistent():
    grid = mapper()
    scan(grid, [2.0], pose_yaw=math.pi / 2)
    assert grid.data[6, 4] == 54
    assert grid.data[5, 4] == 0
    before = grid.data.copy()
    scan(grid, [1.0], pose_x=4.0)
    np.testing.assert_array_equal(grid.data, before)


def test_huge_valid_range_is_clipped_before_integer_conversion():
    grid = mapper()
    scan(grid, [1e20], range_max=1e21)
    assert np.count_nonzero(grid.data == 0) == 4
    assert not np.any(grid.data > 0)


@pytest.mark.parametrize("config", [dict(width=0), dict(width=2.5),
                                      dict(resolution=0.0), dict(offset_x=math.nan)])
def test_invalid_map_geometry_is_rejected(config):
    with pytest.raises(ValueError):
        GridConfig(**config)


def test_bad_scan_metadata_is_rejected_and_empty_scan_is_noop():
    grid = mapper()
    with pytest.raises(ValueError):
        scan(grid, [1.0], range_max=math.inf)
    with pytest.raises(ValueError):
        scan(grid, [1.0], pose_yaw=math.nan)
    scan(grid, [])
    assert np.all(grid.data == -1)


def test_diagonal_bresenham_visits_expected_cells():
    grid = mapper()
    scan(grid, [math.sqrt(8)], pose_x=0.25, pose_y=0.25, angle_min=math.pi / 4)
    expected = np.full((8, 8), -1, dtype=np.int8)
    expected[4, 4] = expected[5, 5] = 0
    expected[6, 6] = 54
    np.testing.assert_array_equal(grid.data, expected)
