"""Check geometry, original default kernel, and previously unsafe edge cases."""

import math
import numpy as np
import pytest

from path_processor import (RDPPathProcessor, SavitzkyGolayPathProcessor,
                            ramer_douglas_peucker, savitzky_golay)


@pytest.mark.parametrize("processor", [RDPPathProcessor(), SavitzkyGolayPathProcessor()])
def test_empty_and_single_point_paths_are_preserved(processor):
    assert processor.process([]).shape == (0, 3)
    np.testing.assert_array_equal(processor.process([[1, 2, 0.3]]), [[1, 2, 0.3]])


def test_rdp_preserves_bends_and_respects_distance_threshold():
    path = np.array([[0, 0, 0], [1, 0.1, 0.1], [2, 0, 0.2], [3, 2, 0.3], [4, 2, 0.4]])
    result = ramer_douglas_peucker(path, epsilon=0.2)
    np.testing.assert_allclose(result, path[[0, 2, 3, 4]])


def test_rdp_uses_segment_distance_for_backtracking_points():
    # Distance to the infinite endpoint line is zero, but distance to the
    # endpoint segment is 3 m: the detour must remain.
    path = [[0, 0, 0], [4, 0, 0.1], [1, 0, 0.2]]
    np.testing.assert_allclose(ramer_douglas_peucker(path, 0.1), path)


def test_rdp_densifies_by_original_interval_policy_and_shortest_heading():
    points = [[0, 0, math.radians(170)], [2, 0, math.radians(-170)]]
    result = ramer_douglas_peucker(points, 0, max_interval=1)
    np.testing.assert_allclose(result[:, 0], [0, 2 / 3, 4 / 3, 2])
    assert np.all(np.abs(result[:, 2]) > math.radians(160))
    np.testing.assert_allclose(result[[0, -1]], points)


@pytest.mark.parametrize("options", [{"epsilon": -1}, {"max_interval": 0}, {"max_interval": -1},
                                     {"epsilon": float("nan")}, {"max_interval": float("inf")}])
def test_rdp_rejects_invalid_thresholds(options):
    with pytest.raises(ValueError):
        ramer_douglas_peucker([[0, 0], [1, 1]], **options)


def test_rdp_duplicate_endpoints_remain_finite():
    path = [[0, 0, 0], [1, 1, 1], [0, 0, 2]]
    result = ramer_douglas_peucker(path, 0.1, 0.5)
    assert np.isfinite(result).all()
    np.testing.assert_allclose(result[[0, -1]], np.asarray(path)[[0, -1]])


@pytest.mark.parametrize("count", range(1, 7))
def test_sg_short_paths_return_the_input_instead_of_empty_output(count):
    path = np.arange(count * 3, dtype=float).reshape(count, 3)
    np.testing.assert_array_equal(savitzky_golay(path), path)


def test_sg_default_kernel_matches_old_interior_and_preserves_both_endpoints():
    path = np.column_stack((np.arange(11), [0, 0, 1, -1, 2, -2, 1, 0, 1, 0, 0], np.full(11, 0.7)))
    result = savitzky_golay(path)
    kernel = np.array([-2, 3, 6, 7, 6, 3, -2]) / 21
    np.testing.assert_allclose(result[5, :2], kernel @ path[2:9, :2], atol=1e-12)
    np.testing.assert_array_equal(result[[0, -1]], path[[0, -1]])
    assert result.shape == path.shape and np.isfinite(result).all()
    assert np.var(np.diff(result[:, 1], n=2)) < np.var(np.diff(path[:, 1], n=2))


def test_sg_reproduces_quadratic_interior_without_changing_input():
    x = np.arange(15, dtype=float)
    path = np.column_stack((x, x * x, np.zeros(len(x))))
    original = path.copy()
    result = savitzky_golay(path)
    np.testing.assert_allclose(result[3:-3, :2], path[3:-3, :2], atol=1e-10)
    np.testing.assert_array_equal(path, original)


@pytest.mark.parametrize("options", [{"window_length": 2}, {"window_length": 4}, {"window_length": 7.0},
                                     {"polyorder": -1}, {"polyorder": 7}, {"polyorder": 1.5}])
def test_sg_invalid_filter_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        savitzky_golay([], **options)
