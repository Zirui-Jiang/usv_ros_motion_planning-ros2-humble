"""Numerical and edge-case contracts for migration from the C++ planners."""

import math
import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from path_planner import PathPlannerCore, create_planner, prune, resample
from path_planner.path_planner import grid_to_index, index_to_grid


def grid(data=None, resolution=1.0, origin_x=0.0, origin_y=0.0):
    return GridSnapshot(np.zeros((8, 8), dtype=np.uint8) if data is None else np.asarray(data, dtype=np.uint8),
                        resolution=resolution, origin_x=origin_x, origin_y=origin_y)


@pytest.mark.parametrize("name", ["astar", "dijkstra", "gbfs"])
def test_open_grid_matches_analytic_optimal_path(name):
    result = PathPlannerCore({"planner_name": name}).make_plan((0.5, 0.5, 0.3), (5.5, 3.5, -0.6), grid())
    assert result.status == AlgorithmStatus.SUCCESS
    np.testing.assert_allclose(result.path[[0, -1]], [[0.5, 0.5, 0.3], [5.5, 3.5, -0.6]])
    # Three diagonal and two axis-aligned steps are necessary and sufficient.
    length = np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1).sum()
    assert length == pytest.approx(3 * math.sqrt(2) + 2)
    assert result.expanded > 0 and result.elapsed >= 0


@pytest.mark.parametrize("name", ["astar", "dijkstra", "gbfs"])
def test_blocking_wall_has_no_path(name):
    data = np.zeros((8, 8), dtype=np.uint8)
    data[:, 4] = 254
    result = PathPlannerCore({"planner_name": name}).make_plan((1.5, 1.5, 0), (6.5, 1.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH
    assert result.path.shape == (0, 3)


def test_two_dimensional_bounds_cannot_alias_across_rows():
    # The only free cells have adjacent *linear* indices (2 and 3), but are
    # separated by blocked neighbours in the actual 3-column map.
    data = np.full((3, 3), 254, dtype=np.uint8)
    data[0, 2] = data[1, 0] = 0
    result = PathPlannerCore().make_plan((2.5, 0.5, 0), (0.5, 1.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH


@pytest.mark.parametrize("which", ["start", "goal"])
def test_blocked_endpoint_fails_explicitly(which):
    data = np.zeros((4, 4), dtype=np.uint8)
    data[(0, 0) if which == "start" else (3, 3)] = 254
    result = PathPlannerCore().make_plan((0.5, 0.5, 0), (3.5, 3.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH
    assert "blocked" in result.message


@pytest.mark.parametrize("point", [(-0.01, 0.5, 0), (8, 0.5, 0), (0.5, float("nan"), 0), (1, 2)])
def test_invalid_endpoint_is_not_projected_or_clamped(point):
    result = PathPlannerCore().make_plan(point, (3.5, 3.5, 0), grid())
    assert result.status == AlgorithmStatus.INVALID_INPUT


def test_unknown_policy_and_original_inflation_threshold():
    data = np.array([[0, 255, 0]], dtype=np.uint8)
    source, target = (0.5, 0.5, 0), (2.5, 0.5, 0)
    assert PathPlannerCore().make_plan(source, target, grid(data)).status == AlgorithmStatus.NO_PATH
    assert PathPlannerCore({"unknown_is_obstacle": False}).make_plan(source, target, grid(data)).success
    data[0, 1] = 127
    assert PathPlannerCore().make_plan(source, target, grid(data)).status == AlgorithmStatus.NO_PATH
    assert PathPlannerCore({"obstacle_factor": 1.0}).make_plan(source, target, grid(data)).success


def test_diagonal_corner_policy_is_explicit():
    scene = grid([[0, 254], [254, 0]])
    source, target = (0.5, 0.5, 0), (1.5, 1.5, 0)
    assert PathPlannerCore().make_plan(source, target, scene).success
    assert PathPlannerCore({"allow_corner_cutting": False}).make_plan(source, target, scene).status == AlgorithmStatus.NO_PATH


def test_outline_uses_owned_mask_and_leaves_snapshot_unchanged():
    scene = grid()
    original = scene.data.copy()
    result = PathPlannerCore({"outline_map": True}).make_plan((0.5, 0.5, 0), (2.5, 2.5, 0), scene)
    assert result.status == AlgorithmStatus.NO_PATH
    np.testing.assert_array_equal(scene.data, original)


def test_nonzero_origin_and_fractional_resolution_are_respected():
    scene = grid(resolution=0.25, origin_x=-5, origin_y=10)
    source, target = (-4.875, 10.125, 1), (-3.875, 11.125, 2)
    result = PathPlannerCore().make_plan(source, target, scene)
    assert result.success
    np.testing.assert_allclose(result.path[[0, -1]], [source, target])
    assert scene.world_to_map(-5.001, 10) is None
    assert scene.world_to_map(-3, 10) is None


def test_same_grid_cell_has_one_pose_with_goal_heading():
    result = PathPlannerCore().make_plan((0.6, 0.6, 0), (0.8, 0.7, 1.2), grid())
    assert result.success
    np.testing.assert_allclose(result.path, [[0.5, 0.5, 1.2]])


def test_expansion_budget_returns_timeout_with_no_partial_path():
    result = PathPlannerCore({"max_expansions": 1}).make_plan((0.5, 0.5, 0), (7.5, 7.5, 0), grid())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 1 and len(result.path) == 0


def test_clock_budget_is_enforced_deterministically(monkeypatch):
    from path_planner.graph_planner import astar_planner
    ticks = iter([10.0, 10.0, 11.1, 11.2, 11.3])
    monkeypatch.setattr(astar_planner.time, "perf_counter", lambda: next(ticks))
    result = PathPlannerCore({"timeout": 1.0}).make_plan((0.5, 0.5, 0), (7.5, 7.5, 0), grid())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 0


@pytest.mark.parametrize("config", [{"planner_name": "rrt"}, {"timeout": 0}, {"obstacle_factor": 0},
                                     {"obstacle_factor": float("nan")}, {"max_expansions": 1.5},
                                     {"max_expansions": 0}, {"enable_resample": True, "resample_ratio": 0}])
def test_configuration_errors_do_not_create_half_initialized_planners(config):
    with pytest.raises(ValueError):
        PathPlannerCore(config)


def test_grid_index_round_trip_and_axis_bounds():
    for y in range(3):
        for x in range(5):
            assert index_to_grid(grid_to_index(x, y, 5, 3), 5, 3) == (x, y)
    for x, y in [(-1, 1), (5, 0), (1, -1), (1, 3)]:
        with pytest.raises(ValueError):
            grid_to_index(x, y, 5, 3)
    with pytest.raises(ValueError):
        index_to_grid(15, 5, 3)


def test_path_pruning_matches_original_last_nearest_waypoint_rule():
    path = np.array([[0, 0, 0], [1, 0, 0.1], [2, 0, 0.2], [3, 0, 0.3]])
    result = prune(path, VesselState(x=1.5, y=0))
    np.testing.assert_allclose(result, path[2:])
    result[0, 0] = 99
    assert path[2, 0] == 2
    np.testing.assert_allclose(prune(path, (2, 0), prune_distance=1.0), path[1:])
    assert prune([], (0, 0)).shape == (0, 3)


def test_resampling_preserves_endpoint_pose_and_tangent_on_straight_line():
    path = [[0, 0, 0.1], [1, 0, 0], [2.2, 0, -0.2]]
    result = resample(path, 0.5)
    np.testing.assert_allclose(result[:, 0], [0, 0.5, 1, 1.5, 2, 2.2])
    np.testing.assert_allclose(result[1:-1, 2], 0)
    np.testing.assert_allclose(result[[0, -1]], np.asarray(path)[[0, -1]])


def test_resampling_circle_tangent_and_duplicate_points():
    points = np.array([[1, 0, 0.2], [1, 0, 0.2], [0, 1, 0], [-1, 0, 0.5]])
    result = resample(points, math.sqrt(2))
    assert result.shape == (3, 3)
    assert abs(abs(result[1, 2]) - math.pi) < 1e-10
    np.testing.assert_allclose(result[[0, -1]], points[[0, -1]])
    assert np.isfinite(resample([[1, 1, 0], [1, 1, 1]], 0.5)).all()


@pytest.mark.parametrize("spacing", [0, -1, float("nan"), float("inf")])
def test_invalid_resampling_spacing_fails(spacing):
    with pytest.raises(ValueError):
        resample([[0, 0], [1, 1]], spacing)


def test_core_resample_configuration_is_applied():
    result = PathPlannerCore({"enable_resample": True, "resample_spacing": 0.25}).make_plan(
        (0.5, 0.5, 0.1), (2.5, 0.5, 0.2), grid())
    assert result.success and len(result.path) == 9
    np.testing.assert_allclose(result.path[[0, -1], 2], [0.1, 0.2])


def test_coarse_resampling_cannot_cut_across_an_obstacle():
    from common.math_helper import path_is_clear
    data = np.zeros((7, 7), dtype=np.uint8)
    data[:5, 3] = 254
    scene = grid(data)
    source, target = (0.5, 0.5, 0), (6.5, 0.5, 0)
    assert not path_is_clear(scene, np.asarray([source, target]))
    result = PathPlannerCore({"enable_resample": True, "resample_spacing": 100.0}).make_plan(source, target, scene)
    assert result.success
    assert len(result.path) > 2
    assert path_is_clear(scene, result.path, threshold=127)
    assert "retained" in result.message
