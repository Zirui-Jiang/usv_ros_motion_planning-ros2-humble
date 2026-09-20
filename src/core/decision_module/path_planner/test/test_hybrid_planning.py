"""Hybrid A* must preserve forward geometry through the existing public API."""
import math
from pathlib import Path

import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot
from path_planner import PathPlannerCore, create_planner


def scene(data=None, resolution=.25, origin=(-5., -5.)):
    return GridSnapshot(np.zeros((100, 100), np.uint8) if data is None else data,
                        resolution, *origin)


def options(**changes):
    return dict(planner_name="hybrid_astar", **{
        "minimum_turning_radius": 2., "motion_step": .75,
        "collision_step": .2, "analytic_expansion_max_length": 16.,
        "footprint_length": .4, "footprint_width": .3,
        "timeout": 4., **changes})


def assert_forward_curvature(path, radius):
    """Independent circular chord identity catches kinks, reversal and yaw snaps."""
    delta = np.diff(path[:, :2], axis=0)
    chord = np.linalg.norm(delta, axis=1)
    angle = (np.diff(path[:, 2]) + math.pi) % (2*math.pi) - math.pi
    assert np.all(chord > 1e-10)
    assert np.all(2*np.abs(np.sin(angle/2)) <= chord/radius + 1e-8)
    middle_heading = path[:-1, 2] + angle/2
    along = delta[:, 0]*np.cos(middle_heading) + delta[:, 1]*np.sin(middle_heading)
    cross = -delta[:, 0]*np.sin(middle_heading) + delta[:, 1]*np.cos(middle_heading)
    assert np.all(along > 0)
    np.testing.assert_allclose(cross, 0., atol=1e-7)


def test_factory_accepts_hybrid_and_keeps_existing_default():
    assert create_planner(options()).name == "hybrid_astar"
    assert create_planner().name == "astar"


def test_straight_plan_retains_exact_world_poses_and_input_snapshot():
    grid = scene(resolution=.3, origin=(-6., -6.))
    original = grid.data.copy()
    start, goal = np.array([.13, .17, 0.]), np.array([6.81, .17, 0.])
    result = PathPlannerCore(options()).make_plan(start, goal, grid)
    assert result.success, result.message
    np.testing.assert_allclose(result.path[[0, -1]], [start, goal], atol=1e-8)
    np.testing.assert_allclose(result.path[:, 1], start[1], atol=1e-8)
    assert_forward_curvature(result.path, 2.)
    np.testing.assert_array_equal(grid.data, original)
    result.path[0] = 99
    np.testing.assert_array_equal(start, [.13, .17, 0.])


@pytest.mark.parametrize("resample", [False, True])
def test_quarter_circle_keeps_radius_even_when_generic_resampling_is_enabled(resample):
    result = PathPlannerCore(options(enable_resample=resample, resample_spacing=100.)).make_plan(
        (0., 0., 0.), (2., 2., math.pi/2), scene())
    assert result.success, result.message
    assert len(result.path) > 10
    np.testing.assert_allclose(result.path[[0, -1]], [[0., 0., 0.], [2., 2., math.pi/2]], atol=1e-8)
    np.testing.assert_allclose(np.linalg.norm(result.path[:, :2]-[0., 2.], axis=1), 2., atol=1e-7)
    assert_forward_curvature(result.path, 2.)


def test_already_at_same_pose_returns_one_pose():
    result = PathPlannerCore(options()).make_plan((.13, .21, .4), (.13, .21, .4), scene())
    assert result.success
    np.testing.assert_allclose(result.path, [[.13, .21, .4]])


def test_nearly_coincident_turn_centres_do_not_invalidate_valid_goal():
    grid = scene(np.zeros((120, 120), np.uint8), resolution=1., origin=(0., 0.))
    start, goal = (60., 60., 0.), (69.9999998, 50., -math.pi/2)
    result = PathPlannerCore(options(minimum_turning_radius=10.,
                                     analytic_expansion_max_length=100.)).make_plan(start, goal, grid)
    assert result.success, result.message
    np.testing.assert_allclose(result.path[-1], goal, atol=1e-8)
    assert_forward_curvature(result.path, 10.)


def test_same_cell_different_pose_is_not_collapsed_or_rotated_in_place():
    result = PathPlannerCore(options()).make_plan((0., 0., 0.), (.1, .1, math.pi/2), scene())
    assert result.success, result.message
    assert len(result.path) > 3
    assert_forward_curvature(result.path, 2.)
    np.testing.assert_allclose(result.path[-1], [.1, .1, math.pi/2], atol=1e-8)


def test_search_routes_around_an_obstacle_when_direct_dubins_is_blocked():
    data = np.zeros((100, 100), np.uint8)
    data[12:29, 32:36] = 254  # obstacle x=3..4, y=-2..2.25
    result = PathPlannerCore(options()).make_plan((0., 0., 0.), (9., 0., 0.), scene(data))
    assert result.success, result.message
    assert result.expanded > 1
    assert np.max(np.abs(result.path[:, 1])) > 2.25
    assert_forward_curvature(result.path, 2.)


def test_captured_long_leg_completes_with_bounded_search_work():
    # 2026-09-16 实际闪烁场景：旧搜索需要2628次展开，反复耗尽1秒预算。
    # 约束搜索工作量而不是机器耗时，避免CPU负载不同造成计时断言不稳定。
    fixture = Path(__file__).parent / 'fixtures/hybrid_long_leg/map.npz'
    with np.load(fixture) as captured:
        grid = GridSnapshot(captured['data'], float(captured['resolution']),
                            float(captured['origin_x']), float(captured['origin_y']))
    start = (20.463065539613464, -17.893448898983326, 2.207099429703204)
    goal = (-10.353519439697266, 21.081073760986328, 2.1980385560365274)
    config = options(minimum_turning_radius=4., motion_step=1., footprint_length=2.2,
                     footprint_width=1.2, obstacle_factor=.25, outline_map=True,
                     analytic_expansion_max_length=20., max_expansions=500, timeout=10.)
    result = PathPlannerCore(config).make_plan(start, goal, grid)
    assert result.success, result.message
    assert result.expanded < 500
    np.testing.assert_allclose(result.path[[0, -1]], [start, goal], atol=1e-8)
    assert_forward_curvature(result.path, 4.)
    from common.collision import footprint_path_is_clear
    assert footprint_path_is_clear(grid, result.path, 2.2, 1.2)


def test_bow_collision_is_rejected_even_when_center_is_free():
    data = np.zeros((100, 100), np.uint8)
    data[19:21, 23] = 254
    result = PathPlannerCore(options(footprint_length=2.2)).make_plan((0., 0., 0.), (5., 0., 0.), scene(data))
    assert result.status == AlgorithmStatus.NO_PATH
    assert result.path.shape == (0, 3)


@pytest.mark.parametrize("point", [(-5.1, 0., 0.), (20., 0., 0.), (1., 2.), (0., math.nan, 0.)])
def test_invalid_endpoint_does_not_project_back_into_map(point):
    result = PathPlannerCore(options()).make_plan(point, (4., 0., 0.), scene())
    assert result.status == AlgorithmStatus.INVALID_INPUT
    assert result.path.shape == (0, 3)


def test_expansion_limit_returns_no_partial_or_cached_path():
    planner = PathPlannerCore(options(max_expansions=1, analytic_expansion_max_length=1.))
    assert planner.make_plan((0., 0., 0.), (.5, 0., 0.), scene()).success
    result = planner.make_plan((0., 0., 0.), (8., 8., math.pi/2), scene())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 1
    assert result.path.shape == (0, 3)


def test_timeout_includes_map_preparation(monkeypatch):
    from types import SimpleNamespace
    from path_planner.graph_planner import hybrid_astar_planner
    ticks = iter([1000., 1002., 1003.])
    monkeypatch.setattr(hybrid_astar_planner, "time", SimpleNamespace(perf_counter=lambda: next(ticks)))
    result = create_planner(options(timeout=1.)).plan((0., 0., 0.), (4., 0., 0.), scene())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 0 and result.path.shape == (0, 3)


def test_extreme_cost_sampling_cannot_bypass_budget():
    result = create_planner(options(collision_step=1e-12, cost_penalty=1.)).plan(
        (0., 0., 0.), (4., 0., 0.), scene())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.path.shape == (0, 3)


def test_wall_cannot_be_jumped_by_long_motion_primitives():
    data = np.zeros((12, 32), np.uint8)
    data[:, 16] = 254
    grid = scene(data, resolution=.5, origin=(-2., -3.))
    result = PathPlannerCore(options(motion_step=5., minimum_turning_radius=4., timeout=4.)).make_plan(
        (0., 0., 0.), (11., 0., 0.), grid)
    assert result.status == AlgorithmStatus.NO_PATH, result.message
    assert result.path.shape == (0, 3)


@pytest.mark.parametrize("change", [
    {"minimum_turning_radius": 0.}, {"minimum_turning_radius": math.inf},
    {"angle_bins": 3}, {"angle_bins": 72.5}, {"angle_bins": True},
    {"motion_step": 0.}, {"collision_step": -1.}, {"timeout": 0.},
    {"analytic_expansion_max_length": 0.}, {"non_straight_penalty": .9},
    {"change_penalty": -1.}, {"cost_penalty": -1.}, {"footprint_padding": -1.},
    {"footprint_length": 0.}, {"footprint_width": math.nan},
    {"max_expansions": 1.2}, {"max_expansions": False}, {"obstacle_factor": 0.},
    {"heuristic_weight": .99}, {"heuristic_weight": math.nan},
    {"heuristic_weight": math.inf}, {"heuristic_weight": True},
    {"turning_radius_multipliers": []}, {"turning_radius_multipliers": [0.5]},
    {"turning_radius_multipliers": [1., True]}, {"turning_radius_multipliers": "1"},
])
def test_bad_configuration_is_rejected_at_construction(change):
    with pytest.raises(ValueError):
        PathPlannerCore(options(**change))


def test_default_curvatures_are_straight_and_minimum_radius():
    planner = create_planner(options())
    radius = planner.minimum_turning_radius
    assert planner.turning_radius_multipliers == (1.,)
    assert planner.curvatures == (0., 1. / radius, -1. / radius)


def test_turning_radius_multipliers_add_shallower_arcs_and_still_plan():
    planner = create_planner(options(turning_radius_multipliers=[2., 1., 1.]))
    radius = planner.minimum_turning_radius
    assert planner.turning_radius_multipliers == (1., 2.)
    assert planner.curvatures == (0., 1. / radius, -1. / radius, .5 / radius, -.5 / radius)
    result = PathPlannerCore(options(turning_radius_multipliers=[1., 2.])).make_plan(
        (0., 0., 0.), (6., 0., 0.), scene())
    assert result.success, result.message
    assert_forward_curvature(result.path, 2.)


def test_reconstruct_preserves_the_radius_of_a_shallower_search_arc():
    from path_planner.graph_planner.dubins import DubinsPath, advance_pose
    from path_planner.graph_planner.hybrid_astar_planner import HybridAStarPathPlanner, _SearchNode

    planner = HybridAStarPathPlanner(options(turning_radius_multipliers=[1., 2.]))
    first = np.array([0., 0., 0.])
    curvature = .5 / planner.minimum_turning_radius
    endpoint = advance_pose(first, curvature, 2.)
    node = _SearchNode(endpoint, 1., _SearchNode(first, 0.), curvature, 2.)

    path = planner._reconstruct(
        node, DubinsPath((), (), planner.minimum_turning_radius), first, endpoint, .2, 1e300)

    np.testing.assert_allclose(np.linalg.norm(path[:, :2] - [0., 4.], axis=1), 4., atol=1e-7)
    assert_forward_curvature(path, 4.)
