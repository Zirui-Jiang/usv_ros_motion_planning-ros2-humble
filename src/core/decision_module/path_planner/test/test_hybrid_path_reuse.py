"""Reference-path reuse must preserve forward geometry and current-map safety."""
import math

import numpy as np
import pytest

from common.models import GridSnapshot
from path_planner.graph_planner.dubins import DubinsPath, sample_dubins
from path_planner.graph_planner.hybrid_collision import HybridCollisionChecker
from path_planner.graph_planner import hybrid_path_reuse as reuse


def options(**changes):
    return {"minimum_turning_radius": 2., "footprint_length": .2,
            "footprint_width": .1, "collision_step": .2, **changes}


def scene(obstacles=(), *, resolution=.1, origin=(-10., -10.), size=600):
    data = np.zeros((size, size), dtype=np.uint8)
    for x, y, cost in obstacles:
        ix = math.floor((x-origin[0])/resolution)
        iy = math.floor((y-origin[1])/resolution)
        data[iy, ix] = cost
    return GridSnapshot(data, resolution, *origin)


def prepare(path, state, goal=None, grid=None, config=None, **kwargs):
    return reuse.prepare_cached_path(
        path, state, np.asarray(path)[-1] if goal is None else goal,
        scene() if grid is None else grid, options() if config is None else config, **kwargs)


def shortcut(path, grid=None, config=None, **kwargs):
    return reuse.shortcut_straight_connections(
        path, scene() if grid is None else grid,
        options() if config is None else config, **kwargs)


def quarter_circle(count=17):
    angles = np.linspace(0., math.pi/2, count)
    return np.column_stack((2*np.sin(angles), 2*(1-np.cos(angles)), angles))


def curved_detour(start=(0., 0., 0.)):
    return sample_dubins(start, DubinsPath(("L", "R", "L"),
                                         (math.pi, 2*math.pi, math.pi), 2.), .2)


def test_small_offset_projects_onto_reference_instead_of_inserting_boat_pose():
    path = np.array([[0., 0., 0.], [2., 0., 0.], [4., 0., 0.], [6., 0., 0.]])
    before = path.copy()
    result = prepare(path, (2.6, .4, .1))

    np.testing.assert_allclose(result, [[2.6, 0., 0.], [4., 0., 0.], [6., 0., 0.]], atol=1e-12)
    np.testing.assert_array_equal(path, before)
    result[0] = -99.
    np.testing.assert_array_equal(path, before)


def test_simplified_long_line_can_be_reused_after_more_than_twenty_metres():
    path = shortcut(np.column_stack((np.arange(0., 41.), np.zeros((41, 2)))))
    assert len(path) == 2
    for position in (3., 8., 13., 18., 23., 28., 33.):
        path = prepare(path, (position, .15, .03), goal=(40., 0., 0.))
        assert path is not None
        np.testing.assert_allclose(path, [[position, 0., 0.], [40., 0., 0.]], atol=1e-10)


def test_projection_on_arc_keeps_constant_radius_and_tangent_heading():
    path = quarter_circle(count=3)
    angle = .3
    # Slightly outside the radius-2 circle centred at (0,2).
    state = (2.2*math.sin(angle), 2.-2.2*math.cos(angle), angle+.02)
    result = prepare(path, state)
    assert result is not None
    np.testing.assert_allclose(result[0], [2*math.sin(angle), 2*(1-math.cos(angle)), angle], atol=1e-10)
    np.testing.assert_array_equal(result[1:], path[1:])
    delta = result[1, :2]-result[0, :2]
    curvature = 2*math.sin((result[1, 2]-result[0, 2])/2)/np.linalg.norm(delta)
    assert curvature == pytest.approx(.5, abs=1e-10)


def test_closed_route_tied_projection_keeps_first_pass_through_start():
    angles = np.linspace(0., 2*math.pi, 33)
    path = np.column_stack((2*np.sin(angles), 2*(1-np.cos(angles)), angles))
    path[-1] = (0., 0., 2*math.pi)
    result = prepare(path, (0., 0., .01), config=options(reuse_search_distance=100.))
    assert result is not None
    np.testing.assert_array_equal(result, path)


def test_projection_is_limited_by_prefix_arc_length_even_on_one_long_segment():
    path = np.array([[0., 0., 0.], [40., 0., 0.]])
    assert prepare(path, (14., 0., 0.)) is None
    result = prepare(path, (11., 0., 0.))
    np.testing.assert_allclose(result, [[10., 0., 0.], [40., 0., 0.]])


@pytest.mark.parametrize("state", [(1., 2.01, 0.), (-2.01, 0., 0.),
                                  (1., 0., 1.05), (1., 0., math.nan)])
def test_excessive_position_or_heading_error_rejects_cache(state):
    assert prepare([[0., 0., 0.], [6., 0., 0.]], state) is None


@pytest.mark.parametrize("goal", [(6.0001, 0., 0.), (6., 0., .001)])
def test_changed_exact_goal_rejects_cache(goal):
    assert prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.), goal=goal) is None


@pytest.mark.parametrize("path", [np.empty((0, 3)), [[0., 0.], [1., 0.]],
                                 [[0., 0., 0.], [math.nan, 0., 0.]],
                                 [[0., 0., 0.], [-1., 0., 0.]],
                                 [[0., 0., 0.], [1., 1., 0.]],
                                 [[0., 0., 0.], [0., 0., .1]],
                                 [[0., 0., 0.], [1., 1., math.pi/2]]])
def test_nonfinite_or_non_forward_or_too_sharp_cache_is_rejected(path):
    goal = np.asarray(path)[-1] if len(path) else (6., 0., 0.)
    assert prepare(path, (0., 0., 0.), goal=goal) is None


@pytest.mark.parametrize("cost", [127, 254])
def test_thin_obstacle_between_clear_endpoints_invalidates_cache(cost):
    grid = scene([(3.05, .01, cost)])
    assert prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.), grid=grid) is None


def test_obstacle_behind_projected_progress_does_not_invalidate_remaining_route():
    grid = scene([(.5, .01, 254)])
    result = prepare([[0., 0., 0.], [6., 0., 0.]], (2., .1, 0.), grid=grid)
    np.testing.assert_allclose(result, [[2., 0., 0.], [6., 0., 0.]])


def test_collision_on_true_arc_between_sparse_endpoints_invalidates_cache():
    grid = scene([(math.sqrt(2), 2-math.sqrt(2), 254)])
    assert prepare(quarter_circle(count=2), (0., 0., 0.), grid=grid) is None


def test_obstacle_on_chord_but_away_from_true_arc_does_not_invalidate_cache():
    result = prepare(quarter_circle(count=2), (0., 0., 0.), grid=scene([(1., 1., 254)]))
    assert result is not None


def test_actual_boat_hull_must_be_clear_even_when_reference_is_clear():
    assert prepare([[0., 0., 0.], [6., 0., 0.]], (.5, .8, 0.),
                   grid=scene([(.5, .8, 254)])) is None


def test_soft_band_under_hull_is_not_inflated_again():
    result = prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.),
                     grid=scene([(3., .5, 200)]),
                     config=options(footprint_length=2.2, footprint_width=1.2))
    assert result is not None


@pytest.mark.parametrize("unknown_is_obstacle", [True, False])
def test_unknown_cells_follow_current_policy(unknown_is_obstacle):
    result = prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.),
                     grid=scene([(3.05, .01, 255)]),
                     config=options(unknown_is_obstacle=unknown_is_obstacle))
    assert (result is None) is unknown_is_obstacle


def test_rolling_map_origin_is_applied_to_each_cache_validation():
    path = [[5., 5., 0.], [11., 5., 0.]]
    assert prepare(path, (5.2, 5.1, 0.), grid=scene(origin=(2., 2.))) is not None
    assert prepare(path, (5.2, 5.1, 0.),
                   grid=scene([(9., 5., 254)], origin=(2., 2.))) is None


def test_single_pose_at_exact_goal_is_preserved():
    result = prepare([[.13, .17, .2]], (.13, .17, .2))
    np.testing.assert_array_equal(result, [[.13, .17, .2]])


def test_contiguous_circle_samples_share_one_real_swept_check(monkeypatch):
    calls = []
    original = HybridCollisionChecker.motion_is_clear

    def counted(self, start, curvature, distance, deadline=math.inf):
        calls.append((curvature, distance))
        return original(self, start, curvature, distance, deadline=deadline)

    monkeypatch.setattr(HybridCollisionChecker, "motion_is_clear", counted)
    result = prepare(quarter_circle(count=101), (0., 0., 0.))
    assert result is not None
    assert len(calls) == 1
    assert calls[0] == pytest.approx((.5, math.pi))


def test_expired_reuse_deadline_is_not_converted_to_no_path():
    with pytest.raises(TimeoutError):
        prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.), deadline=0.)


def test_reuse_rechecks_deadline_after_current_pose_validation(monkeypatch):
    original = HybridCollisionChecker.pose_is_clear
    clock = [0.]

    def delayed(self, pose):
        clear = original(self, pose)
        clock[0] = 2.
        return clear

    monkeypatch.setattr(HybridCollisionChecker, "pose_is_clear", delayed)
    monkeypatch.setattr(reuse.time, "perf_counter", lambda: clock[0])
    with pytest.raises(TimeoutError):
        prepare([[0., 0., 0.], [6., 0., 0.]], (0., 0., 0.), deadline=1.)


def test_shortcut_compresses_same_direction_line_and_preserves_exact_endpoints():
    path = np.array([[.13, .17, 0.], [2., .17, 0.], [4., .17, 0.], [6.31, .17, 0.]])
    before = path.copy()
    result = shortcut(path)
    np.testing.assert_array_equal(result, path[[0, -1]])
    np.testing.assert_array_equal(path, before)


def test_shortcut_can_remove_detour_only_with_tangent_endpoint_headings():
    path = curved_detour()
    result = shortcut(path)
    np.testing.assert_array_equal(result, path[[0, -1]])


def test_shortcut_rejects_line_crossing_obstacle():
    path = np.array([[0., 0., 0.], [3., 0., 0.], [6., 0., 0.]])
    np.testing.assert_array_equal(shortcut(path, scene([(3., 0., 254)])), path)


def test_shortcut_keeps_necessary_turning_samples():
    path = quarter_circle()
    np.testing.assert_array_equal(shortcut(path), path)


def test_shortcut_cannot_cut_across_blocked_detour():
    path = curved_detour()
    grid = scene([(4., 0., 254)])
    result = shortcut(path, grid)
    assert len(result) > 2
    np.testing.assert_array_equal(result[[0, -1]], path[[0, -1]])
    assert prepare(result, result[0], grid=grid) is not None


def test_shortcut_checks_full_hull_not_only_line_centre():
    path = curved_detour()
    grid = scene([(4., .5, 254)])
    result = shortcut(path, grid, options(footprint_length=2.2, footprint_width=1.2))
    assert len(result) > 2


def test_shortcut_budget_caps_real_expensive_checks(monkeypatch):
    pieces = [curved_detour((8.*index, 0., 0.))[:-1] for index in range(20)]
    path = np.vstack([*pieces, [[160., 0., 0.]]])
    grid = scene([(4.+8*index, 0., 254) for index in range(20)], resolution=.5)
    calls = []
    original = HybridCollisionChecker.motion_is_clear

    def counted(self, start, curvature, distance, deadline=math.inf):
        calls.append(distance)
        return original(self, start, curvature, distance, deadline=deadline)

    monkeypatch.setattr(HybridCollisionChecker, "motion_is_clear", counted)
    result = shortcut(path, grid)
    np.testing.assert_array_equal(result[[0, -1]], path[[0, -1]])
    assert 0 < len(calls) <= 16


def test_expired_shortcut_deadline_raises_timeout():
    with pytest.raises(TimeoutError):
        shortcut([[0., 0., 0.], [3., 0., 0.], [6., 0., 0.]], deadline=0.)
