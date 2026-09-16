"""Ordered mandatory anchors, continuous speed and bounded route planning."""
import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_decision.runtime import PlanningRequest, plan_navigation
from usv_navigation.configuration import resolve_parameters, T


def request(points, state=None, **changes):
    cfg = resolve_parameters({'dynamics.velocity_max': 3.,
                              'dynamics.acceleration_min': -.3,
                              'dynamics.acceleration_max': .25})
    values = dict(version=1, goal=points[0][:3],
                  state=state or VesselState(5., 10., stamp=1.),
                  grid=GridSnapshot(np.zeros((80, 100), np.uint8), 1., stamp=1.),
                  route=np.empty((0, 3)), parameters=cfg)
    values.update(changes)
    return PlanningRequest(**values, waypoints=tuple(points), waypoint_radius=1.5)


def test_intermediate_anchor_is_crossed_without_terminal_goal_or_stop():
    result = plan_navigation(request(((12., 10., 0., 1.), (35., 10., 0., 1.))))
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert result.trajectory.goal is None
    rows = result.trajectory.points
    near = np.linalg.norm(rows[:, 2:4] - (12., 10.), axis=1) < 1.5
    assert near.any()
    assert rows[near, 6].min() > .1
    assert rows[-1, 2] > 14.


def test_route_retains_current_anchor_even_if_old_route_is_nearer_later_leg():
    points = ((8., 10., 0., 1.), (30., 10., 0., 1.))
    result = plan_navigation(request(points, state=VesselState(20., 10., np.pi),
                                    route=np.array([[20., 10., 0.], [30., 10., 0.]])))
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert np.any(np.all(np.isclose(result.route[:, :2], (8., 10.)), axis=1))


def test_unreachable_current_anchor_cannot_be_skipped():
    grid = np.zeros((80, 100), np.uint8)
    grid[10, 12] = 254
    result = plan_navigation(request(((12.1, 10.1, 0., 1.), (35., 10., 0., 1.)),
                                    grid=GridSnapshot(grid, 1.)))
    assert result.status == AlgorithmStatus.NO_PATH


@pytest.mark.parametrize('points', [((20., 10., 0., 1.),),
    ((12., 10., 0., 1.), (12., 10., 0., 1.), (20., 10., 0., 1.))])
def test_final_prediction_stops_and_duplicate_points_are_finite(points):
    req = request(points)
    req.parameters[T + 'los.trajectory_time'] = 60.
    result = plan_navigation(req)
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert np.isfinite(result.trajectory.points).all()
    assert result.trajectory[-1].velocity <= .01
    assert np.linalg.norm(result.trajectory.points[-1, 2:4] - (20., 10.)) < 1.5
    assert result.trajectory.goal == ((20., 10.) if len(points) == 1 else None)


def test_speed_profile_uses_model_acceleration_bounds():
    points = ((15., 10., 0., 2*1852/3600), (40., 10., 0., 4*1852/3600),
              (65., 10., 0., 2*1852/3600))
    req = request(points)
    req.parameters[T + 'los.trajectory_time'] = 60.
    result = plan_navigation(req)
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    rows = result.trajectory.points
    slopes = np.diff(rows[:, 6]) / np.diff(rows[:, 0])
    assert slopes.max() <= .25 + 1e-8
    assert slopes.min() >= -.3 - 1e-8
    assert rows[0, 6] == 0.
    assert rows[(rows[:, 2] > 22.) & (rows[:, 2] < 32.), 6].max() > 1.5


def test_ninety_degree_turn_decelerates_before_anchor():
    result = plan_navigation(request(((15., 10., 0., 2.), (15., 35., 0., 2.))))
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    rows = result.trajectory.points
    near = np.linalg.norm(rows[:, 2:4] - (15., 10.), axis=1) < 2.
    assert near.any()
    assert rows[near, 6].max() < .6
    assert np.max(np.abs(rows[:, 5] * rows[:, 6])) <= .3 + 1e-8


def test_exact_off_center_anchors_are_retained_in_order():
    points = ((12.17, 10.31, 0., 1.), (20.72, 15.69, 0., 1.))
    result = plan_navigation(request(points))
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    matches = [np.flatnonzero(np.all(result.route[:, :2] == p[:2], axis=1)) for p in points]
    assert all(len(indices) for indices in matches)
    assert matches[0][0] < matches[1][0]


def test_many_repeated_anchors_have_bounded_prefix_and_no_final_goal():
    result = plan_navigation(request(((12., 10., 0., 1.),)*255))
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert len(result.route) <= 64
    assert result.trajectory.goal is None


def test_expired_total_budget_returns_timeout():
    req = request(((12., 10., 0., 1.), (20., 10., 0., 1.)))
    req.parameters['navigation.planning_timeout'] = 1e-12
    result = plan_navigation(req)
    assert result.status == AlgorithmStatus.TIMEOUT


@pytest.mark.parametrize('next_point', [(150., 10., 0., 1.), (30., 10., 0., 1.)])
def test_unavailable_preview_anchor_keeps_safe_current_prefix(next_point):
    grid = np.zeros((80, 100), np.uint8)
    grid[10, 30] = 254
    points = ((12., 10., 0., 1.), next_point, (20., 10., 0., 1.))
    req = request(points, grid=GridSnapshot(grid, 1.))
    req.parameters[T+'los.trajectory_time'] = 60.
    result = plan_navigation(req)
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert result.trajectory.goal is None
    assert np.array_equal(result.route[-1, :2], points[0][:2])
    assert not np.any(np.all(result.route[:, :2] == points[2][:2], axis=1))
    assert result.trajectory[-1].velocity <= .01
    assert np.linalg.norm(result.trajectory.points[-1, 2:4]-points[0][:2]) < 1.5
    assert 'prefix' in result.message

    # Advancing measured progress makes that unavailable point mandatory;
    # the same failure must now reject the reference, not skip to point 3.
    retry = request(points[1:], state=VesselState(11., 10., stamp=2.), grid=req.grid)
    failed = plan_navigation(retry)
    assert failed.status in (AlgorithmStatus.NO_PATH, AlgorithmStatus.INVALID_INPUT)
    assert failed.trajectory.empty()
