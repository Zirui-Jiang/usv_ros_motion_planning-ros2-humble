"""The decision adapter must preserve the curved output of Hybrid A*."""
from types import SimpleNamespace

import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, PlanResult, VesselState
from usv_decision import runtime
from usv_navigation.configuration import resolve_parameters


def curved_request(monkeypatch, *, ordered=False):
    # A known radius-4 quarter circle isolates adapter behavior from search.
    angles = np.linspace(0., np.pi/2, 33)
    curve = np.column_stack((10.+4.*np.sin(angles), 14.-4.*np.cos(angles), angles))
    received = []

    class CurvedPlanner:
        def __init__(self, config):
            received.append(config.copy())
            self.planner = SimpleNamespace(preserves_kinematics=True,
                                          unknown_is_obstacle=True,
                                          outline_map=False, allow_corner_cutting=False)

        def prune(self, path, state):
            return np.asarray(path).copy()

        def make_plan(self, start, goal, grid):
            return PlanResult(AlgorithmStatus.SUCCESS, curve.copy(), expanded=3)

    monkeypatch.setattr(runtime, 'PathPlannerCore', CurvedPlanner)
    parameters = resolve_parameters({})
    parameters.update(planner_name='hybrid_astar')
    parameters['navigation.footprint_length'] = 2.6
    parameters['navigation.footprint_width'] = 1.4
    request = runtime.PlanningRequest(
        1, tuple(curve[-1]), VesselState(*curve[0], stamp=1.),
        GridSnapshot(np.zeros((60, 60), np.uint8), 1., stamp=1.),
        curve.copy(), parameters,
        waypoints=(tuple(curve[-1]) + (1.,),) if ordered else ())
    return request, curve, received


def test_hybrid_reuses_clear_cached_geometry_from_actual_pose(monkeypatch):
    request, curve, received = curved_request(monkeypatch)
    response = runtime.plan_navigation(request)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert not response.searched and response.expanded == 0
    np.testing.assert_array_equal(response.route, curve)


def test_hybrid_uses_navigation_hull_dimensions_in_the_search(monkeypatch):
    request, curve, received = curved_request(monkeypatch)
    runtime.plan_navigation(request)
    assert received[0]['footprint_length'] == 2.6
    assert received[0]['footprint_width'] == 1.4


def test_hybrid_ordered_leg_keeps_arc_samples_without_straight_shortcuts(monkeypatch):
    request, curve, received = curved_request(monkeypatch, ordered=True)
    response = runtime.plan_navigation(request)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    np.testing.assert_array_equal(response.route, curve)
    assert received[0]['footprint_length'] == 2.6
    assert received[0]['footprint_width'] == 1.4


@pytest.mark.parametrize('ordered', [False, True])
def test_actual_hybrid_search_runs_through_decision_and_los(ordered):
    """Use the real factory, geometry, hull checks and guidance, without doubles."""
    parameters = resolve_parameters({'planner_name': 'hybrid_astar',
                                     'navigation.planning_timeout': 4.})
    goal = (4., 4., np.pi/2)
    grid = GridSnapshot(np.zeros((120, 120), np.uint8), .25, -5., -5., stamp=1.)
    request = runtime.PlanningRequest(
        7, goal, VesselState(stamp=1.), grid, np.empty((0, 3)), parameters,
        waypoints=(goal+(1.,),) if ordered else ())
    response = runtime.plan_navigation(request)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert response.searched and response.expanded > 0 and response.version == 7
    assert len(response.route) > 20
    np.testing.assert_allclose(response.route[[0, -1]], [[0., 0., 0.], goal], atol=1e-8)
    np.testing.assert_allclose(np.linalg.norm(response.route[:, :2]-[0., 4.], axis=1),
                               4., atol=1e-7)
    assert len(response.trajectory.points) > 2
    assert response.trajectory.points[-1, 1] > 1.
    assert response.trajectory.goal == goal[:2]
