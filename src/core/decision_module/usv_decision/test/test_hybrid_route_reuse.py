"""Global geometry is cached; only fresh local references update every cycle."""
from dataclasses import replace

import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_decision.runtime import PlanningRequest, plan_navigation
from usv_navigation.configuration import resolve_parameters


def request(*, ordered=False):
    cfg = resolve_parameters({'planner_name': 'hybrid_astar', 'navigation.planning_timeout': 4.})
    goals = ((12.5, 10.5, 0., .7), (20.5, 10.5, 0., .9)) if ordered else ()
    return PlanningRequest(1, goals[0][:3] if goals else (20.5, 10.5, 0.),
        VesselState(2.5, 10.5, 0., stamp=10.),
        GridSnapshot(np.zeros((60, 80), np.uint8), .5, stamp=10.),
        np.empty((0, 3)), cfg, waypoints=goals)


def test_small_tracking_error_reuses_global_geometry_and_refreshes_local_reference():
    job = request()
    first = plan_navigation(job)
    assert first.status == AlgorithmStatus.SUCCESS and first.searched
    state = replace(job.state, x=4.5, y=10.65, psi=.05, stamp=10.2)
    second = plan_navigation(replace(job, state=state, route=first.route))
    assert second.status == AlgorithmStatus.SUCCESS, second.message
    assert not second.searched and second.expanded == 0
    assert second.trajectory.stamp == 10.2
    np.testing.assert_allclose(second.trajectory.points[0, 2:5], state.pose)
    # Global reference stays on its original line, not a new pose-to-route kink.
    np.testing.assert_allclose(second.route[:, 1], 10.5)
    np.testing.assert_allclose(second.route[:, 2], 0.)
    assert second.route[0, 0] > first.route[0, 0]
    np.testing.assert_allclose(second.route[-1], job.goal)


def test_unrelated_map_change_does_not_trigger_global_search():
    job = request()
    first = plan_navigation(job)
    data = job.grid.data.copy()
    data[45, 55] = 254
    second = plan_navigation(replace(job, route=first.route, grid=GridSnapshot(data,.5,stamp=10.2)))
    assert second.status == AlgorithmStatus.SUCCESS
    assert not second.searched


def test_blocked_remaining_route_replans_and_never_reuses_unsafe_geometry():
    job = request()
    first = plan_navigation(job)
    data = job.grid.data.copy()
    data[:, 24] = 254
    cfg = dict(job.parameters, **{'navigation.max_expansions': 200})
    second = plan_navigation(replace(job, route=first.route, grid=GridSnapshot(data,.5,stamp=10.2),parameters=cfg))
    assert second.searched
    assert second.status in (AlgorithmStatus.NO_PATH, AlgorithmStatus.TIMEOUT)
    assert second.trajectory.empty()


def test_large_tracking_error_triggers_global_replan():
    job = request()
    first = plan_navigation(job)
    second = plan_navigation(replace(job, route=first.route,
                                      state=replace(job.state, y=14.5, stamp=10.2)))
    assert second.searched


def test_simplified_long_line_remains_reusable_after_more_than_twenty_metres():
    job = replace(request(), goal=(34.5, 10.5, 0.))
    first = plan_navigation(job)
    assert first.status == AlgorithmStatus.SUCCESS
    route = first.route
    for index, x in enumerate((5.5, 8.5, 11.5, 14.5, 17.5, 20.5, 23.5, 26.5)):
        state = replace(job.state, x=x, stamp=11.+index)
        result = plan_navigation(replace(job, route=route, state=state))
        assert result.status == AlgorithmStatus.SUCCESS, result.message
        assert not result.searched
        assert result.trajectory.stamp == state.stamp
        route = result.route


def test_ordered_cached_route_keeps_mandatory_anchors_and_updates_speed_reference():
    job = request(ordered=True)
    first = plan_navigation(job)
    assert first.status == AlgorithmStatus.SUCCESS
    second = plan_navigation(replace(job,route=first.route,state=replace(job.state,x=4.5,stamp=10.2)))
    assert second.status == AlgorithmStatus.SUCCESS, second.message
    assert not second.searched and second.expanded == 0
    assert np.any(np.all(np.isclose(second.route[:,:2],job.waypoints[0][:2]),axis=1))
    assert second.trajectory.goal is None
    # Decision advances the task revision and drops the completed phase's cache once.
    next_leg = replace(job, version=2, goal=job.waypoints[1][:3], waypoints=job.waypoints[1:],
                       route=np.empty((0, 3)), state=replace(job.state,x=12.5,stamp=20.))
    third = plan_navigation(next_leg)
    assert third.status == AlgorithmStatus.SUCCESS, third.message
    assert third.searched
    assert third.trajectory.goal == job.waypoints[-1][:2]
    # Normal updates within the new phase reuse the newly established route again.
    fourth = plan_navigation(replace(next_leg, route=third.route,
                                     state=replace(next_leg.state, x=12.7, stamp=20.2)))
    assert fourth.status == AlgorithmStatus.SUCCESS, fourth.message
    assert not fourth.searched


@pytest.mark.parametrize('value', [0., -1., float('nan')])
@pytest.mark.parametrize('name', ['reuse_max_deviation','reuse_max_heading_error','reuse_search_distance'])
def test_invalid_reuse_configuration_is_rejected_at_startup(name,value):
    with pytest.raises(ValueError):
        resolve_parameters({'planner_name':'hybrid_astar','path_planner.'+name:value})
