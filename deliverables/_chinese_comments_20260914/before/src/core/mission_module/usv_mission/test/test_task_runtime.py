from concurrent.futures import Future
from dataclasses import replace
import numpy as np
import pytest
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_mission.configuration import C, resolve_parameters
from usv_mission.runtime import LatestPlanWorker, PlanningRequest, fresh, plan_navigation, readiness


def request(version=1):
    return PlanningRequest(version, (18.5, 10.5, 0.),
                           VesselState(2.5, 10.5, 0., stamp=10.),
                           GridSnapshot(np.zeros((30, 30), np.uint8), 1., stamp=10.),
                           np.empty((0, 3)), resolve_parameters({}))


def test_complete_offline_chain_and_route_reuse():
    job = request()
    result = plan_navigation(job)
    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert result.searched and len(result.route) and len(result.trajectory)
    assert result.trajectory.stamp == 10.
    assert result.trajectory.goal == job.goal[:2]
    repeated = plan_navigation(PlanningRequest(2, job.goal, job.state, job.grid, result.route, job.parameters))
    assert repeated.status == AlgorithmStatus.SUCCESS
    assert not repeated.searched


def soft_escape_request():
    job = request()
    data = job.grid.data.copy()
    data[:, 2:6] = [120, 100, 75, 50]
    return replace(job, grid=GridSnapshot(data, 1., stamp=10.))


def test_legal_soft_escape_route_is_reused_without_a_second_global_search():
    job = soft_escape_request()
    first = plan_navigation(job)
    assert first.status == AlgorithmStatus.SUCCESS and first.searched
    second = plan_navigation(replace(job, route=first.route))
    assert second.status == AlgorithmStatus.SUCCESS
    assert not second.searched and second.expanded == 0
    np.testing.assert_array_equal(second.route, first.route)


@pytest.mark.parametrize('new_cost', [120, 121, 254, 255])
def test_reused_escape_route_rechecks_latest_map_plateau_uphill_hard_and_unknown(new_cost):
    job = soft_escape_request()
    first = plan_navigation(job)
    assert first.status == AlgorithmStatus.SUCCESS
    changed = job.grid.data.copy()
    changed[:, 3] = new_cost
    latest = replace(job, route=first.route, grid=GridSnapshot(changed, 1., stamp=10.2))
    repeated = plan_navigation(latest)
    assert repeated.searched
    assert repeated.status == AlgorithmStatus.NO_PATH
    assert repeated.trajectory.empty()


def test_reused_escape_route_checks_connection_from_the_latest_vessel_position():
    job = soft_escape_request()
    first = plan_navigation(job)
    # The map and route are unchanged, but moving to the free cell west of
    # the inflated band would require re-entering high cost to reuse it.
    latest = replace(job, route=first.route, state=replace(job.state, x=1.5, stamp=10.2))
    repeated = plan_navigation(latest)
    assert repeated.searched and repeated.status == AlgorithmStatus.NO_PATH


def test_failure_is_explicit_and_has_no_reference():
    job = request()
    grid = job.grid.data.copy()
    grid[:, 12] = 254
    response = plan_navigation(PlanningRequest(1, job.goal, job.state,
                              GridSnapshot(grid, 1., stamp=10.), job.route, job.parameters))
    assert response.status == AlgorithmStatus.NO_PATH
    assert response.trajectory.empty()


def test_default_pid_keeps_straight_grid_endpoint_without_a_goal_kink():
    job = PlanningRequest(1, (18.125, 10.125, 0.), VesselState(14.125, 10.125, 0., stamp=10.),
                          GridSnapshot(np.zeros((60, 60), np.uint8), .5, stamp=10.),
                          np.empty((0, 3)), resolve_parameters({}))
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS
    np.testing.assert_allclose(response.route[:, 1], 10.25)
    np.testing.assert_array_equal(response.route[-1], [18.25, 10.25, 0.])
    assert response.trajectory.goal == job.goal[:2]
    # PID's 1.5 m actual-goal disk still contains the original 1 m LOS disk.
    terminal = response.trajectory.points[-1, 2:4]
    assert .9 < np.linalg.norm(terminal - [18.25, 10.25]) <= 1.
    assert np.linalg.norm(terminal - job.goal[:2]) <= 1.5


def test_los_stop_disk_does_not_stop_lqr_outside_the_requested_goal():
    cfg = resolve_parameters({C+'controller_name': 'lqr'})
    job = PlanningRequest(1, (10.49, 10.49, 0.), VesselState(9.3, 10.25, 0., stamp=10.),
                          GridSnapshot(np.zeros((40, 40), np.uint8), .5, stamp=10.),
                          np.empty((0, 3)), cfg)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS
    assert np.any(response.trajectory.points[:, 6] > 0.)
    np.testing.assert_array_equal(response.route[-1], [10.25, 10.25, 0.])
    assert response.trajectory.goal == job.goal[:2]
    terminal = response.trajectory.points[-1, 2:4]
    assert np.linalg.norm(terminal - job.goal[:2]) <= 1.
    # Merely keeping the 1 m LOS disk would stop this state at distance .95
    # from its centre even though the actual goal remains over 1.2 m away.
    assert response.trajectory.points[-1, 6] == 0.


@pytest.mark.parametrize('tolerance', [.1, .125])
def test_small_or_equal_tolerance_appends_goal_without_losing_grid_endpoint(tolerance):
    cfg = resolve_parameters({C+'pid_controller.goal_distance_tolerance': tolerance})
    job = PlanningRequest(1, (10.375, 10.25, 0.), VesselState(8.25, 10.25, 0., stamp=10.),
                          GridSnapshot(np.zeros((40, 40), np.uint8), .5, stamp=10.),
                          np.empty((0, 3)), cfg)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS
    np.testing.assert_array_equal(response.route[-2], [10.25, 10.25, 0.])
    np.testing.assert_array_equal(response.route[-1], job.goal)
    assert response.trajectory.goal == job.goal[:2]
    assert np.linalg.norm(response.trajectory.points[-1, 2:4] - job.goal[:2]) <= tolerance
    repeated = plan_navigation(replace(job, route=response.route))
    assert repeated.status == AlgorithmStatus.SUCCESS and not repeated.searched
    np.testing.assert_array_equal(repeated.route, response.route)


@pytest.mark.parametrize('blocked_cost', [75, 254, 255])
def test_required_goal_connection_checks_soft_hard_and_unknown_cells(blocked_cost):
    job = request()
    data = job.grid.data.copy()
    data[10, 6] = blocked_cost
    # A cached prefix itself is valid, but its required goal extension crosses
    # another cell. Soft cost 75 is physically clear but exceeds factor .25.
    job = replace(job, goal=(8.5, 10.5, 0.),
                  route=np.array([[2.5, 10.5, 0.], [4.5, 10.5, 0.]]),
                  grid=GridSnapshot(data, 1., stamp=10.))
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.NO_PATH
    assert response.trajectory.empty()
    assert 'goal connection' in response.message
    np.testing.assert_array_equal(response.route, job.route)


@pytest.mark.parametrize('stamp,now,timeout,expected', [(10.,10.4,.5,True), (10.,10.6,.5,False), (11.,10.,.5,False), (float('nan'),10.,1.,False)])
def test_freshness(stamp, now, timeout, expected):
    assert fresh(stamp, now, timeout) == expected


def test_input_gate_does_not_reuse_stale_trajectory():
    job = request()
    plan = plan_navigation(job)
    assert readiness(10.1, job.state, job.grid, plan.trajectory, job.parameters) == 'NAVIGATING'
    assert readiness(11., job.state, job.grid, plan.trajectory, job.parameters) == 'WAITING_STATE'
    assert readiness(10.1, job.state, None, plan.trajectory, job.parameters) == 'WAITING_MAP'
    assert readiness(10.1, job.state, job.grid, None, job.parameters) == 'WAITING_REFERENCE'


def test_worker_keeps_only_latest_pending_and_invalidates():
    class Executor:
        def __init__(self):
            self.jobs = []
        def submit(self, fn, job):
            future = Future()
            future.set_running_or_notify_cancel()
            self.jobs.append((job, future))
            return future
        def shutdown(self, **kwargs):
            pass
    executor = Executor()
    worker = LatestPlanWorker(executor)
    worker.offer(request(1))
    worker.offer(request(2))
    worker.offer(request(3))
    assert len(executor.jobs) == 1 and worker.pending.version == 3 and worker.replaced == 1
    old_result = plan_navigation(request(1))
    executor.jobs[0][1].set_result(old_result)
    assert worker.poll() is old_result
    assert len(executor.jobs) == 2 and executor.jobs[1][0].version == 3
    assert not len(executor.jobs[1][0].route)  # Never copy an old goal's route.
    worker.offer(request(4))
    worker.invalidate()
    assert worker.pending is None
    cancelled_result = plan_navigation(request(3))
    executor.jobs[1][1].set_result(cancelled_result)
    assert worker.poll() is cancelled_result  # Caller must reject its task version.
    worker.close()


def test_first_search_refreshes_pending_route_but_keeps_latest_inputs():
    class Executor:
        def __init__(self):
            self.jobs = []
        def submit(self, fn, job):
            future = Future()
            future.set_running_or_notify_cancel()
            self.jobs.append((job, future))
            return future
    executor = Executor()
    worker = LatestPlanWorker(executor)
    first = request()
    latest = replace(first, state=replace(first.state, x=2.4, stamp=10.2))
    worker.offer(first)
    worker.offer(latest)
    completed = plan_navigation(first)
    executor.jobs[0][1].set_result(completed)
    assert worker.poll() is completed
    dispatched = executor.jobs[1][0]
    assert dispatched.state is latest.state and dispatched.grid is latest.grid
    assert dispatched.parameters is latest.parameters
    np.testing.assert_array_equal(dispatched.route, completed.route)
    assert not np.shares_memory(dispatched.route, completed.route)
    repeated = plan_navigation(dispatched)
    assert repeated.status == AlgorithmStatus.SUCCESS
    assert not repeated.searched


def test_pending_route_is_rechecked_against_latest_obstacles():
    first = request()
    completed = plan_navigation(first)
    blocked = first.grid.data.copy()
    blocked[:, 12] = 254
    latest = replace(first, route=completed.route,
                     grid=GridSnapshot(blocked, 1., stamp=10.2))
    response = plan_navigation(latest)
    assert response.searched and response.status == AlgorithmStatus.NO_PATH
