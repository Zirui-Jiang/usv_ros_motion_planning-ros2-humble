from concurrent.futures import Future
from dataclasses import replace
import numpy as np
import pytest
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from sim_env.configuration import C, resolve_parameters
from sim_env.runtime import LatestPlanWorker, PlanningRequest, fresh, plan_navigation, readiness


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


def test_failure_is_explicit_and_has_no_reference():
    job = request()
    grid = job.grid.data.copy()
    grid[:, 12] = 254
    response = plan_navigation(PlanningRequest(1, job.goal, job.state,
                              GridSnapshot(grid, 1., stamp=10.), job.route, job.parameters))
    assert response.status == AlgorithmStatus.NO_PATH
    assert response.trajectory.empty()


def test_los_reaches_requested_goal_instead_of_nearby_cell_centre():
    cfg = resolve_parameters({C+'controller_name': 'lqr'})
    job = PlanningRequest(1, (10.49, 10.49, 0.), VesselState(9.3, 10.25, 0., stamp=10.),
                          GridSnapshot(np.zeros((40, 40), np.uint8), .5, stamp=10.),
                          np.empty((0, 3)), cfg)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS
    assert np.any(response.trajectory.points[:, 6] > 0.)
    np.testing.assert_array_equal(response.route[-1], job.goal)


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
