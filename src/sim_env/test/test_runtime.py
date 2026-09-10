from concurrent.futures import Future
import numpy as np
import pytest
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from sim_env.configuration import resolve_parameters
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
    executor.jobs[0][1].set_result('old version result')
    assert worker.poll() == 'old version result'
    assert len(executor.jobs) == 2 and executor.jobs[1][0].version == 3
    worker.offer(request(4))
    worker.invalidate()
    assert worker.pending is None
    executor.jobs[1][1].set_result('cancelled task result')
    assert worker.poll() == 'cancelled task result'  # Caller must reject its task version.
    worker.close()
