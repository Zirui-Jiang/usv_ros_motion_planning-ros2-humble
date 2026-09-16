"""Measured waypoint passage discards only the completed phase's route cache."""
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from trajectory_planner.trajectory import Trajectory
from usv_decision import decision as decision_module
from usv_decision.route_progress import RouteProgress
from usv_decision.runtime import plan_navigation
from usv_navigation.configuration import T


class RecordingWorker:
    """Keep requests local: this boundary does not start background planners."""

    def __init__(self):
        self.requests = []
        self.invalidations = 0

    def offer(self, request):
        self.requests.append(request)

    def invalidate(self):
        self.invalidations += 1

    def close(self):
        pass


@pytest.fixture
def phase(monkeypatch):
    worker, references, clock = RecordingWorker(), [], [100.]
    monkeypatch.setattr(decision_module, 'LatestPlanWorker', lambda: worker)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/route_phase_cache_test'])
    node = decision_module.DecisionNode()
    monkeypatch.setattr(node, 'now_seconds', lambda: clock[0])
    monkeypatch.setattr(node, 'reference_pub', SimpleNamespace(publish=references.append))
    node.config.update(planner_name='hybrid_astar')
    node.config['navigation.planning_timeout'] = 4.
    node.config[T+'los.trajectory_time'] = 10.
    points = ((10., 0., 0., .8), (14., 10., np.pi/2, .8))
    node.route_progress = RouteProgress(points, 1.5, max_speed=2.1, frame_id=node.frame)
    node.goal, node.desired_speed_mps = points[0][:3], points[0][3]
    node.state = VesselState(7.8, 0., 0., u=.4, stamp=clock[0], frame_id=node.frame)
    node.grid = GridSnapshot(np.zeros((160, 160), np.uint8), .5, -30., -30.,
                             stamp=clock[0], frame_id=node.frame)
    # Valid old geometry: straight to A, radius-4 left quarter-circle, then north.
    node.route = np.array([[7.8, 0., 0.], [10., 0., 0.],
                           [14., 4., np.pi/2], [14., 10., np.pi/2]])
    node.trajectory = Trajectory([[0., 0., 7.8, 0., 0., 0., .4, 0.],
                                  [.5, .2, 8., 0., 0., 0., .4, 0.]],
                                 clock[0], node.frame, None)
    node.mode = 'NAVIGATING'
    node._update_route_progress()
    try:
        yield SimpleNamespace(node=node, worker=worker, references=references, clock=clock)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def move(phase, pose, stamp):
    phase.clock[0] = stamp
    phase.node.state = VesselState(*pose, u=.4, stamp=stamp, frame_id=phase.node.frame)
    phase.node._update_route_progress()


def test_unpassed_anchor_keeps_cache_and_does_not_invalidate_worker(phase):
    node, worker = phase.node, phase.worker
    route, trajectory, revision = node.route, node.trajectory, node.version

    move(phase, (8.2, 0., 0.), 100.5)
    move(phase, (8.3, 0., 0.), 100.7)

    assert node.route_progress.index == 0
    assert node.route is route and node.trajectory is trajectory
    assert node.version == revision
    assert worker.invalidations == 0 and worker.requests == []


@pytest.mark.parametrize('pose', [(8.6, 0., 0.), (9.5, 1., np.pi/3), (10.4, 0., .1)])
def test_confirmed_passage_clears_old_cache_but_keeps_safe_pass_through_reference(phase, pose):
    node, worker = phase.node, phase.worker
    trajectory, revision, task_id = node.trajectory, node.version, node.task_id

    move(phase, pose, 101.)

    assert node.route_progress.index == 1
    assert node.route.shape == (0, 3)
    assert node.version == revision+1 and node.task_id == task_id
    assert node.trajectory is trajectory
    assert worker.invalidations == 1 and len(worker.requests) == 1
    request = worker.requests[0]
    assert request.route.shape == (0, 3)
    assert request.state is node.state and request.version == node.version
    assert request.goal == (14., 10., np.pi/2)
    assert request.waypoints == ((14., 10., np.pi/2, .8),)
    reference = phase.references[-1]
    assert reference.active and reference.pass_through
    assert reference.state == 'NAVIGATING'
    assert reference.revision == node.version and reference.task_id == task_id
    assert node._terminal_reference_sequence is None


def test_new_phase_searches_once_then_regular_updates_reuse_its_new_cache(phase):
    node, worker = phase.node, phase.worker
    move(phase, (9.5, 1., np.pi/3), 101.)
    assert len(worker.requests) == 1
    assert worker.requests[0].route.shape == (0, 3)
    first = plan_navigation(worker.requests[0])
    assert first.status == AlgorithmStatus.SUCCESS, first.message
    assert first.searched and first.expanded > 0
    np.testing.assert_allclose(first.route[0], node.state.pose, atol=1e-10)
    assert not np.any(np.linalg.norm(first.route[:, :2]-(10., 0.), axis=1) < 1e-7)

    node.route, node.trajectory = first.route, first.trajectory
    revision, invalidations = node.version, worker.invalidations
    for stamp in (101.2, 101.4, 101.6):
        route = node.route
        request_count = len(worker.requests)
        # No new measured anchor passage: progress must not clear the new cache.
        move(phase, (9.5, 1., np.pi/3), stamp)
        assert node.route is route and node.version == revision
        assert worker.invalidations == invalidations
        assert len(worker.requests) == request_count
        node.grid = GridSnapshot(node.grid.data, .5, -30., -30.,
                                 stamp=stamp, frame_id=node.frame)
        node._planning()
        np.testing.assert_array_equal(worker.requests[-1].route, route)
        following = plan_navigation(worker.requests[-1])
        assert following.status == AlgorithmStatus.SUCCESS, following.message
        assert not following.searched and following.expanded == 0
        node.route, node.trajectory = following.route, following.trajectory


def test_single_goal_without_route_progress_keeps_its_cache(phase):
    node, worker = phase.node, phase.worker
    node.route_progress = None
    route, trajectory, revision = node.route, node.trajectory, node.version
    move(phase, (9.5, 1., np.pi/3), 101.)
    assert node.route is route and node.trajectory is trajectory
    assert node.version == revision and worker.invalidations == 0
    assert worker.requests == []
