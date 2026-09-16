"""Publish accepted remaining-route changes without starting global searches."""
from concurrent.futures import Future
from copy import deepcopy
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('usv_navigation_interfaces')

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from trajectory_planner.trajectory import Trajectory
from usv_decision.decision import DecisionNode
from usv_decision.runtime import PlanningResponse
from usv_navigation_interfaces.msg import ControlReport


class PublicationLog:
    """Record payloads while still using the node's real ROS publisher."""
    def __init__(self, publisher):
        self.publisher = publisher
        self.messages = []

    def publish(self, message):
        self.publisher.publish(message)
        self.messages.append(deepcopy(message))


@pytest.fixture
def decision(request):
    planner = getattr(request, 'param', 'astar')
    rclpy.init(args=['--ros-args', '-r', '__ns:=/global_path_publication_test',
                    '-p', 'planner_name:=' + planner])
    node = DecisionNode()
    try:
        for name in ('path_pub', 'trajectory_pub', 'reference_pub'):
            setattr(node, name, PublicationLog(getattr(node, name)))
        goal = PoseStamped()
        goal.header.frame_id = node.frame
        goal.pose.position.x = 8.
        goal.pose.orientation.w = 1.
        # Accept before supplying inputs, so no live worker job is submitted.
        node._on_goal(goal)
        for publisher in (node.path_pub, node.trajectory_pub, node.reference_pub):
            publisher.messages.clear()
        node.state = VesselState(stamp=node.now_seconds(), frame_id=node.frame)
        node.grid = free_grid(node)
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def free_grid(node, *, blocked=False):
    data = np.zeros((40, 40), dtype=np.uint8)
    if blocked:
        data[10, 14] = 254  # x=2..2.5, y=0..0.5: blocks the rollout, not its start.
    return GridSnapshot(data, .5, -5., -5., node.now_seconds(), node.frame)


def consume(node, *, searched=True, route=None, status=AlgorithmStatus.SUCCESS,
            version=None):
    route = np.array([[0., 0., 0.], [4., 0., 0.], [8., 0., 0.]]
                     if route is None else route, dtype=float)
    trajectory = Trajectory(
        [[0., 0., node.state.x, 0., 0., 0., 1., 0.],
         [4., 4., node.state.x + 4., 0., 0., 0., 1., 0.]],
        node.state.stamp, node.frame, node.goal[:2])
    # Inject a completed process boundary result; node acceptance and ROS
    # publication stay real, without running a second planner or simulator.
    future = Future()
    future.set_result(PlanningResponse(
        node.version if version is None else version, status, route, trajectory,
        .01, searched=searched))
    node.worker.future = future
    node._consume_plan()


def path_xy(message):
    return [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]


def test_unchanged_cached_route_still_updates_local_trajectory_and_reference(decision):
    consume(decision)
    assert path_xy(decision.path_pub.messages[-1]) == [(0., 0.), (4., 0.), (8., 0.)]
    for x in (1., 2.):
        decision.state = VesselState(x=x, stamp=decision.now_seconds(), frame_id=decision.frame)
        consume(decision, searched=False)
    assert len(decision.path_pub.messages) == 1
    assert len(decision.trajectory_pub.messages) == 3
    assert len(decision.reference_pub.messages) == 3
    assert path_xy(decision.trajectory_pub.messages[-1])[0] == (2., 0.)
    assert decision.reference_pub.messages[-1].trajectory_stamp == decision.state.stamp
    assert decision.reference_pub.messages[-1].points


@pytest.mark.parametrize('decision', ['astar', 'hybrid_astar'], indirect=True)
def test_each_completed_cache_trim_immediately_updates_display_without_search(decision):
    consume(decision)
    source_stamp = decision.state.stamp
    # Consecutive completed local cycles need no display timer or wall sleep.
    for offset, x, expected_publications in ((.2, 1., 2), (.4, 2., 3), (.6, 3., 4)):
        decision.state = VesselState(x=x, stamp=source_stamp + offset, frame_id=decision.frame)
        consume(decision, searched=False, route=[[x, 0., 0.], [4., 0., 0.], [8., 0., 0.]])
        assert len(decision.path_pub.messages) == expected_publications
        assert path_xy(decision.path_pub.messages[-1])[0] == (x, 0.)
    assert path_xy(decision.path_pub.messages[-1]) == [(3., 0.), (4., 0.), (8., 0.)]
    assert all(message.poses for message in decision.path_pub.messages)
    assert len(decision.trajectory_pub.messages) == 4
    assert len(decision.reference_pub.messages) == 4
    assert decision.reference_pub.messages[-1].trajectory_stamp == decision.state.stamp
    assert decision.counters['global_searches'] == 1
    consume(decision, searched=False, route=[[3., 0., 0.], [4., 0., 0.], [8., 0., 0.]])
    assert len(decision.path_pub.messages) == 4
    consume(decision, searched=False, route=[[3.5, 0., 0.], [4., 0., 0.], [8., 0., 0.]])
    assert len(decision.path_pub.messages) == 5
    assert path_xy(decision.path_pub.messages[-1])[0] == (3.5, 0.)
    assert decision.counters['global_searches'] == 1


def test_fresh_search_publishes_only_an_actual_route_replacement(decision):
    consume(decision)
    consume(decision)
    replacement = [[0., 0., 0.], [4., 1., 0.], [8., 0., 0.]]
    consume(decision, route=replacement)
    assert len(decision.path_pub.messages) == 2
    assert path_xy(decision.path_pub.messages[-1]) == [(0., 0.), (4., 1.), (8., 0.)]
    assert len(decision.trajectory_pub.messages) == 3


def test_late_subscriber_receives_latest_trimmed_path_without_republishing(decision):
    consume(decision)
    consume(decision, searched=False, route=[[2., 0., 0.], [4., 0., 0.], [8., 0., 0.]])
    received = []
    listener = Node('late_global_path_reader')
    try:
        listener.create_subscription(
            Path, decision.path_pub.publisher.topic_name, received.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + 2.5
        while not received and time.monotonic() < deadline:
            # Only spin the late reader: the decision timers cannot generate
            # another path that would conceal a missing retained publication.
            rclpy.spin_once(listener, timeout_sec=.02)
        assert received, 'Late reader did not receive the retained global route'
        assert path_xy(received[-1]) == [(2., 0.), (4., 0.), (8., 0.)]
        assert len(decision.path_pub.messages) == 2
    finally:
        listener.destroy_node()


@pytest.mark.parametrize('status', [AlgorithmStatus.TIMEOUT, AlgorithmStatus.NO_PATH])
def test_failed_plan_clears_visible_global_path_once(decision, status):
    consume(decision)
    consume(decision, route=[], status=status)
    consume(decision, route=[], status=status)
    assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0]
    assert not len(decision.route)
    assert decision.trajectory is None
    assert not decision.reference_pub.messages[-1].points


def test_local_failure_retains_global_candidate_but_clears_display_and_reference(decision):
    consume(decision)
    cached_route = [[1., 0., 0.], [4., 0., 0.], [8., 0., 0.]]
    # A validated global route is still useful when LOS/swept-hull validation
    # alone fails. Keeping a candidate is not permission to execute it.
    consume(decision, searched=False, route=cached_route, status=AlgorithmStatus.NO_PATH)
    consume(decision, searched=False, route=cached_route, status=AlgorithmStatus.NO_PATH)
    np.testing.assert_array_equal(decision.route, cached_route)
    assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0]
    assert decision.trajectory is None
    assert not decision.reference_pub.messages[-1].points


def test_failure_before_first_route_does_not_repeat_empty_paths(decision):
    consume(decision, route=[], status=AlgorithmStatus.TIMEOUT)
    consume(decision, route=[], status=AlgorithmStatus.TIMEOUT)
    assert decision.path_pub.messages == []
    assert not decision.reference_pub.messages[-1].points


def test_valid_pending_cache_restores_global_path_once_after_failure(decision):
    consume(decision)
    consume(decision, route=[], status=AlgorithmStatus.TIMEOUT)
    # A previously queued request can still return a revalidated cached route
    # after the failed result cleared the display.
    consume(decision, searched=False)
    consume(decision, searched=False)
    assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0, 3]
    assert decision.reference_pub.messages[-1].points


def test_latest_map_rejection_never_publishes_the_rejected_route(decision):
    consume(decision)
    decision.grid = free_grid(decision, blocked=True)
    consume(decision, route=[[0., 0., 0.], [3., 1., 0.], [8., 0., 0.]])
    consume(decision, route=[[0., 0., 0.], [3., 1., 0.], [8., 0., 0.]])
    assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0]
    assert decision.mode == 'WAITING_REFERENCE'
    assert decision.trajectory is None
    assert not decision.reference_pub.messages[-1].points


def test_map_invalidation_clears_visible_path_before_waiting_for_replan(decision):
    consume(decision)
    decision.grid = free_grid(decision, blocked=True)
    decision._map_updated()
    decision._map_updated()
    assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0]
    assert not decision.reference_pub.messages[-1].points


def test_stale_worker_result_does_not_replace_or_clear_current_global_path(decision):
    consume(decision)
    consume(decision, version=decision.version - 1, status=AlgorithmStatus.TIMEOUT)
    assert len(decision.path_pub.messages) == 1
    assert len(decision.reference_pub.messages) == 1
    assert decision.trajectory is not None


def test_cancel_service_clears_global_path_and_stops_reference(decision):
    consume(decision)
    cancellation = decision._cancel(Trigger.Request(), Trigger.Response())
    try:
        cancellation.send(None)
        assert [len(message.poses) for message in decision.path_pub.messages] == [3, 0]
        assert not decision.reference_pub.messages[-1].active
        assert not decision.reference_pub.messages[-1].points
        decision._resolve_cancellations(ControlReport(
            task_id=decision.task_id, reference_sequence=decision.sequence))
        with pytest.raises(StopIteration) as completed:
            cancellation.send(None)
        assert completed.value.value.success
    finally:
        cancellation.close()
