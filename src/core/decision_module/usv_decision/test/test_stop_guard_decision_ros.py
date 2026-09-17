"""Stop preemption and persistent terminal guarding use the same control owner."""
import math
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from usv_decision.decision import DecisionNode
from usv_navigation_interfaces.msg import ControlReport, RouteWaypoint
from usv_navigation_interfaces.srv import NavigateRoute, StopNavigation
from trajectory_planner.trajectory import Trajectory


@pytest.fixture
def decision():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/stop_guard_decision'])
    node = DecisionNode()
    node.peers['control'].observe('control', 1, 0.)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def route(node, request_id='route', guard=True, heading=True):
    request = NavigateRoute.Request(request_id=request_id, waypoint_radius=1.5,
        guard_on_arrival=guard, guard_heading_enabled=heading, guard_heading=math.pi / 2.)
    request.header.frame_id = node.frame
    point = RouteWaypoint(desired_speed_mps=.4)
    point.pose.x = 5.
    request.waypoints = [point]
    return request


def send_route(node, request):
    return node._navigate_route(request, NavigateRoute.Response())


def stop(node, request_id='stop', superseded=('route',)):
    return node._stop(StopNavigation.Request(request_id=request_id,
        superseded_request_ids=list(superseded)), StopNavigation.Response())


def report(node, **kwargs):
    return ControlReport(control_session='control', planner_session=node.session,
        task_id=node.task_id, reference_sequence=node.sequence, **kwargs)


def test_guard_configuration_is_part_of_idempotency_and_validated_atomically(decision):
    value = route(decision)
    accepted = send_route(decision, value)
    assert accepted.accepted
    assert decision.guard_on_arrival and decision.guard_heading_enabled
    assert decision.guard_heading == pytest.approx(math.pi / 2.)
    assert send_route(decision, value).reason == 'IDEMPOTENT'
    value.guard_heading_enabled = False
    assert not send_route(decision, value).accepted
    assert decision.task_id == accepted.task_id


def test_invalid_guard_heading_does_not_replace_any_task(decision):
    value = route(decision)
    value.guard_heading = float('nan')
    assert not send_route(decision, value).accepted
    assert decision.goal is None
    value.guard_heading = 0.
    value.guard_on_arrival = False
    assert not send_route(decision, value).accepted


@pytest.mark.parametrize('guard_state', ['GUARDING', 'GUARD_BRAKING', 'GUARD_BLOCKED'])
def test_terminal_guarding_keeps_task_and_goal_but_retires_planning(decision, guard_state):
    send_route(decision, route(decision))
    original_task = decision.task_id
    decision.trajectory = Trajectory([[0., 0., 5., 0., 0., 0., 0., 0.]],
        decision.now_seconds(), decision.frame, (5., 0.))
    decision._publish_reference()
    decision._on_control_report(report(decision, state=guard_state))
    assert decision.motion_mode == 'HOLD'
    assert decision.mode == 'GUARDING'
    assert decision.task_id == original_task
    assert decision.goal == (5., 0., 0.)
    assert decision.route_summary['passed_waypoints'] == 1
    assert decision.trajectory is None
    assert decision.route_progress is None
    decision._on_control_report(report(decision, task_finished=True, state='GOAL_REACHED'))
    assert decision.goal is not None and decision.motion_mode == 'HOLD'


def test_intermediate_or_stale_report_cannot_enter_guarding(decision):
    value = route(decision)
    last = RouteWaypoint(desired_speed_mps=.4)
    last.pose.x = 10.
    value.waypoints.append(last)
    send_route(decision, value)
    decision._on_control_report(report(decision, state='GUARDING'))
    assert decision.motion_mode == 'NAVIGATE'
    assert decision.route_progress.index == 0


def test_stop_preempts_route_and_blocks_delayed_requests_without_a_position(decision):
    pending = route(decision)
    accepted = stop(decision)
    assert accepted.accepted
    assert decision.motion_mode == 'BRAKE' and decision.goal is None
    assert not send_route(decision, pending).accepted
    assert not send_route(decision, route(decision, 'new')).accepted
    snapshot = (decision.task_id, decision.version)
    assert stop(decision).reason == 'IDEMPOTENT'
    assert (decision.task_id, decision.version) == snapshot
    goal = PoseStamped()
    goal.header.frame_id = decision.frame
    goal.pose.orientation.w = 1.
    decision._on_goal(goal)
    assert (decision.task_id, decision.version) == snapshot
    # The coroutine has no suspension on this rejection path.
    pending_cancel = decision._cancel(Trigger.Request(), Trigger.Response())
    with pytest.raises(StopIteration) as returned:
        pending_cancel.send(None)
    assert not returned.value.value.success
    assert decision.motion_mode == 'BRAKE'


def test_only_matching_stopped_report_finishes_braking_and_never_resumes_route(decision):
    send_route(decision, route(decision))
    accepted = stop(decision)
    decision._on_control_report(report(decision, task_finished=True, state='GOAL_REACHED'))
    assert decision.motion_mode == 'BRAKE'
    decision._on_control_report(report(decision, task_finished=True, state='STOPPED', left=-.1))
    assert decision.motion_mode == 'BRAKE'
    decision._on_control_report(report(decision, task_finished=True, state='STOPPED'))
    assert decision.mode == 'STOPPED' and decision.goal is None
    assert decision.accepted_task_id == accepted.task_id
    assert decision.request_id == 'stop'
    assert not send_route(decision, route(decision)).accepted
    assert send_route(decision, route(decision, 'after-stop')).accepted
    # A delayed same-stop retry must not preempt this new execution.
    task = decision.task_id
    assert stop(decision).reason == 'IDEMPOTENT'
    assert decision.task_id == task and decision.goal is not None


def test_mode_reference_has_a_fresh_sequence_lease_but_navigation_does_not(decision):
    received = []
    decision.reference_pub = SimpleNamespace(publish=received.append)
    stop(decision)
    decision._publish_reference(changed=False)
    decision._publish_reference(changed=False)
    assert received[-1].active and received[-1].motion_mode == 'BRAKE'
    assert received[-1].sequence > received[-2].sequence
    assert received[-1].header.frame_id == decision.frame


def test_stop_ids_cannot_be_reused_for_navigation(decision):
    stop(decision)
    decision._on_control_report(report(decision, task_finished=True, state='STOPPED'))
    assert not send_route(decision, route(decision, 'stop')).accepted


def test_peer_restarts_preserve_stop_intent_but_reject_old_control_reports(decision):
    accepted = stop(decision)
    old_report = report(decision, task_finished=True, state='STOPPED')
    for role in ('control', 'perception'):
        before = decision.task_id
        decision._on_peer_restart(role)
        assert decision.task_id > before
        assert decision.motion_mode == 'BRAKE' and decision.mode == 'STOPPING'
        assert decision.accepted_task_id == accepted.task_id
        assert decision.request_id == 'stop'
        assert stop(decision).task_id == accepted.task_id
        decision._on_control_report(old_report)
        assert decision.motion_mode == 'BRAKE'
    decision._on_control_report(report(decision, task_finished=True, state='STOPPED'))
    assert decision.mode == 'STOPPED'
    assert decision.accepted_task_id == accepted.task_id
