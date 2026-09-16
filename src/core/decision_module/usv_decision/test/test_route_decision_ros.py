"""Whole-route service and actual passage must preserve one controller task."""
import pytest

rclpy = pytest.importorskip('rclpy')
from usv_decision.decision import DecisionNode
from usv_navigation_interfaces.msg import RouteWaypoint, ControlReport
from usv_navigation_interfaces.srv import NavigateRoute
from common.models import VesselState
from trajectory_planner.trajectory import Trajectory


@pytest.fixture
def decision():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/route_decision_test'])
    node = DecisionNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def request(node, points=((5., 0., .4), (10., 0., .7)), request_id='route-one'):
    result = NavigateRoute.Request(request_id=request_id, waypoint_radius=1.5)
    result.header.frame_id = node.frame
    for x, y, speed in points:
        point = RouteWaypoint(desired_speed_mps=speed)
        point.pose.x, point.pose.y = x, y
        result.waypoints.append(point)
    return result


def send(node, value):
    return node._navigate_route(value, NavigateRoute.Response())


def test_route_validation_is_atomic_and_internal_retry_is_idempotent(decision):
    invalid = request(decision, points=((5., 0., .4), (10., 0., float('nan'))))
    before = decision.task_id
    assert not send(decision, invalid).accepted
    assert decision.goal is None and decision.task_id == before
    valid = request(decision)
    accepted = send(decision, valid)
    assert accepted.accepted
    assert send(decision, valid).reason == 'IDEMPOTENT'
    assert decision.task_id == accepted.task_id
    valid.waypoints[1].pose.x = 11.
    assert not send(decision, valid).accepted
    assert decision.route_progress.points[1][0] == 10.


def test_actual_passage_changes_revision_and_speed_but_keeps_controller_task(decision):
    send(decision, request(decision))
    task, revision = decision.task_id, decision.version
    trajectory = Trajectory([[0., 0., 4., 0., 0., 0., .4, 0.]],
                            decision.now_seconds(), decision.frame, None)
    decision.trajectory = trajectory
    decision.state = VesselState(x=4., stamp=decision.now_seconds(), frame_id=decision.frame)
    decision._update_route_progress()
    assert decision.task_id == task and decision.version == revision + 1
    assert decision.goal == (10., 0., 0.) and decision.desired_speed_mps == .7
    assert decision.trajectory is trajectory
    assert decision.route_progress.terminal


def test_closed_route_cannot_finish_early_or_from_previous_phase_report(decision):
    send(decision, request(decision, points=((5., 0., .4), (0., 0., .7))))
    decision.peers['control'].observe('control', 1, 0.)
    report = ControlReport(control_session='control', planner_session=decision.session,
        task_id=decision.task_id, task_finished=True, reference_sequence=decision.sequence)
    decision._on_control_report(report)
    assert decision.goal == (5., 0., 0.)
    decision.state = VesselState(x=4., stamp=decision.now_seconds(), frame_id=decision.frame)
    decision._update_route_progress()
    decision._on_control_report(report)
    assert decision.goal == (0., 0., 0.)
    # Only a terminal trajectory enables completion, and its report must be current.
    decision.trajectory = Trajectory([[0., 0., 1., 0., 0., 0., .1, 0.]],
                                    decision.now_seconds(), decision.frame, (0., 0.))
    decision._publish_reference()
    report.reference_sequence = decision.sequence
    decision._on_control_report(report)
    assert decision.mode == 'GOAL_REACHED' and decision.goal is None
    assert decision.route_summary['passed_waypoints'] == 2


def test_stale_odometry_does_not_advance(decision):
    send(decision, request(decision))
    decision.state = VesselState(x=5., stamp=decision.now_seconds() - 20., frame_id=decision.frame)
    decision._update_route_progress()
    assert decision.route_progress.index == 0


def test_invalid_radius_empty_route_and_busy_request_preserve_active_route(decision):
    active = request(decision)
    accepted = send(decision, active)
    for invalid in (request(decision, points=(), request_id='empty'),
                    request(decision, request_id='busy')):
        assert not send(decision, invalid).accepted
        assert decision.task_id == accepted.task_id
    active.waypoint_radius = 0.
    assert not send(decision, active).accepted
    assert decision.route_progress.radius == 1.5
