"""ROS-message-level retry tests with a deterministic fake navigation service."""

import json
from collections import deque
from types import SimpleNamespace
import time

import pytest

pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time

from usv_ladds_bridge.leg_state import LegBook
from usv_ladds_bridge.single_leg_node import SingleLegNode
from common.geodesy import GeoReference


def _leg(travel_id=53):
    point = SimpleNamespace(
        x=83.38367674866956,
        y=91.682939134985,
        speed_mps=4 * 1852 / 3600,
    )
    return SimpleNamespace(
        identity=(1000, travel_id),
        fingerprint=f"route-{travel_id}",
        travel_id=travel_id,
        x=83.38367674866956,
        y=91.682939134985,
        speed_mps=4 * 1852 / 3600,
        points=(point,),
    )


class _Now:
    @staticmethod
    def to_msg():
        return Time()


class _Clock:
    @staticmethod
    def now():
        return _Now()


class _Future:
    def __init__(self):
        self.callback = None
        self.response = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def resolve(self, **values):
        self.response = SimpleNamespace(**values)
        self.callback(self)


class _Client:
    def __init__(self):
        self.requests = []
        self.futures = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = _Future()
        self.futures.append(future)
        return future


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class _Harness:
    _receive = SingleLegNode._receive
    _submit = SingleLegNode._submit
    _accepted = SingleLegNode._accepted
    _navigation = SingleLegNode._navigation
    _tick = SingleLegNode._tick
    _fresh_navigation_session = SingleLegNode._fresh_navigation_session

    def __init__(self):
        self.book = LegBook(max_timeout_retries=3)
        self.client = _Client()
        self.cancel_client = _Client()
        self.origin = GeoReference(30.5453, 114.392931)
        self.frame = "otter/map"
        self.waypoint_radius = 1.5
        self.future = None
        self.submitted_at = 0.0
        self.submitted_status_serial = 0
        self.retry_after_status_serial = 0
        self.navigation_status_serial = 1
        self.navigation_status_at = time.monotonic()
        self.navigation_session = "planner-a"
        self.retired_navigation_sessions = deque(maxlen=8)
        self.waiting_record = None
        self.navigation_status = {}
        self.cancel_future = None
        self.cancel_required = False
        self.cancel_record = None
        self.cancel_generation = 0
        self.cancel_started_at = 0.0
        self.cancel_retry_at = 0.0
        self.cancel_timed_out = False
        self.cancel_session = ''
        self.service_timeout = 0.01
        self.notices = []
        self.completions = []

    def __getattr__(self, name):
        method = getattr(SingleLegNode, name, None)
        if callable(method):
            return method.__get__(self)
        raise AttributeError(name)

    @staticmethod
    def get_clock():
        return _Clock()

    def _notice(self, state, reason="", record=None):
        self.notices.append((state, reason, record))

    def _complete(self, record):
        self.completions.append(record)

    def _warning(self, text):
        pass


def _status(session="planner-a", request_id="", state="IDLE", task_id=0, goal=None,
            waypoint_count=0, waypoint_index=0, passed_waypoints=0,
            waypoint_radius=1.5):
    return SimpleNamespace(
        data=json.dumps(
            {
                "state": state,
                "request_id": request_id,
                "task_id": task_id,
                "current_task_id": task_id,
                "goal": goal,
                "waypoint_count": waypoint_count,
                "waypoint_index": waypoint_index,
                "passed_waypoints": passed_waypoints,
                "waypoint_radius": waypoint_radius,
                "modules": {"decision": session},
            }
        )
    )


def test_timeout_retries_three_times_with_same_request_id_then_stays_unknown():
    node = _Harness()
    _, record = node.book.receive(_leg(), planner_session="planner-a")
    node._submit(record)

    for _ in range(3):
        node.submitted_at = time.monotonic() - 1.0
        node._tick()
        assert record.state == "RETRY_PENDING"
        node._navigation(_status())
        assert record.state == "SUBMITTING"

    node.submitted_at = time.monotonic() - 1.0
    node._tick()
    node._navigation(_status())

    assert len(node.client.requests) == 4
    assert {request.request_id for request in node.client.requests} == {
        record.request_id
    }
    assert record.state == "SUBMIT_UNKNOWN"
    assert "confirm navigation is cancelled" in record.reason.lower()
    assert "restart" in record.reason.lower()
    assert node.book.receive(_leg(54))[0] == "busy"


def test_new_decision_session_aborts_pending_retry_and_late_future_is_ignored():
    node = _Harness()
    _, record = node.book.receive(_leg(), planner_session="planner-a")
    node._submit(record)
    first_future = node.client.futures[0]
    node.submitted_at = time.monotonic() - 1.0
    node._tick()

    node._navigation(_status(session="planner-b"))
    first_future.resolve(
        accepted=True,
        planner_session="planner-a",
        task_id=7,
        reason="ACCEPTED",
    )

    assert len(node.client.requests) == 1
    assert record.state == "ABORTED"
    assert node.book.active is None
    assert node.completions == []


def _message(travel_id=58, speed=4, points=None):
    if points is None:
        points = [{'type': 1, 'longitude': 114.3938,
                   'latitude': 30.546127, 'speed': speed}]
    return SimpleNamespace(data=json.dumps({
        'travelId': travel_id, 'taskType': 16, 'count': len(points),
        'points': points,
    }))


def _accept(node, index=-1, task_id=7):
    node.client.futures[index].resolve(
        accepted=True, planner_session='planner-a', task_id=task_id, reason='ACCEPTED')


def _multi_points():
    return [
        {'type': 1, 'longitude': 114.3938,
         'latitude': 30.546127, 'speed': 2},
        {'type': 1, 'longitude': 114.3938,
         'latitude': 30.546227, 'speed': 4},
        {'type': 1, 'longitude': 114.3938,
         'latitude': 30.546327, 'speed': 2},
    ]


def test_one_packet_submits_one_route_request_with_all_ordered_segment_speeds():
    node = _Harness()

    node._receive(_message(points=_multi_points()))

    assert len(node.client.requests) == 1
    request = node.client.requests[0]
    assert request.header.frame_id == 'otter/map'
    assert request.waypoint_radius == 1.5
    assert [(point.pose.x, point.pose.y) for point in request.waypoints] == pytest.approx([
        (83.38367674866956, 91.682939134985),
        (83.38359129340336, 102.76910894104553),
        (83.38350583759512, 113.85527891640633),
    ], abs=1e-9)
    assert [point.pose.theta for point in request.waypoints] == [0.0, 0.0, 0.0]
    assert [point.desired_speed_mps for point in request.waypoints] == pytest.approx([
        1.028888888888889, 2.057777777777778, 1.028888888888889,
    ])


def test_navigation_progress_does_not_complete_route_before_goal_reached():
    node = _Harness()
    node._receive(_message(points=_multi_points()))
    record = node.book.active
    _accept(node)

    node._navigation(_status(
        request_id=record.request_id,
        state='NAVIGATING',
        task_id=7,
        goal=[83.38350583759512, 113.85527891640633, 0.0],
        waypoint_count=3,
        waypoint_index=3,
        passed_waypoints=3,
    ))

    assert node.book.active is record
    assert record.state == 'ACTIVE'
    assert node.completions == []


def test_status_notice_uses_correlated_navigation_progress_and_current_segment():
    node = _Harness()
    node._receive(_message(points=_multi_points()))
    record = node.book.active
    _accept(node)
    node.navigation_status = json.loads(_status(
        request_id=record.request_id,
        state='NAVIGATING',
        task_id=7,
        waypoint_count=3,
        waypoint_index=1,
        passed_waypoints=1,
        waypoint_radius=1.5,
    ).data)
    node.status_pub = _Publisher()

    SingleLegNode._notice(node, 'ACTIVE', '', record)

    notice = node.status_pub.messages[-1]
    assert notice['waypoint_count'] == 3
    assert notice['waypoint_index'] == 1
    assert notice['current_waypoint'] == 2
    assert notice['passed_waypoints'] == 1
    assert notice['waypoint_radius'] == 1.5
    assert notice['target_x'] == pytest.approx(83.38359129340336)
    assert notice['target_y'] == pytest.approx(102.76910894104553)
    assert notice['desired_speed_mps'] == pytest.approx(2.057777777777778)


def test_receiving_same_route_after_completion_starts_new_execution():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._navigation(_status(request_id=first.request_id, state='GOAL_REACHED', task_id=7))
    node._receive(_message())

    assert len(node.client.requests) == 2
    assert node.client.requests[1].request_id != first.request_id
    assert node.client.requests[1].waypoints[0].desired_speed_mps == pytest.approx(
        2.0577777778)
    assert node.completions == [first]


def test_active_route_is_cancelled_before_same_route_restarts():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._receive(_message(speed=2))

    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')

    assert first.state == 'ABORTED'
    assert len(node.client.requests) == 2
    assert node.client.requests[1].request_id != first.request_id
    assert node.client.requests[1].waypoints[0].desired_speed_mps == pytest.approx(
        1.0288888889)


def test_cancelled_status_does_not_bypass_zero_output_acknowledgement():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._receive(_message())
    node._navigation(_status(request_id=first.request_id, state='CANCELLED', task_id=7))
    assert len(node.client.requests) == 1
    node.cancel_client.futures[0].resolve(success=False, message='zero-output timeout')
    node._tick()
    assert len(node.client.requests) == 1
    node.cancel_retry_at = 0.0
    node._tick()
    node.cancel_client.futures[-1].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2


def test_new_route_during_unconfirmed_submission_waits_for_acceptance_before_cancel():
    node = _Harness()
    node._receive(_message())
    node._receive(_message(speed=2))
    assert node.cancel_client.requests == []
    assert len(node.client.requests) == 1
    _accept(node)
    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2


def test_latest_waiting_route_replaces_previous_and_superseded_completion_is_suppressed():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._receive(_message(speed=2))
    waiting = node.waiting_record
    node._receive(_message(speed=3))
    node._navigation(_status(request_id=first.request_id, state='GOAL_REACHED', task_id=7))
    assert node.completions == []
    assert waiting.state == 'ABORTED'
    assert len(node.cancel_client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2
    assert node.client.requests[-1].waypoints[0].desired_speed_mps == pytest.approx(
        1.5433333333)


def test_single_received_route_waiting_for_navigation_starts_when_ready_without_resend():
    node = _Harness()
    node.client.ready = False
    node._receive(_message())
    assert node.client.requests == []
    node.client.ready = True
    node._navigation(_status())
    assert len(node.client.requests) == 1


def test_gui_navigation_is_cancelled_before_received_route_starts():
    node = _Harness()
    node._navigation(_status(state='ACTIVE', goal=[3, 4, 0], task_id=8))
    node._receive(_message())
    assert len(node.cancel_client.requests) == 1
    assert node.client.requests == []
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 1


def test_invalid_packet_does_not_interrupt_existing_navigation():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._receive(SimpleNamespace(data='{"travelId":58,"count":3}'))
    assert node.book.active is first
    assert node.cancel_client.requests == []
    assert node.notices[-1][0] == 'INVALID_TASK'


def test_cancel_timeout_keeps_same_future_and_waits_for_late_success():
    node = _Harness()
    node._receive(_message())
    _accept(node)
    node._receive(_message())
    future = node.cancel_client.futures[0]
    node.cancel_started_at = time.monotonic() - 1.0
    node._tick()
    node._tick()
    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 1
    assert any(state == 'CANCEL_UNKNOWN' for state, _, _ in node.notices)
    future.resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2


def test_old_cancel_and_accept_callbacks_do_not_affect_new_session_execution():
    node = _Harness()
    node._receive(_message())
    old_future = node.client.futures[0]
    _accept(node)
    node._receive(_message())
    cancel_future = node.cancel_client.futures[0]
    node._navigation(_status(session='planner-b'))
    current = node.book.active
    notice_count = len(node.notices)
    cancel_future.resolve(success=True, message='old cancellation')
    old_future.resolve(accepted=True, planner_session='planner-a', task_id=7, reason='ACCEPTED')
    assert node.book.active is current
    assert current.planner_session == 'planner-b'
    assert len(node.client.requests) == 2
    assert len(node.notices) == notice_count


def test_unknown_old_submission_cannot_be_bypassed_until_matching_status_arrives():
    node = _Harness()
    node._receive(_message())
    old = node.book.active
    node._receive(_message(speed=2))
    for _ in range(4):
        node.submitted_at = time.monotonic() - 1.0
        node._tick()
        node._navigation(_status())
    assert old.state == 'SUBMIT_UNKNOWN'
    assert len(node.client.requests) == 4
    assert node.cancel_client.requests == []
    node._navigation(_status(request_id=old.request_id, state='ACTIVE', task_id=7, goal=[83, 92, 0]))
    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 4
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 5
    assert node.client.requests[-1].request_id != old.request_id


def test_busy_due_to_gui_race_cancels_then_retries_same_received_execution():
    node = _Harness()
    node._receive(_message())
    request_id = node.client.requests[0].request_id
    node.client.futures[0].resolve(
        accepted=False, planner_session='planner-a', task_id=0, reason='BUSY: GUI goal active')
    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2
    assert node.client.requests[-1].request_id == request_id


def test_newer_packet_is_preserved_when_older_submission_returns_busy():
    node = _Harness()
    node._receive(_message(speed=4))
    first_id = node.client.requests[0].request_id
    node._receive(_message(speed=2))
    node.client.futures[0].resolve(
        accepted=False, planner_session='planner-a', task_id=0, reason='BUSY: GUI goal active')
    assert len(node.cancel_client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert len(node.client.requests) == 2
    assert node.client.requests[-1].request_id != first_id
    assert node.client.requests[-1].waypoints[0].desired_speed_mps == pytest.approx(
        1.0288888889)


def test_new_session_service_acceptance_before_status_preserves_completion_association():
    node = _Harness()
    node._receive(_message())
    record = node.book.active
    node.client.futures[0].resolve(
        accepted=True, planner_session='planner-b', task_id=2, reason='ACCEPTED')

    assert node.book.active is record
    assert record.planner_session == 'planner-b'
    assert node.navigation_session == 'planner-b'
    assert node._fresh_navigation_session() == ''
    node._navigation(_status(session='planner-a'))
    assert node.book.active is record
    assert node.navigation_session == 'planner-b'
    node._navigation(_status(session='planner-b', request_id=record.request_id,
                             state='GOAL_REACHED', task_id=2))
    assert node.completions == [record]


def test_response_from_retired_session_does_not_rebind_current_request():
    node = _Harness()
    node._navigation(_status(session='planner-b'))
    node._receive(_message())
    record = node.book.active
    node.client.futures[0].resolve(
        accepted=True, planner_session='planner-a', task_id=9, reason='ACCEPTED')
    assert node.book.active is record
    assert record.planner_session == 'planner-b'
    assert record.task_id == -1
    assert record.state == 'SUBMITTING'
    node._navigation(_status(session='planner-a', request_id=record.request_id,
                             state='GOAL_REACHED', task_id=9))
    assert node.completions == []
    assert node.navigation_session == 'planner-b'
