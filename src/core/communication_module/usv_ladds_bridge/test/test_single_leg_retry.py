"""ROS-message-level retry tests with a deterministic fake navigation service."""

import json
from collections import deque
from types import SimpleNamespace
import time

import pytest

pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time

from usv_ladds_bridge.leg_state import LegBook
from usv_ladds_bridge.stop_task import StopExecution
from usv_ladds_bridge.route_node import RouteNode
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
    _receive = RouteNode._receive
    _submit = RouteNode._submit
    _accepted = RouteNode._accepted
    _navigation = RouteNode._navigation
    _tick = RouteNode._tick
    _fresh_navigation_session = RouteNode._fresh_navigation_session

    def __init__(self):
        self.book = LegBook(max_timeout_retries=3)
        self.client = _Client()
        self.cancel_client = _Client()
        self.stop_client = _Client()
        self.stop_record = None
        self.stop_futures = set()
        self.stop_submitted_at = 0.0
        self.stop_retry_at = 0.0
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
        self.tx = _Publisher()
        self.progress_tx = _Publisher()

    def __getattr__(self, name):
        method = getattr(RouteNode, name, None)
        if callable(method):
            return method.__get__(self)
        raise AttributeError(name)

    @staticmethod
    def get_clock():
        return _Clock()

    def _notice(self, state, reason="", record=None):
        self.notices.append((state, reason, record))

    def _report_arrival(self, record):
        self.completions.append(record)
        RouteNode._report_arrival(self, record)

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


def _message(travel_id=58, speed=4, points=None, task_type=16):
    if points is None:
        points = [{'type': 1, 'longitude': 114.3938,
                   'latitude': 30.546127, 'speed': speed}]
    return SimpleNamespace(data=json.dumps({
        'travelId': travel_id, 'taskType': task_type, 'count': len(points),
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

    RouteNode._notice(node, 'ACTIVE', '', record)

    notice = node.status_pub.messages[-1]
    assert notice['waypoint_count'] == 3
    assert notice['waypoint_index'] == 1
    assert notice['current_waypoint'] == 2
    assert notice['passed_waypoints'] == 1
    assert notice['waypoint_radius'] == 1.5
    assert notice['target_x'] == pytest.approx(83.38359129340336)
    assert notice['target_y'] == pytest.approx(102.76910894104553)
    assert notice['desired_speed_mps'] == pytest.approx(2.057777777777778)


def _progress_status(record, index, state='NAVIGATING', count=3):
    return _status(request_id=record.request_id, state=state, task_id=7,
                   waypoint_count=count, waypoint_index=index, passed_waypoints=index)


def test_0053_reports_each_confirmed_waypoint_once_then_final_before_0052():
    node = _Harness()
    node._receive(_message(travel_id=61, points=_multi_points()))
    record = node.book.active
    _accept(node)

    for index in (0, 0, 1, 1, 2):
        node._navigation(_progress_status(record, index))
    assert node.progress_tx.messages == [
        {'travelId': 61, 'taskIndex': 0},
        {'travelId': 61, 'taskIndex': 1},
        {'travelId': 61, 'taskIndex': 2},
    ]
    assert node.tx.messages == []

    node._navigation(_progress_status(record, 3, state='GOAL_REACHED'))
    assert node.progress_tx.messages[-1] == {'travelId': 61, 'taskIndex': 3}
    assert node.tx.messages == [{'travelId': 61, 'taskFinish': 1}]


def test_0053_ignores_status_of_another_request_or_waypoint_count():
    node = _Harness()
    node._receive(_message(points=_multi_points()))
    record = node.book.active
    _accept(node)

    node._navigation(_status(request_id='gui-goal', state='NAVIGATING', task_id=6,
                             waypoint_count=3, waypoint_index=1))
    node._navigation(_progress_status(record, 1, count=2))
    assert node.progress_tx.messages == []


def test_0053_is_not_sent_for_aborted_route():
    node = _Harness()
    node._receive(_message(points=_multi_points()))
    record = node.book.active
    _accept(node)
    node._navigation(_progress_status(record, 1))
    node._navigation(_progress_status(record, 2, state='CANCELLED'))
    assert record.state == 'ABORTED'
    assert node.progress_tx.messages == [{'travelId': 58, 'taskIndex': 1}]


def test_0053_guard_route_reports_final_index_once_with_guard_0052():
    node = _Harness()
    content = json.loads(_message(travel_id=62, points=_multi_points()).data)
    content.update(taskType=80)
    node._receive(SimpleNamespace(data=json.dumps(content)))
    record = node.book.active
    _accept(node)
    node._navigation(_progress_status(record, 2))
    for _ in range(2):
        node._navigation(_progress_status(record, 3, state='GUARDING'))
    assert node.progress_tx.messages == [
        {'travelId': 62, 'taskIndex': 2},
        {'travelId': 62, 'taskIndex': 3},
    ]
    assert node.tx.messages == [{'travelId': 62, 'taskType': 80, 'taskFinish': 2}]


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


def test_guard_arrival_sends_0052_once_without_ending_micro_control():
    node = _Harness()
    content = json.loads(_message(points=_multi_points()).data)
    content.update(taskType=80, guardingHeading=0)
    node._receive(SimpleNamespace(data=json.dumps(content)))
    request = node.client.requests[-1]
    assert request.guard_on_arrival is True
    assert request.guard_heading_enabled is True
    assert request.guard_heading == pytest.approx(1.5707963267948966)
    record = node.book.active
    _accept(node)
    node._navigation(_status(request_id=record.request_id, state='GUARDING', task_id=7))
    assert record.state == 'GUARDING'
    assert node.book.active is record
    assert node.completions == [record]
    assert node.tx.messages == [{
        'travelId': record.leg.travel_id, 'taskType': 80, 'taskFinish': 2,
    }]
    assert node.notices[-1][0] == 'GUARDING'
    for state in ('GUARDING', 'GUARD_BLOCKED', 'GUARDING'):
        node._navigation(_status(request_id=record.request_id, state=state, task_id=7))
    assert node.completions == [record]
    assert len(node.tx.messages) == 1
    assert node.book.active is record
    # A new route must still cancel the ongoing guard, despite the arrival ACK.
    node._receive(_message(travel_id=59))
    assert len(node.cancel_client.requests) == 1
    assert len(node.client.requests) == 1


def test_guard_arrival_during_replacement_is_not_reported_as_new_route_arrival():
    node = _Harness()
    content = json.loads(_message().data)
    content.update(taskType=80, guardingHeading=None)
    node._receive(SimpleNamespace(data=json.dumps(content)))
    record = node.book.active
    _accept(node)
    node._receive(_message(travel_id=59))
    node._navigation(_status(request_id=record.request_id, state='GUARDING', task_id=7))
    assert node.tx.messages == []
    assert record.state == 'ABORTED'
    assert len(node.client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='CANCELLED')
    assert len(node.client.requests) == 2


def test_stop_does_not_wait_for_pending_route_acceptance_or_fresh_status():
    node = _Harness()
    node._receive(_message())
    record = node.book.active
    old_future = node.client.futures[0]
    node._receive(_message(speed=2))
    waiting = node.waiting_record
    node.navigation_status_at = float('-inf')
    node._receive_stop(SimpleNamespace(data='{}'))
    assert len(node.stop_client.requests) == 1
    assert record.request_id in node.stop_client.requests[0].superseded_request_ids
    assert record.state == waiting.state == 'ABORTED'
    assert node.waiting_record is None
    assert node.book.active is None
    old_future.resolve(accepted=True, planner_session='planner-a', task_id=7, reason='ACCEPTED')
    assert node.book.active is None
    assert node.completions == []


def test_stop_accepted_does_not_mean_stopped_and_new_route_waits_for_matching_status():
    node = _Harness()
    node._receive_stop(SimpleNamespace(data='{}'))
    stop = node.stop_record
    node._receive(_message())
    node.stop_client.futures[0].resolve(accepted=True, planner_session='planner-a', task_id=8, reason='STOPPING')
    assert node.client.requests == []
    node._navigation(_status(request_id='old-stop', state='STOPPED', task_id=7))
    assert node.client.requests == []
    node._navigation(_status(request_id=stop.request_id, state='STOPPED', task_id=8))
    assert len(node.client.requests) == 1
    assert node.completions == []


def test_stop_timeout_retries_same_id_but_old_unresolved_stop_still_blocks_new_route():
    node = _Harness()
    node._receive_stop(SimpleNamespace(data='{}'))
    stop = node.stop_record
    node.stop_submitted_at = time.monotonic() - 1
    node._tick()
    assert len(node.stop_client.requests) == 2
    assert {r.request_id for r in node.stop_client.requests} == {stop.request_id}
    node.stop_client.futures[-1].resolve(accepted=True, planner_session='planner-a', task_id=8, reason='STOPPING')
    node._navigation(_status(request_id=stop.request_id, state='STOPPED', task_id=8))
    node._receive(_message())
    assert node.client.requests == []
    node.stop_client.futures[0].resolve(accepted=True, planner_session='planner-a', task_id=8, reason='STOPPING')
    assert len(node.client.requests) == 1


def test_stop_during_cancel_waits_for_old_cancel_result_before_new_route():
    node = _Harness()
    node._receive(_message())
    _accept(node)
    node._receive(_message(speed=2))
    cancel_future = node.cancel_client.futures[0]
    node._receive_stop(SimpleNamespace(data='{}'))
    stop = node.stop_record
    node.stop_client.futures[0].resolve(accepted=True, planner_session='planner-a', task_id=8, reason='STOPPING')
    node._navigation(_status(request_id=stop.request_id, state='STOPPED', task_id=8))
    node._receive(_message(speed=3))
    assert len(node.client.requests) == 1
    cancel_future.resolve(success=False, message='Active braking cannot be cancelled')
    assert len(node.client.requests) == 2


def test_malformed_stop_does_not_abort_current_task():
    node = _Harness()
    node._receive(_message())
    record = node.book.active
    node._receive_stop(SimpleNamespace(data='[]'))
    assert node.book.active is record
    assert node.stop_client.requests == []


def test_berthing_arrival_sends_0052_with_task_finish_4():
    node = _Harness()
    node._receive_berth(_message(task_type=4))
    record = node.book.active
    assert len(record.leg.points) == 1
    assert record.leg.protocol == '0212'
    assert record.leg.task_type == 4
    _accept(node)
    node._navigation(_status(request_id=record.request_id, state='GOAL_REACHED', task_id=7))
    assert node.completions == [record]
    assert node.tx.messages == [{
        'travelId': record.leg.travel_id, 'taskType': 4, 'taskFinish': 4,
    }]


def test_berth_progress_does_not_send_0053():
    # 0053 属于 0211 航路；0212 靠离泊只回 0052。
    node = _Harness()
    node._receive_berth(_message(task_type=4))
    record = node.book.active
    _accept(node)
    node._navigation(_progress_status(record, 0, count=1))
    node._navigation(_progress_status(record, 1, state='GOAL_REACHED', count=1))
    assert node.completions == [record]
    assert node.progress_tx.messages == []


def test_route_arrival_still_sends_task_finish_1():
    node = _Harness()
    node._receive(_message())
    record = node.book.active
    _accept(node)
    node._navigation(_status(request_id=record.request_id, state='GOAL_REACHED', task_id=7))
    assert node.tx.messages == [{'travelId': record.leg.travel_id, 'taskFinish': 1}]


def test_berth_replaces_active_route_before_submitting():
    node = _Harness()
    node._receive(_message())
    first = node.book.active
    _accept(node)
    node._receive_berth(_message(travel_id=59, speed=2, task_type=3))
    assert first.state == 'ACTIVE'
    assert len(node.cancel_client.requests) == 1
    node.cancel_client.futures[0].resolve(success=True, message='zero thrust active')
    assert first.state == 'ABORTED'
    assert '0212' in first.reason
    assert len(node.client.requests) == 2
    assert node.book.active.request_id != first.request_id
    assert node.book.active.leg.protocol == '0212'
    assert node.client.requests[-1].waypoints[0].desired_speed_mps == pytest.approx(
        1.0288888889)
