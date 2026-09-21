"""Real ROS message/service integration with controlled navigation test fixtures.

No Gazebo, LaDDS, or vessel controller is started. GOAL_REACHED is supplied by
the fixture to test correlation and feedback, not evidence of physical arrival.
Each route publication happens once. ROS domain 85 plus unique topic/service
names keeps these tests separate from the user's navigation system. Run with
the Humble and multi-waypoint overlays sourced, using /usr/bin/python3.
"""

import json
import threading
import time
from uuid import uuid4

import pytest

pytest.importorskip("rclpy")

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from usv_navigation_interfaces.srv import NavigateRoute

from usv_ladds_bridge.route_node import RouteNode


def _route(speeds=(2, 4, 2)):
    latitudes = (30.546127, 30.546227, 30.546327)
    return json.dumps({
        "head": {"sendPlatform": 1000, "packageSeq": 3,
                 "time": "2026-09-02 13:44:49.573"},
        "content": {
            "travelId": 58, "taskType": 16, "count": 3,
            "points": [
                {"type": 1, "latitude": latitude,
                 "longitude": 114.3938, "speed": speed}
                for latitude, speed in zip(latitudes, speeds)
            ],
            "areasCount": 0, "areas": [],
        },
    })


class NavigationFixture(Node):
    """Test services retain cancel until the test explicitly releases its reply."""

    def __init__(self, namespace, rx_topic, tx_topic, status_topic):
        super().__init__("navigation_fixture", use_global_arguments=False)
        self.lock = threading.Lock()
        self.requests = []
        self.feedback = []
        self.notices = []
        self.cancel_count = 0
        self.cancel_started = threading.Event()
        self.cancel_release = threading.Event()
        self.cancel_success = True
        self.task_counter = 0
        self.session = "fixture-" + uuid4().hex
        self.status = self.make_status("IDLE", "", 0)
        self.route_pub = self.create_publisher(String, rx_topic, 10)
        self.nav_pub = self.create_publisher(
            String, namespace + "/navigation_status", 10)
        self.create_subscription(String, tx_topic, self._feedback, 10)
        self.create_subscription(String, status_topic, self._notice, 10)
        group = ReentrantCallbackGroup()
        self.create_service(
            NavigateRoute, namespace + "/navigation/navigate_route", self._navigate,
            callback_group=group)
        self.create_service(
            Trigger, namespace + "/cancel_navigation", self._cancel,
            callback_group=group)
        self.create_timer(0.05, self._heartbeat, callback_group=group)

    def make_status(self, state, request_id, task_id, current_task_id=None,
                    waypoint_count=0, waypoint_index=0, passed_waypoints=0,
                    waypoint_radius=1.5):
        return {
            "state": state, "request_id": request_id, "task_id": task_id,
            "current_task_id": task_id if current_task_id is None else current_task_id,
            "waypoint_count": waypoint_count,
            "waypoint_index": waypoint_index,
            "passed_waypoints": passed_waypoints,
            "waypoint_radius": waypoint_radius,
            "modules": {"decision": self.session},
        }

    def _feedback(self, message):
        with self.lock:
            self.feedback.append(json.loads(message.data))

    def _notice(self, message):
        with self.lock:
            self.notices.append(json.loads(message.data))

    def _heartbeat(self):
        with self.lock:
            data = json.dumps(self.status)
        self.nav_pub.publish(String(data=data))

    def _navigate(self, request, response):
        with self.lock:
            self.task_counter += 1
            task_id = self.task_counter
            self.requests.append((request, task_id))
            self.status = self.make_status(
                "ACTIVE", request.request_id, task_id,
                waypoint_count=len(request.waypoints),
                waypoint_radius=request.waypoint_radius,
            )
        response.accepted = True
        response.reason = "TEST_FIXTURE_ACCEPTED"
        response.planner_session = self.session
        response.task_id = task_id
        return response

    def _cancel(self, request, response):
        with self.lock:
            self.cancel_count += 1
        self.cancel_started.set()
        released = self.cancel_release.wait(timeout=8)
        response.success = released and self.cancel_success
        response.message = "TEST_FIXTURE_STOP_CONFIRMED" if response.success else "TEST_FIXTURE_STOP_FAILED"
        if response.success:
            with self.lock:
                self.task_counter += 1
                self.status = self.make_status(
                    "CANCELLED", self.status["request_id"], self.status["task_id"],
                    self.task_counter,
                    waypoint_count=self.status["waypoint_count"],
                    waypoint_index=self.status["waypoint_index"],
                    passed_waypoints=self.status["passed_waypoints"],
                    waypoint_radius=self.status["waypoint_radius"],
                )
        return response

    def set_status(self, state, request_id, task_id, waypoint_index=None):
        with self.lock:
            count = self.status["waypoint_count"]
            index = count if state == "GOAL_REACHED" else self.status["waypoint_index"]
            if waypoint_index is not None:
                index = waypoint_index
            self.status = self.make_status(
                state, request_id, task_id,
                waypoint_count=count,
                waypoint_index=index,
                passed_waypoints=index,
                waypoint_radius=self.status["waypoint_radius"],
            )

    def has_notice(self, state, request_id, waypoint_index=None):
        with self.lock:
            return any(
                n.get("state") == state
                and n.get("request_id") == request_id
                and (waypoint_index is None
                     or n.get("waypoint_index") == waypoint_index)
                for n in self.notices
            )


@pytest.fixture
def ros_pair(monkeypatch):
    assert not rclpy.ok(), "run these integration checks in their own pytest process"
    monkeypatch.setenv("ROS_DOMAIN_ID", "85")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    prefix = "/ladds_execution_test_" + uuid4().hex
    namespace = prefix + "/vessel"
    rx_topic, tx_topic, status_topic = (
        prefix + "/rx", prefix + "/tx", prefix + "/status")
    rclpy.init(args=[
        "--ros-args", "-p", "navigation_namespace:=" + namespace,
        "-p", "receive_topic:=" + rx_topic,
        "-p", "transmit_topic:=" + tx_topic,
        "-p", "status_topic:=" + status_topic,
        "-p", "use_sim_time:=false", "-p", "service_timeout:=4.0",
    ])
    adapter = fixture = executor = thread = None
    errors = []
    try:
        adapter = RouteNode()
        fixture = NavigationFixture(namespace, rx_topic, tx_topic, status_topic)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(adapter)
        executor.add_node(fixture)

        def spin():
            try:
                executor.spin()
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=spin, daemon=True)
        thread.start()

        def until(predicate, description, timeout=5):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert not errors, repr(errors)
                if predicate():
                    return
                time.sleep(0.01)
            assert predicate(), description

        until(lambda: fixture.route_pub.get_subscription_count() == 1
              and adapter.client.service_is_ready()
              and bool(adapter._fresh_navigation_session())
              and fixture.count_publishers(tx_topic) == 1,
              "isolated ROS graph did not discover adapter and navigation fixture")
        yield fixture, until
        assert not errors, repr(errors)
    finally:
        if fixture is not None:
            fixture.cancel_release.set()
        if executor is not None:
            executor.shutdown(timeout_sec=3)
        if thread is not None:
            thread.join(timeout=3)
        if adapter is not None:
            adapter.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize("replacement_speeds", [(2, 4, 2), (2, 3, 2)])
def test_route_reexecution_waits_for_cancel_and_correlates_completion(
        ros_pair, replacement_speeds):
    fixture, until = ros_pair
    first_json = _route()
    # One publish represents one external execution request, including an
    # unchanged route, header sequence, and header time on subsequent receipts.
    fixture.route_pub.publish(String(data=first_json))
    until(lambda: len(fixture.requests) == 1, "first NavigateRoute not received")
    first, first_task = fixture.requests[0]
    until(lambda: fixture.has_notice("ACTIVE", first.request_id), "first execution not acknowledged")
    assert first.waypoint_radius == 1.5
    assert len(first.waypoints) == 3
    assert [point.desired_speed_mps for point in first.waypoints] == pytest.approx(
        [2 * 1852 / 3600, 4 * 1852 / 3600, 2 * 1852 / 3600])
    assert [(point.pose.x, point.pose.y) for point in first.waypoints] == pytest.approx([
        (83.38367674866956, 91.682939134985),
        (83.38359129340336, 102.76910894104553),
        (83.38350583759512, 113.85527891640633),
    ])

    replacement_json = (
        first_json if replacement_speeds == (2, 4, 2)
        else _route(replacement_speeds)
    )
    fixture.route_pub.publish(String(data=replacement_json))
    until(fixture.cancel_started.is_set, "new 0211 did not request cancellation")
    time.sleep(0.15)
    assert len(fixture.requests) == 1, "replacement submitted before stop confirmation"
    # An old arrival received during replacement must not report route completion.
    fixture.set_status("GOAL_REACHED", first.request_id, first_task)
    time.sleep(0.15)
    assert fixture.feedback == []
    assert len(fixture.requests) == 1

    fixture.cancel_release.set()
    until(lambda: len(fixture.requests) == 2, "replacement missing after stop confirmation")
    second, second_task = fixture.requests[1]
    assert second.request_id != first.request_id
    assert [point.desired_speed_mps for point in second.waypoints] == pytest.approx(
        [speed * 1852 / 3600 for speed in replacement_speeds])
    assert [point.pose for point in second.waypoints] == [
        point.pose for point in first.waypoints]
    until(lambda: fixture.has_notice("ACTIVE", second.request_id), "replacement not acknowledged")
    # Late old statuses after the replacement was accepted are also harmless.
    fixture.nav_pub.publish(String(data=json.dumps(
        fixture.make_status("GOAL_REACHED", first.request_id, first_task))))
    time.sleep(0.15)
    assert fixture.feedback == []

    fixture.set_status("NAVIGATING", second.request_id, second_task, waypoint_index=1)
    until(lambda: fixture.has_notice("ACTIVE", second.request_id, waypoint_index=1),
          "intermediate route progress was not reported")
    time.sleep(0.15)
    assert fixture.feedback == [], "intermediate progress completed the route"

    fixture.set_status("GOAL_REACHED", second.request_id, second_task)
    until(lambda: len(fixture.feedback) == 1, "matching arrival did not publish 0052")
    time.sleep(0.2)  # Repeated navigation status must not repeat the completion.
    assert fixture.feedback == [{"travelId": 58, "taskFinish": 1}]

    fixture.route_pub.publish(String(data=replacement_json))
    until(lambda: len(fixture.requests) == 3, "completed route was not executable again")
    third, _ = fixture.requests[2]
    until(lambda: fixture.has_notice("ACTIVE", third.request_id), "third execution not acknowledged")
    assert len({first.request_id, second.request_id, third.request_id}) == 3
    assert fixture.cancel_count == 1, "completed execution does not require another stop"
    assert fixture.feedback == [{"travelId": 58, "taskFinish": 1}]


@pytest.mark.parametrize('first_hold_state', ['GUARDING', 'WAITING_MAP'])
def test_guard_arrival_feedback_is_once_per_execution_and_guard_stays_active(
        ros_pair, first_hold_state):
    fixture, until = ros_pair
    payload = json.loads(_route())
    payload['content'].update(taskType=80, guardingHeading=None)
    route_json = json.dumps(payload)
    fixture.route_pub.publish(String(data=route_json))
    until(lambda: len(fixture.requests) == 1, 'guard route was not submitted')
    first, first_task = fixture.requests[0]
    assert first.guard_on_arrival and not first.guard_heading_enabled
    until(lambda: fixture.has_notice('ACTIVE', first.request_id), 'guard route not acknowledged')

    fixture.set_status('NAVIGATING', first.request_id, first_task, waypoint_index=2)
    until(lambda: fixture.has_notice('ACTIVE', first.request_id, waypoint_index=2),
          'intermediate progress missing')
    assert fixture.feedback == []

    # HOLD remains proof of terminal arrival when the first GUARDING packet
    # was missed and the next observed micro-control state is a safety wait.
    with fixture.lock:
        fixture.status = fixture.make_status(
            first_hold_state, first.request_id, first_task,
            waypoint_count=3, waypoint_index=3, passed_waypoints=3)
        fixture.status['motion_mode'] = 'HOLD'
    until(lambda: len(fixture.feedback) == 1, 'guard arrival did not publish 0052')
    fixture.set_status('GUARDING', first.request_id, first_task, waypoint_index=3)
    until(lambda: fixture.has_notice('GUARDING', first.request_id), 'guard did not stay active')
    time.sleep(0.2)
    assert fixture.feedback == [{'travelId': 58, 'taskFinish': 1}]
    assert not fixture.has_notice('COMPLETED', first.request_id)

    # Identical business content is a new execution, not a duplicate status.
    # It must cancel the still-running guard, then earn its own arrival ACK.
    fixture.route_pub.publish(String(data=route_json))
    until(fixture.cancel_started.is_set, '0052 incorrectly ended the guard task')
    time.sleep(0.15)
    assert len(fixture.requests) == 1
    assert len(fixture.feedback) == 1
    fixture.cancel_release.set()
    until(lambda: len(fixture.requests) == 2, 'new guard route not submitted after cancellation')
    second, second_task = fixture.requests[1]
    assert second.request_id != first.request_id
    until(lambda: fixture.has_notice('ACTIVE', second.request_id), 'new guard route not acknowledged')
    fixture.nav_pub.publish(String(data=json.dumps(fixture.make_status(
        'GUARDING', first.request_id, first_task, waypoint_count=3, waypoint_index=3))))
    time.sleep(0.15)
    assert len(fixture.feedback) == 1
    fixture.set_status('GUARDING', second.request_id, second_task, waypoint_index=3)
    until(lambda: len(fixture.feedback) == 2, 'new execution did not get its own 0052')
    time.sleep(0.2)
    assert fixture.feedback == [{'travelId': 58, 'taskFinish': 1}] * 2
    assert fixture.cancel_count == 1
    assert not fixture.has_notice('COMPLETED', second.request_id)
