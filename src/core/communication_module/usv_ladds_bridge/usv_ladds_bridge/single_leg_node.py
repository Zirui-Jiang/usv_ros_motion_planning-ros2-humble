"""Translate one 0211 voyage into one atomic route navigation request."""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from usv_navigation_interfaces.msg import RouteWaypoint as RouteWaypointMessage
from usv_navigation_interfaces.srv import NavigateRoute
from common.geodesy import GeoReference

from .leg_state import LegBook
from .route_task import parse_route
from .single_leg import make_completion


NAVIGATION_STATUS_FRESHNESS = 2.5


class SingleLegNode(Node):
    def __init__(self):
        super().__init__('ladds_single_leg')
        defaults = dict(origin_latitude=30.5453, origin_longitude=114.392931,
                        map_frame='otter/map', navigation_namespace='/otter',
                        receive_topic='/ladds/rx', transmit_topic='/ladds/tx',
                        status_topic='/ladds/task_status', service_timeout=5.0,
                        waypoint_radius=1.5)
        values = {k: self.declare_parameter(
            k, v, ParameterDescriptor(read_only=True)).value for k, v in defaults.items()}
        self.origin = GeoReference(values['origin_latitude'], values['origin_longitude'])
        self.frame = values['map_frame']
        if not self.frame:
            raise ValueError('map_frame is empty')
        namespace = values['navigation_namespace'].rstrip('/')
        self.client = self.create_client(
            NavigateRoute, namespace + '/navigation/navigate_route')
        self.cancel_client = self.create_client(Trigger, namespace + '/cancel_navigation')
        self.tx = self.create_publisher(String, values['transmit_topic'], 10)
        self.status_pub = self.create_publisher(String, values['status_topic'], 10)
        self.create_subscription(String, values['receive_topic'], self._receive, 10)
        self.create_subscription(String, namespace + '/navigation_status', self._navigation, 10)
        self.book = LegBook()
        self.future = None
        self.submitted_at = 0.
        self.submitted_status_serial = 0
        self.retry_after_status_serial = 0
        self.navigation_status_serial = 0
        self.navigation_status_at = float('-inf')
        self.navigation_session = ''
        self.retired_navigation_sessions = deque(maxlen=8)
        self.waiting_record = None
        self.navigation_status = {}
        self.cancel_future = None
        self.cancel_required = False
        self.cancel_record = None
        self.cancel_generation = 0
        self.cancel_started_at = 0.
        self.cancel_retry_at = 0.
        self.cancel_timed_out = False
        self.cancel_session = ''
        self.service_timeout = float(values['service_timeout'])
        if not 0 < self.service_timeout <= 60:
            raise ValueError('service_timeout must be in (0, 60] seconds')
        self.waypoint_radius = float(values['waypoint_radius'])
        if not math.isfinite(self.waypoint_radius) or self.waypoint_radius <= 0:
            raise ValueError('waypoint_radius must be a positive finite number')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self._tick, clock=self.steady_clock)
        self.last_warning = ('', 0.)
        self.get_logger().info(
            f'0211 route adapter: ENU origin {self.origin.latitude}, '
            f'{self.origin.longitude}; frame={self.frame}; speed unit=knots; '
            f'waypoint radius={self.waypoint_radius} m; '
            'configure DDS receive=0211 and send=0052; '
            'each received 0211 starts a new execution, so send once per execution')

    def _notice(self, state, reason='', record=None):
        data = dict(state=state, reason=reason)
        if record is not None:
            points = record.leg.points
            count = len(points)
            index, passed = 0, 0
            radius = self.waypoint_radius
            status = self.navigation_status
            modules = status.get('modules') if isinstance(status, dict) else None
            correlated = (
                isinstance(modules, dict)
                and modules.get('decision') == record.planner_session
                and status.get('request_id') == record.request_id
                and (record.task_id < 0 or status.get('task_id') == record.task_id)
                and status.get('waypoint_count') == count
            )
            if correlated:
                candidate_index = status.get('waypoint_index')
                candidate_passed = status.get('passed_waypoints')
                candidate_radius = status.get('waypoint_radius')
                if type(candidate_index) is int and 0 <= candidate_index <= count:
                    index = candidate_index
                if type(candidate_passed) is int and 0 <= candidate_passed <= count:
                    passed = candidate_passed
                if (type(candidate_radius) in (int, float)
                        and math.isfinite(candidate_radius) and candidate_radius > 0):
                    radius = float(candidate_radius)
            target = points[min(index, count - 1)]
            data.update(travelId=record.leg.travel_id, request_id=record.request_id,
                        target_x=target.x, target_y=target.y,
                        desired_speed_mps=target.speed_mps,
                        waypoint_count=count, waypoint_index=index,
                        current_waypoint=min(index + 1, count),
                        passed_waypoints=passed, waypoint_radius=radius,
                        planner_session=record.planner_session, task_id=record.task_id)
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def _warning(self, text):
        now = time.monotonic()
        if self.last_warning[0] != text or now - self.last_warning[1] >= 5:
            self.get_logger().warning(text)
            self.last_warning = (text, now)

    def _receive(self, message):
        try:
            leg = parse_route(message.data, self.origin)
        except (ValueError, TypeError) as error:
            self._notice('INVALID_TASK', str(error))
            self._warning(str(error))
            return
        # The sender sends once per execution. Even identical 0211 content is
        # an explicit restart, while service retries retain their request_id.
        if self.waiting_record is not None:
            previous = self.waiting_record
            previous.state = 'ABORTED'
            previous.reason = 'Replaced by a newer 0211 before navigation started'
            self._notice(previous.state, previous.reason, previous)
        self.waiting_record = self.book.prepare(leg)
        if self.book.active is not None:
            self.cancel_required = True
            self.cancel_record = self.book.active
        self._advance()

    def _advance(self):
        """Progress one received execution without requiring another DDS packet."""
        waiting = self.waiting_record
        if waiting is None:
            return
        session = self._fresh_navigation_session()
        if not session:
            self._notice('WAITING_NAVIGATION', 'Waiting for a fresh navigation status', waiting)
            return
        active = self.book.active
        if active is not None and active.task_id < 0:
            # Cancelling before a delayed NavigateRoute request is accepted could
            # let that request start after cancellation and after its replacement.
            self._notice('WAITING_PREVIOUS_ACCEPTANCE', active.reason, waiting)
            return
        if active is not None or self.navigation_status.get('goal') is not None:
            self.cancel_required = True
            if active is not None:
                self.cancel_record = active
        if self.cancel_required:
            self._cancel_previous(session)
            return
        if not self.client.service_is_ready():
            self._notice('WAITING_NAVIGATION', 'Navigation service unavailable', waiting)
            return
        self.waiting_record = None
        self.book.activate(waiting, planner_session=session)
        self._submit(waiting)

    def _cancel_previous(self, session):
        if self.cancel_future is not None or time.monotonic() < self.cancel_retry_at:
            return
        if not self.cancel_client.service_is_ready():
            self._notice('WAITING_CANCEL', 'Navigation cancellation service unavailable',
                         self.waiting_record)
            return
        self.cancel_generation += 1
        generation = self.cancel_generation
        self.cancel_session = session
        self.cancel_started_at = time.monotonic()
        self.cancel_timed_out = False
        try:
            future = self.cancel_client.call_async(Trigger.Request())
            self.cancel_future = future
            future.add_done_callback(
                lambda result: self._cancelled(session, generation, result))
            self._notice('CANCELLING', 'Waiting for confirmed zero thrust before replacement',
                         self.waiting_record)
        except Exception as error:
            self.cancel_retry_at = time.monotonic() + 2.
            self._notice('WAITING_CANCEL', str(error), self.waiting_record)

    def _cancelled(self, session, generation, future):
        if (self.cancel_future is not future or generation != self.cancel_generation
                or session != self.navigation_session):
            return
        try:
            response = future.result()
        except Exception as error:
            # No request identity exists on Trigger: an unresolved old cancel
            # must never be allowed to run later against the replacement.
            self.cancel_timed_out = True
            self._notice('CANCEL_UNKNOWN', str(error), self.waiting_record)
            return
        self.cancel_future = None
        if not response.success:
            self.cancel_retry_at = time.monotonic() + 2.
            self._notice('WAITING_CANCEL', response.message, self.waiting_record)
            return
        previous = self.cancel_record
        if previous is not None:
            self.book.abort(previous, 'Replaced by a newly received 0211')
            # A matching CANCELLED/GOAL_REACHED status may have arrived first.
            previous.state, previous.reason = 'ABORTED', 'Replaced by a newly received 0211'
            self._notice(previous.state, previous.reason, previous)
        self.cancel_required = False
        self.cancel_record = None
        self.cancel_retry_at = 0.
        # Cancel success is newer than the cached pre-cancel status, and
        # already certifies goal clearing plus actual zero-output acknowledgement.
        self.navigation_status = dict(self.navigation_status, goal=None)
        self._advance()

    def _submit(self, record):
        try:
            generation = self.book.begin_submission(record.request_id)
        except ValueError:
            return
        request = NavigateRoute.Request()
        request.request_id = record.request_id
        request.header.frame_id = self.frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.waypoint_radius = self.waypoint_radius
        for point in record.leg.points:
            waypoint = RouteWaypointMessage()
            waypoint.pose.x = point.x
            waypoint.pose.y = point.y
            # A type=1 point has no heading constraint in this protocol subset.
            waypoint.pose.theta = 0.0
            waypoint.desired_speed_mps = point.speed_mps
            request.waypoints.append(waypoint)
        self.submitted_at = time.monotonic()
        self.submitted_status_serial = self.navigation_status_serial
        try:
            future = self.client.call_async(request)
            self.future = future
            future.add_done_callback(
                lambda result: self._accepted(record.request_id, generation, result))
        except Exception as error:
            self.book.submission_failed(record.request_id, generation, str(error))
            self.retry_after_status_serial = self.navigation_status_serial
        self._notice(record.state, record.reason, record)

    def _accepted(self, request_id, generation, future):
        record = self.book.active
        if (record is None or record.request_id != request_id
                or record.generation != generation):
            return
        try:
            response = future.result()
            if response.planner_session in self.retired_navigation_sessions:
                return
            if response.accepted and response.planner_session:
                # The service can discover a restarted decision node before its
                # first status arrives. Preserve this current request's binding.
                self._select_navigation_session(response.planner_session)
            self.book.acknowledge(request_id, response.accepted, response.planner_session,
                                  response.task_id, response.reason, generation)
        except Exception as error:
            self.book.submission_failed(request_id, generation, str(error))
            self.retry_after_status_serial = self.navigation_status_serial
        self._notice(record.state, record.reason, record)
        if (record.state == 'REJECTED' and record.reason.upper().startswith('BUSY')
                and record.task_id < 0):
            # A GUI target may win between an IDLE status and our service call.
            # Keep this received execution unless a newer packet superseded it.
            if self.waiting_record is None and self.book.defer_busy(record):
                self.waiting_record = record
            self.cancel_required = True
            self.cancel_record = None
        elif record.state in {'REJECTED', 'ABORTED'} and record.task_id < 0:
            self.cancel_required = False
            self.cancel_record = None
        if self.future is future:
            self.future = None
        self._advance()

    def _navigation(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        modules = status.get('modules') if isinstance(status, dict) else None
        session = modules.get('decision') if isinstance(modules, dict) else None
        if isinstance(session, str) and session:
            if not self._select_navigation_session(session):
                return
            self.navigation_status = status
            self.navigation_status_at = time.monotonic()
            self.navigation_status_serial += 1
        previous = self.book.active
        completed = self.book.observe(status)
        if completed is not None:
            if self.waiting_record is None and not self.cancel_required:
                self._complete(completed)
            else:
                completed.state, completed.reason = 'ABORTED', 'Replaced by a newly received 0211'
                self._notice(completed.state, completed.reason, completed)
        elif previous is not None:
            self._notice(previous.state, previous.reason, previous)
        record = self.book.active
        if (record is not None and record.state == 'RETRY_PENDING'
                and record.planner_session == session
                and self.navigation_status_serial > self.retry_after_status_serial
                and self.client.service_is_ready()
                and not (self.waiting_record is not None and record.task_id >= 0)):
            self._submit(record)
        self._advance()

    def _select_navigation_session(self, session):
        if session in self.retired_navigation_sessions:
            return False
        if session != self.navigation_session:
            if self.navigation_session:
                self.retired_navigation_sessions.append(self.navigation_session)
            self.navigation_session = session
            self.navigation_status = {}
            self.navigation_status_at = float('-inf')
            self.cancel_generation += 1
            self.cancel_future = None
            self.cancel_required = False
            self.cancel_record = None
            self.cancel_retry_at = 0.
        return True

    def _complete(self, record):
        # 0052 is the documented business JSON, without inventing unspecified
        # common-header fields. Only matching arrival completes this execution.
        self.tx.publish(String(data=json.dumps(make_completion(record.leg), ensure_ascii=False)))
        self._notice('COMPLETED', record.reason, record)

    def _tick(self):
        record = self.book.active
        if record is not None and record.state == 'SUBMITTING':
            if time.monotonic() - self.submitted_at > self.service_timeout:
                self.book.submission_timed_out(record.request_id, record.generation)
                self.retry_after_status_serial = self.navigation_status_serial
                self._notice(record.state, record.reason, record)
        if (self.cancel_future is not None and not self.cancel_timed_out
                and time.monotonic() - self.cancel_started_at > self.service_timeout):
            self.cancel_timed_out = True
            self._notice('CANCEL_UNKNOWN',
                         'Cancellation reply timed out; waiting for its result before starting replacement',
                         self.waiting_record)
        self._advance()

    def _fresh_navigation_session(self):
        if (not self.navigation_session
                or time.monotonic() - self.navigation_status_at
                > NAVIGATION_STATUS_FRESHNESS):
            return ''
        return self.navigation_session


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SingleLegNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
