"""0211 航路适配节点：把完整 points 提交到 NavigateRoute。"""
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
from usv_navigation_interfaces.srv import NavigateRoute, StopNavigation
from common.geodesy import GeoReference

from .leg_state import LegBook
from .route_task import parse_route
from .single_leg import make_completion
from .stop_task import StopExecution, parse_stop


NAVIGATION_STATUS_FRESHNESS = 2.5


# 0211 航路适配：把完整 points 提交到 NavigateRoute。
class RouteNode(Node):
    def __init__(self):
        super().__init__('usv_ladds_route')
        defaults = dict(origin_latitude=30.5453, origin_longitude=114.392931,
                        map_frame='otter/map', navigation_namespace='/otter',
                        receive_topic='/ladds/rx', transmit_topic='/ladds/tx',
                        stop_receive_topic='/ladds/stop_rx',
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
        self.stop_client = self.create_client(StopNavigation, namespace + '/navigation/stop')
        self.tx = self.create_publisher(String, values['transmit_topic'], 10)
        self.status_pub = self.create_publisher(String, values['status_topic'], 10)
        self.create_subscription(String, values['receive_topic'], self._receive, 10)
        self.create_subscription(String, values['stop_receive_topic'], self._receive_stop, 10)
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
        self.stop_record = None
        self.stop_futures = set()
        self.stop_submitted_at = 0.
        self.stop_retry_at = 0.
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
            'configure DDS receive=0211+0331 and send=0052; '
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
            data.update(taskType=getattr(record.leg, 'task_type', 16),
                        guard_on_arrival=getattr(record.leg, 'guard_on_arrival', False),
                        guard_heading_yaw=getattr(record.leg, 'guard_heading', None),
                        warnings=list(getattr(record.leg, 'warnings', ())))
        elif self.stop_record is not None:
            data.update(protocol='0331', request_id=self.stop_record.request_id,
                        planner_session=self.stop_record.planner_session,
                        task_id=self.stop_record.task_id)
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
        for warning in leg.warnings:
            self._warning(warning)
            self._notice('UNAPPLIED_CONSTRAINT', warning, self.waiting_record)
        if self.book.active is not None:
            self.cancel_required = True
            self.cancel_record = self.book.active
        self._advance()

    def _receive_stop(self, message):
        """0331 preempts even an unacknowledged route; no fresh pose/status gate."""
        try:
            warnings = parse_stop(message.data)
        except (ValueError, TypeError) as error:
            self._notice('INVALID_STOP', str(error))
            self._warning(str(error))
            return
        for warning in warnings:
            self._warning(warning)
        previous = self.book.active
        waiting = self.waiting_record
        if waiting is not None:
            waiting.state, waiting.reason = 'ABORTED', 'Superseded by 0331 active braking'
            self._notice(waiting.state, waiting.reason, waiting)
            self.waiting_record = None
        if previous is not None:
            self.book.abort(previous, 'Superseded by 0331 active braking')
            self._notice(previous.state, previous.reason, previous)
        # Late NavigateRoute callbacks now fail the active-record identity test.
        # Decision also blacklists these IDs so a delayed request cannot start.
        self.future = None
        self.cancel_required = False
        self.cancel_record = None
        if self.stop_record is None or self._stop_ready():
            self.stop_record = StopExecution(tuple(self.book.records))
            self.stop_retry_at = 0.
        # Repeated 0331 during braking is idempotent and cancels queued routes.
        self._try_stop()

    def _stop_ready(self):
        stop = self.stop_record
        return (stop is not None and stop.accepted and stop.stopped
                and not self.stop_futures and self.cancel_future is None)

    def _try_stop(self):
        stop = self.stop_record
        if stop is None or stop.accepted:
            return
        now = time.monotonic()
        if now < self.stop_retry_at:
            return
        if stop.attempts >= 4:
            stop.state = 'STOP_UNKNOWN'
            self._notice(stop.state, 'Stop reply unknown after retries; navigation remains blocked')
            return
        if not self.stop_client.service_is_ready():
            self._notice('WAITING_STOP', 'Active-braking service unavailable; navigation remains blocked')
            return
        request = StopNavigation.Request()
        request.request_id = stop.request_id
        request.superseded_request_ids = list(stop.superseded_request_ids)
        stop.attempts += 1
        stop.state = 'STOP_REQUESTED'
        self.stop_submitted_at = now
        self.stop_retry_at = now + self.service_timeout
        try:
            future = self.stop_client.call_async(request)
            self.stop_futures.add(future)
            future.add_done_callback(lambda result: self._stop_accepted(stop, result))
        except Exception as error:
            self._notice('STOP_UNKNOWN', str(error))
            return
        self._notice(stop.state, 'Active braking requested; acceptance is not stopped confirmation')

    def _stop_accepted(self, stop, future):
        self.stop_futures.discard(future)
        if self.stop_record is not stop:
            return
        try:
            response = future.result()
        except Exception as error:
            self._notice('STOP_UNKNOWN', str(error))
            return
        if response.planner_session in self.retired_navigation_sessions:
            return
        if not response.accepted:
            if not stop.accepted:
                stop.state = 'STOP_REJECTED'
            self._notice(stop.state, response.reason)
            return
        self._select_navigation_session(response.planner_session)
        if stop.task_id >= 0 and stop.task_id != response.task_id:
            stop.accepted = False
            stop.stopped = False
            stop.attempts = 4
            self._notice('STOP_UNKNOWN', 'Stop response and status disagree on task ID')
            return
        stop.accepted = True
        stop.planner_session = response.planner_session
        stop.task_id = response.task_id
        if not stop.stopped:
            stop.state = 'STOPPING'
        self._notice(stop.state, response.reason)
        self._advance()

    def _observe_stop(self, status):
        stop = self.stop_record
        if stop is None or not isinstance(status, dict):
            return
        modules = status.get('modules', {})
        if not isinstance(modules, dict):
            return
        session, task_id = modules.get('decision'), status.get('task_id')
        if (status.get('request_id') != stop.request_id
                or not isinstance(session, str) or not session
                or type(task_id) is not int or task_id < 0
                or (stop.planner_session and stop.planner_session != session)
                or (stop.task_id >= 0 and stop.task_id != task_id)):
            return
        stop.planner_session, stop.task_id = session, task_id
        if status.get('state') == 'STOPPED':
            stop.stopped = True
            stop.state = 'STOPPED'
            self._notice('STOPPED', 'Navigation confirmed low speed; no position hold; no 0052')
        elif status.get('state') == 'STOPPING':
            stop.state = 'STOPPING'

    def _advance(self):
        """Progress one received execution without requiring another DDS packet."""
        if self.stop_record is not None:
            self._try_stop()
            if not self._stop_ready():
                if self.waiting_record is not None:
                    self._notice('WAITING_STOP',
                                 'Waiting for STOPPED and all earlier stop/cancel replies',
                                 self.waiting_record)
                return
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
        self.stop_record = None
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
        if self.stop_record is not None:
            # This Trigger predates 0331. Decision refuses to cancel BRAKE;
            # wait for its result before allowing any later route to start.
            try:
                future.result()
            except Exception as error:
                self.cancel_timed_out = True
                self._notice('CANCEL_UNKNOWN', str(error), self.waiting_record)
                return
            self.cancel_future = None
            self.cancel_required = False
            self._advance()
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
        request.guard_on_arrival = getattr(record.leg, 'guard_on_arrival', False)
        guard_heading = getattr(record.leg, 'guard_heading', None)
        request.guard_heading_enabled = guard_heading is not None
        request.guard_heading = 0.0 if guard_heading is None else guard_heading
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
        self._observe_stop(status)
        previous = self.book.active
        arrived = self.book.observe(status)
        if arrived is not None:
            if self.waiting_record is None and not self.cancel_required:
                self._report_arrival(arrived)
            else:
                reason = 'Replaced by a newly received 0211'
                self.book.abort(arrived, reason)
                arrived.state, arrived.reason = 'ABORTED', reason
                self._notice(arrived.state, arrived.reason, arrived)
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
            stop = self.stop_record
            if stop is not None and stop.planner_session and stop.planner_session != session:
                # Decision restarted: repeat the same stop identity on the new
                # process. Never infer STOPPED from loss of the previous server.
                stop.planner_session = ''
                stop.task_id = -1
                stop.accepted = False
                stop.stopped = False
                stop.attempts = 0
                self.stop_retry_at = 0.
        return True

    def _report_arrival(self, record):
        # 0052 is the documented business JSON, without inventing unspecified
        # common-header fields. A guard route reports arrival but keeps holding.
        self.tx.publish(String(data=json.dumps(make_completion(record.leg), ensure_ascii=False)))
        self._notice(record.state, record.reason, record)

    def _tick(self):
        if (self.stop_record is not None and not self.stop_record.accepted
                and time.monotonic() - self.stop_submitted_at > self.service_timeout):
            self.stop_retry_at = 0.
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
        node = RouteNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
