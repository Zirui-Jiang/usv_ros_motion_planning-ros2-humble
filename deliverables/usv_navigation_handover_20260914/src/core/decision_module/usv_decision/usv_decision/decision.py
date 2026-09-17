"""Ordinary mission ownership and planning, independent of motion control."""
import json
import time
import numpy as np
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from common.models import AlgorithmStatus
from common.collision import footprint_path_is_clear
from common.visualizer import make_path
from usv_navigation_interfaces.msg import NavigationReference, ControlReport
from usv_navigation.configuration import DEFAULTS, HOT_PARAMETERS, C, T, validate_update
from .runtime import LatestPlanWorker, PlanningRequest
from usv_navigation.state_rules import fresh
from usv_navigation.protocol import set_message_trajectory, PEER_TIMEOUT
from usv_navigation.ros_support import NavigationInputs
from usv_navigation.node_runner import run_node


class DecisionNode(NavigationInputs):
    def __init__(self):
        # The external node and parameter-service names remain compatible.
        super().__init__('usv_simulation', role='decision')
        self.worker = LatestPlanWorker()
        self.version = self.task_id = self.sequence = 0
        self.goal = self.trajectory = None
        self.route = np.empty((0, 3))
        self.mode, self.reason = 'IDLE', ''
        self.controller_name = self.config[C + 'controller_name']
        self.last_clock = self.now_seconds()
        self.last_status_wall = 0.
        self.control_report = None
        self.control_report_wall = float('-inf')
        self.cancel_waiters = []
        self.path_pub = self.create_publisher(Path, 'global_path', 1)
        self.trajectory_pub = self.create_publisher(Path, 'local_trajectory', 1)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.reference_pub = self.create_publisher(NavigationReference, 'navigation/reference', 10)
        self.create_subscription(PoseStamped, 'goal', self._on_goal, 1)
        self.create_subscription(ControlReport, 'navigation/control_report', self._on_control_report, 10)
        self.service_group = ReentrantCallbackGroup()
        self.create_service(Trigger, 'cancel_navigation', self._cancel, callback_group=self.service_group)
        self.create_service(Trigger, 'export_parameters', self._export)
        self.create_timer(1. / self.config['navigation.planning_frequency'], self._planning)
        self.create_timer(1. / self.config[C + 'control_frequency'], self._tick, clock=self.steady_clock)

    def _apply_parameters(self):
        changes = {key: self.get_parameter(key).value for key in HOT_PARAMETERS
                   if self.get_parameter(key).value != self.config[key]}
        if not changes:
            return
        self.config = validate_update(self.config, changes)
        if any(key.startswith(T) for key in changes):
            self.version += 1
            self.worker.invalidate()
            self.trajectory = None
        self.counters['parameter_updates'] += 1
        self._publish_reference()

    def _trajectory_clear(self, trajectory):
        # Preserve acceptance checks against the latest state and map. The
        # periodic current-hull execution check is owned by the control node.
        return self._current_footprint_clear() and footprint_path_is_clear(
            self.grid, trajectory.points[:, [2, 3, 4]], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    def _map_updated(self):
        if self.trajectory is not None and not self._trajectory_clear(self.trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Latest map invalidated reference'
            self.counters['map_invalidations'] += 1
            self._publish_reference()

    def _reset_task(self, mode, reason='', clear_inputs=False, publish=True):
        self.version += 1
        self.task_id += 1
        self.worker.invalidate()
        self.goal = self.trajectory = None
        self.route = np.empty((0, 3))
        self.mode, self.reason = mode, reason
        self.control_report = None
        if clear_inputs:
            self._clear_inputs()
        if publish:
            self._publish_reference()
        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(make_path(self.route, self.frame, stamp))
        self.trajectory_pub.publish(make_path(self.route, self.frame, stamp))

    def _on_goal(self, message):
        # Publish the replacement once after validation. An inactive snapshot
        # closes its task ID at the controller and must never precede an active
        # snapshot with that same ID.
        self._reset_task('WAITING_REFERENCE', publish=False)
        try:
            self.goal = self._pose(message.pose, message.header.frame_id, message.header.stamp)
            self._publish_reference()
            self._planning()
        except (ValueError, TransformException) as error:
            self.mode, self.reason = 'INVALID_GOAL', str(error)
            self._publish_reference()

    async def _cancel(self, request, response):
        self._reset_task('CANCELLED')
        future = Future()
        self.cancel_waiters.append((self.task_id, self.sequence, time.monotonic() + 2., future))
        acknowledged = await future
        response.success = acknowledged
        response.message = ('Navigation cancelled; zero thrust active' if acknowledged else
                            'Navigation cancelled; control zero-output acknowledgement timed out')
        return response

    def _export(self, request, response):
        values = {key: self.get_parameter(key).value for key in DEFAULTS}
        response.success = True
        response.message = yaml.safe_dump({self.get_fully_qualified_name(): {'ros__parameters': values}}, sort_keys=True)
        return response

    def _on_peer_available(self, role):
        self._publish_reference()

    def _on_peer_restart(self, role):
        self._reset_task('MODULE_RESTART', role + ' restarted; submit a new goal')

    def _planning(self):
        now = self.now_seconds()
        if self.goal is None or self.state is None or self.grid is None:
            return
        if not fresh(self.state.stamp, now, self.config['navigation.state_timeout']) or not fresh(
                self.grid.stamp, now, self.config['navigation.map_timeout']):
            return
        try:
            self.worker.offer(PlanningRequest(self.version, self.goal, self.state, self.grid,
                                               self.route.copy(), dict(self.config)))
        except Exception as error:
            self.trajectory = None
            self.mode, self.reason = 'PLANNING_ERROR', str(error)
            self.counters['worker_errors'] += 1
            self._publish_reference()

    def _consume_plan(self):
        try:
            result = self.worker.poll()
        except Exception as error:
            self.trajectory = None
            self.mode, self.reason = 'PLANNING_ERROR', str(error)
            self.counters['worker_errors'] += 1
            self._publish_reference()
            return
        if result is None:
            return
        self.timings['planning_ms'].append(result.elapsed * 1000)
        if result.version != self.version or self.goal is None:
            self.counters['stale_results'] += 1
            return
        self.counters['global_searches'] += int(result.searched)
        self.route, self.reason = result.route, result.message
        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(make_path(self.route, self.frame, stamp))
        if result.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            self.mode, self.trajectory = result.status.value.upper(), None
        elif not self._trajectory_clear(result.trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Worker reference conflicts with latest map'
            self.counters['unsafe_results'] += 1
        else:
            self.trajectory = result.trajectory
            self.mode = 'NAVIGATING'
            self.trajectory_pub.publish(make_path(self.trajectory.points[:, [2, 3, 4]], self.frame, stamp))
        self._publish_reference()

    def _publish_reference(self, changed=True):
        if changed:
            self.sequence += 1
        message = NavigationReference(
            planner_session=self.session, control_session=self.peers['control'].session,
            perception_session=self.peers['perception'].session,
            task_id=self.task_id, revision=self.version, sequence=self.sequence,
            active=self.goal is not None, state=self.mode, reason=self.reason,
            parameters_json=json.dumps(self.config, sort_keys=True))
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        if self.goal is not None:
            message.goal.x, message.goal.y, message.goal.theta = self.goal
        set_message_trajectory(message, self.trajectory)
        self.reference_pub.publish(message)

    def _on_control_report(self, message):
        if (message.control_session != self.peers['control'].session
                or message.planner_session != self.session):
            self.counters['stale_control_reports'] += 1
            return
        # A later goal or cancellation must not hide a real zero ACK for an
        # earlier cancellation. Only current-task reports may update status.
        self._resolve_cancellations(message)
        if message.task_id != self.task_id:
            self.counters['stale_control_reports'] += 1
            return
        self.control_report, self.control_report_wall = message, time.monotonic()
        if message.clock_reset:
            self._reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
        elif message.task_finished and self.goal is not None:
            self._reset_task('GOAL_REACHED')
        elif message.revoke_reference and message.reference_sequence == self.sequence:
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = message.state, message.reason
            self._publish_reference()

    def _resolve_cancellations(self, report=None):
        pending = []
        for task, sequence, deadline, future in self.cancel_waiters:
            acknowledged = (report is not None and report.task_id == task
                            and report.reference_sequence >= sequence
                            and report.left == report.right == report.lateral == 0.)
            if acknowledged or time.monotonic() >= deadline:
                if not future.done():
                    future.set_result(bool(acknowledged))
            else:
                pending.append((task, sequence, deadline, future))
        self.cancel_waiters = pending

    def _tick(self):
        now = self.now_seconds()
        if now < self.last_clock - 1e-6:
            self._reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
            self.counters['clock_resets'] += 1
        self.last_clock = now
        self._apply_parameters()
        self._consume_odom()
        self._consume_plan()
        self._publish_reference(changed=False)
        self._resolve_cancellations()
        if time.perf_counter() - self.last_status_wall >= 1.:
            self._publish_status()

    def _publish_status(self):
        self.last_status_wall = time.perf_counter()
        self.counters['pending_replacements'] = self.worker.replaced
        metrics, counters, process = self.statistics()
        mode, reason = self.mode, self.reason
        control_process = {}
        report = self.control_report
        if report is not None and time.monotonic() - self.control_report_wall <= PEER_TIMEOUT:
            try:
                metrics.update(json.loads(report.metrics_json or '{}'))
                counters.update(json.loads(report.counters_json or '{}'))
                control_process = json.loads(report.process_json or '{}')
            except (ValueError, TypeError):
                self.counters['invalid_control_statistics'] += 1
            if self.goal is not None:
                if report.state != 'WAITING_REFERENCE' or self.trajectory is not None or not reason:
                    mode, reason = report.state, report.reason
        if self.goal is not None and not self.peers_ready('control', 'perception'):
            mode, reason = 'WAITING_MODULE', 'Waiting for onboard modules'
        self.status_pub.publish(String(data=json.dumps(dict(
            state=mode, reason=reason, version=self.version, stamp=self.now_seconds(),
            controller=self.controller_name, goal=self.goal, metrics=metrics, counters=counters,
            process=process, modules={'decision': self.session, 'control': self.peers['control'].session,
                                      'perception': self.peers['perception'].session,
                                      'control_process': control_process}))))

    def destroy_node(self):
        if self.context.ok():
            self._reset_task('SHUTDOWN')
        for _, _, _, future in self.cancel_waiters:
            if not future.done():
                future.set_result(False)
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    run_node(DecisionNode, args)
