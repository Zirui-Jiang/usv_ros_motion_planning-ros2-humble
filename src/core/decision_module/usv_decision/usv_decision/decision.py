# 普通导航决策节点拥有目标任务和规划生命周期，发布路径/轨迹参考并依据控制报告确认停止与到达。
"""Ordinary mission ownership and planning, independent of motion control."""
from collections import OrderedDict
import json
import math
import time
import numpy as np
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from common.models import AlgorithmStatus
from common.collision import footprint_path_is_clear
from common.visualizer import make_path
from usv_navigation_interfaces.msg import NavigationReference, ControlReport
from usv_navigation_interfaces.srv import NavigateTo, NavigateRoute, StopNavigation
from usv_navigation.configuration import DEFAULTS, HOT_PARAMETERS, C, T, validate_update
from trajectory_planner.trajectory import restart_trajectory_clock
from .runtime import LatestPlanWorker, PlanningRequest
from .route_progress import RouteProgress
from usv_navigation.state_rules import fresh
from usv_navigation.protocol import set_message_trajectory, PEER_TIMEOUT
from usv_navigation.ros_support import NavigationInputs
from usv_navigation.node_runner import run_node


# 保留外部节点名 usv_simulation；后台规划进程与控制执行分离，ROS 回调只投递和验收快照。
class DecisionNode(NavigationInputs):
    def __init__(self):
        # The external node and parameter-service names remain compatible.
        super().__init__('usv_simulation', role='decision')
        self.worker = LatestPlanWorker()
        self.version = self.task_id = self.sequence = 0
        self.goal = self.trajectory = None
        self.motion_mode = 'NAVIGATE'
        self.guard_on_arrival = self.guard_heading_enabled = False
        self.guard_heading = 0.
        self.route_progress = None
        self.route_summary = {}
        self._terminal_reference_sequence = None
        self.request_id = ''
        self.accepted_task_id = 0
        self.desired_speed_mps = self.model.cruising_speed
        # Keep a bounded in-process replay window. Idempotency intentionally
        # does not survive a process restart or eviction of very old tasks.
        self._accepted_requests = OrderedDict()
        # Stop fences must not expire while a delayed service request may exist.
        # Like the other task identities, these are scoped to this process session.
        self._superseded_requests = set()
        self._stop_requests = {}
        self.route = np.empty((0, 3))
        # 全局搜索按需触发；已验收的剩余航路变化及时显示，相同路线去重。
        self._published_global_path = np.empty((0, 3))
        self.mode, self.reason = 'IDLE', ''
        self.controller_name = self.config[C + 'controller_name']
        self.last_clock = self.now_seconds()
        self.last_status_wall = 0.
        self.control_report = None
        self.control_report_wall = float('-inf')
        self.cancel_waiters = []
        self.path_pub = self.create_publisher(
            Path, 'global_path', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.trajectory_pub = self.create_publisher(Path, 'local_trajectory', 1)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.reference_pub = self.create_publisher(NavigationReference, 'navigation/reference', 10)
        self.create_subscription(PoseStamped, 'goal', self._on_goal, 1)
        self.create_subscription(ControlReport, 'navigation/control_report', self._on_control_report, 10)
        self.service_group = ReentrantCallbackGroup()
        self.create_service(NavigateTo, 'navigation/navigate_to', self._navigate,
                            callback_group=self.service_group)
        self.create_service(NavigateRoute, 'navigation/navigate_route', self._navigate_route,
                            callback_group=self.service_group)
        self.create_service(Trigger, 'cancel_navigation', self._cancel, callback_group=self.service_group)
        self.create_service(StopNavigation, 'navigation/stop', self._stop,
                            callback_group=self.service_group)
        self.create_service(Trigger, 'export_parameters', self._export)
        self.create_timer(1. / self.config['navigation.planning_frequency'], self._planning)
        self.create_timer(1. / self.config[C + 'control_frequency'], self._tick, clock=self.steady_clock)

    # PID 热参数随参考同步；LOS 热参数另增规划版本并使旧轨迹失效，但不更换任务号。
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

    # 验收参考时同时检查最新实测船体和参考扫掠船体；逐拍执行校验由控制节点负责。
    def _trajectory_clear(self, trajectory):
        # Preserve acceptance checks against the latest state and map. The
        # periodic current-hull execution check is owned by the control node.
        return self._current_footprint_clear() and footprint_path_is_clear(
            self.grid, trajectory.points[:, [2, 3, 4]], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    def _publish_global_path(self, stamp, *, path=None, force=False):
        """只发布已验收的航路变化；相同路径和重复清空都不重发。"""
        path = self.route if path is None else path
        if force or not np.array_equal(path, self._published_global_path):
            self.path_pub.publish(make_path(path, self.frame, stamp))
            self._published_global_path = path.copy()

    # 新地图阻断已有参考时立即清空路线与轨迹，并通知控制端等待新参考。
    def _map_updated(self):
        if self.trajectory is not None and not self._trajectory_clear(self.trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Latest map invalidated reference'
            self.counters['map_invalidations'] += 1
            self._publish_global_path(self.get_clock().now().to_msg())
            self._publish_reference()

    # 增加版本与任务号，失效化排队规划，清空可视化和运行状态，再发布当前任务快照。
    def _reset_task(self, mode, reason='', clear_inputs=False, publish=True):
        self._snapshot_route_progress()
        self.route_progress = None
        self._terminal_reference_sequence = None
        self.version += 1
        self.task_id += 1
        self.worker.invalidate()
        self.goal = self.trajectory = None
        self.motion_mode = 'NAVIGATE'
        self.guard_on_arrival = self.guard_heading_enabled = False
        self.guard_heading = 0.
        self.route = np.empty((0, 3))
        self.mode, self.reason = mode, reason
        self.control_report = None
        if clear_inputs:
            self._clear_inputs()
        if publish:
            self._publish_reference()
        stamp = self.get_clock().now().to_msg()
        self._publish_global_path(stamp, force=True)
        self.trajectory_pub.publish(make_path(self.route, self.frame, stamp))

    # 替换目标先清理旧任务，新目标验证后一次性发布；同一任务号不能先发停止再发活动参考。
    def _on_goal(self, message):
        if self.motion_mode == 'BRAKE':
            self.counters['goals_rejected_while_braking'] += 1
            return
        # Publish the replacement once after validation. An inactive snapshot
        # closes its task ID at the controller and must never precede an active
        # snapshot with that same ID.
        self._reset_task('WAITING_REFERENCE', publish=False)
        self.request_id = ''
        self.accepted_task_id = 0
        self.route_summary = {}
        self.desired_speed_mps = self.model.cruising_speed
        try:
            self.goal = self._pose(message.pose, message.header.frame_id, message.header.stamp)
            self._publish_reference()
            self._planning()
        except (ValueError, TransformException) as error:
            self.mode, self.reason = 'INVALID_GOAL', str(error)
            self._publish_reference()

    # 原子校验外部请求：重复请求只返回首次任务号，不重启已执行或已完成任务。
    def _navigate(self, request, response):
        response.planner_session = self.session
        response.task_id = 0
        request_id = request.request_id
        if self._request_is_stopped(request_id):
            response.accepted, response.reason = False, 'Request superseded by an active stop'
            return response
        if not request_id.strip():
            response.accepted, response.reason = False, 'request_id must not be empty'
            return response
        try:
            desired_speed = float(request.desired_speed_mps)
            if not math.isfinite(desired_speed) or desired_speed <= 0.:
                raise ValueError('desired_speed_mps must be finite and greater than zero')
            if desired_speed > self.model.velocity_max:
                raise ValueError(
                    f'desired_speed_mps {desired_speed:g} exceeds velocity_max {self.model.velocity_max:g}')
            target = self._pose(request.goal.pose, request.goal.header.frame_id,
                                request.goal.header.stamp)
        except (ValueError, TransformException) as error:
            response.accepted, response.reason = False, str(error)
            return response

        signature = (target, desired_speed)
        previous = self._accepted_requests.get(request_id)
        if previous is not None:
            self._accepted_requests.move_to_end(request_id)
            previous_signature, previous_task_id = previous
            response.task_id = previous_task_id
            if previous_signature == signature:
                response.accepted, response.reason = True, 'IDEMPOTENT'
            else:
                response.accepted = False
                response.reason = 'request_id was already accepted with different parameters'
            return response
        if self.goal is not None or self.motion_mode == 'BRAKE':
            response.accepted, response.reason = False, 'BUSY: an active navigation goal already exists'
            return response

        # All validation is complete before the task lifecycle is mutated.
        self._reset_task('WAITING_REFERENCE', publish=False)
        self.route_summary = {}
        self.goal = target
        self.request_id = request_id
        self.accepted_task_id = self.task_id
        self.desired_speed_mps = desired_speed
        self._accepted_requests[request_id] = (signature, self.accepted_task_id)
        if len(self._accepted_requests) > 256:
            self._accepted_requests.popitem(last=False)
        self._publish_reference()
        self._planning()
        response.accepted, response.reason = True, 'ACCEPTED'
        response.task_id = self.accepted_task_id
        return response

    def _navigate_route(self, request, response):
        """Validate the complete ordered route before changing the active task."""
        response.planner_session, response.task_id = self.session, 0
        if self._request_is_stopped(request.request_id):
            response.accepted, response.reason = False, 'Request superseded by an active stop'
            return response
        try:
            if not request.request_id.strip():
                raise ValueError('request_id must not be empty')
            points = []
            for waypoint in request.waypoints:
                values = (waypoint.pose.x, waypoint.pose.y, waypoint.pose.theta,
                          waypoint.desired_speed_mps)
                if not all(map(math.isfinite, values)):
                    raise ValueError('Waypoint fields must be finite')
                if not 0. < values[3] <= self.model.velocity_max:
                    raise ValueError('Waypoint speed must be positive and within velocity_max')
                pose = Pose()
                pose.position.x, pose.position.y = values[:2]
                pose.orientation.z = math.sin(values[2] / 2.)
                pose.orientation.w = math.cos(values[2] / 2.)
                target = self._pose(pose, request.header.frame_id, request.header.stamp)
                points.append((*target, values[3]))
            progress = RouteProgress(points, request.waypoint_radius,
                                     max_speed=self.model.velocity_max, frame_id=self.frame)
            guard = bool(request.guard_on_arrival)
            heading_enabled = bool(request.guard_heading_enabled)
            if heading_enabled and not guard:
                raise ValueError('guard_heading_enabled requires guard_on_arrival')
            heading = 0.
            if heading_enabled:
                if not math.isfinite(request.guard_heading):
                    raise ValueError('guard_heading must be finite')
                # Transform the requested terminal yaw with the route frame,
                # rather than silently treating a non-map yaw as map yaw.
                pose.orientation.z = math.sin(request.guard_heading / 2.)
                pose.orientation.w = math.cos(request.guard_heading / 2.)
                heading = self._pose(pose, request.header.frame_id, request.header.stamp)[2]
        except (ValueError, TransformException) as error:
            response.accepted, response.reason = False, str(error)
            return response
        signature = ('route', progress.points, progress.radius, guard, heading_enabled, heading)
        previous = self._accepted_requests.get(request.request_id)
        if previous is not None:
            self._accepted_requests.move_to_end(request.request_id)
            response.task_id = previous[1]
            response.accepted = previous[0] == signature
            response.reason = ('IDEMPOTENT' if response.accepted else
                               'request_id was already accepted with different parameters')
            return response
        if self.goal is not None or self.motion_mode == 'BRAKE':
            response.accepted, response.reason = False, 'BUSY: an active navigation goal already exists'
            return response
        self._reset_task('WAITING_REFERENCE', publish=False)
        self.route_progress = progress
        self.guard_on_arrival, self.guard_heading_enabled = guard, heading_enabled
        self.guard_heading = heading
        self._snapshot_route_progress()
        self.goal, self.desired_speed_mps = progress.points[0][:3], progress.points[0][3]
        self.request_id, self.accepted_task_id = request.request_id, self.task_id
        self._accepted_requests[request.request_id] = (signature, self.task_id)
        if len(self._accepted_requests) > 256:
            self._accepted_requests.popitem(last=False)
        self._publish_reference()
        self._planning()
        response.accepted, response.reason, response.task_id = True, 'ACCEPTED', self.task_id
        return response

    def _request_is_stopped(self, request_id):
        return request_id in self._superseded_requests or request_id in self._stop_requests

    def _stop(self, request, response):
        """Preempt immediately; acceptance is not a measured stopped acknowledgement."""
        response.planner_session, response.task_id = self.session, 0
        request_id = request.request_id
        superseded = tuple(sorted(set(request.superseded_request_ids)))
        if (not request_id.strip() or len(request_id) > 256 or len(superseded) > 256
                or any(not value.strip() or len(value) > 256 for value in superseded)):
            response.accepted, response.reason = False, 'Invalid stop request identity'
            return response
        if request_id in self._accepted_requests:
            response.accepted, response.reason = False, 'Stop ID already used for navigation'
            return response
        previous = self._stop_requests.get(request_id)
        if previous is not None:
            response.task_id = previous[1]
            response.accepted = previous[0] == superseded
            response.reason = ('IDEMPOTENT' if response.accepted else
                               'Stop request_id was already accepted with different parameters')
            return response
        self._superseded_requests.update(superseded)
        if self.request_id:
            self._superseded_requests.add(self.request_id)
        self._reset_task('STOPPING', publish=False)
        self.motion_mode = 'BRAKE'
        self.request_id, self.accepted_task_id = request_id, self.task_id
        self._stop_requests[request_id] = (superseded, self.task_id)
        self._publish_reference()
        response.accepted, response.reason, response.task_id = True, 'ACCEPTED', self.task_id
        return response

    def _snapshot_route_progress(self):
        progress = self.route_progress
        if progress is not None:
            self.route_summary = dict(waypoint_count=len(progress.points),
                waypoint_index=progress.index, passed_waypoints=progress.passed_count,
                waypoint_radius=progress.radius)

    def _update_route_progress(self):
        if self.motion_mode != 'NAVIGATE':
            return
        progress = self.route_progress
        if progress is None or self.state is None or not fresh(
                self.state.stamp, self.now_seconds(), self.config['navigation.state_timeout']):
            return
        if progress.update(self.state):
            self.goal, self.desired_speed_mps = progress.remaining[0][:3], progress.remaining[0][3]
            # A waypoint is a planning revision, not a new control task. Keep the
            # safe existing reference while the worker computes the next one.
            self.version += 1
            # 到达盘已确认上一航点，新阶段不再回贴已完成的入航段；仅此次丢弃全局缓存。
            self.route = np.empty((0, 3))
            self.worker.invalidate()
            self._snapshot_route_progress()
            self._publish_reference()
            self._planning()

    # 取消服务等待对应任务和参考序号的实际零输出确认，墙钟两秒超时则返回失败。
    async def _cancel(self, request, response):
        # A late cancellation issued before 0331 must not turn off active braking.
        if self.motion_mode == 'BRAKE':
            response.success = False
            response.message = 'Active braking cannot be cancelled by navigation cancellation'
            return response
        if self.mode == 'STOPPED':
            self._publish_reference()
        else:
            self._reset_task('CANCELLED')
        future = Future()
        self.cancel_waiters.append((self.task_id, self.sequence, time.monotonic() + 2., future))
        acknowledged = await future
        response.success = acknowledged
        response.message = ('Navigation cancelled; zero thrust active' if acknowledged else
                            'Navigation cancelled; control zero-output acknowledgement timed out')
        return response

    # 导出当前节点有效参数的 ROS YAML 快照，区别于项目启动配置文件格式。
    def _export(self, request, response):
        values = {key: self.get_parameter(key).value for key in DEFAULTS}
        response.success = True
        response.message = yaml.safe_dump({self.get_fully_qualified_name(): {'ros__parameters': values}}, sort_keys=True)
        return response

    def _on_peer_available(self, role):
        self._publish_reference()

    def _on_peer_restart(self, role):
        if self.motion_mode == 'BRAKE':
            # Preserve the stop's external accepted ID, but fence the old
            # control instance with a new internal task ID and peer sessions.
            self._reset_task('STOPPING', role + ' restarted; awaiting fresh braking inputs',
                             publish=False)
            self.motion_mode = 'BRAKE'
            self._publish_reference()
        else:
            self._reset_task('MODULE_RESTART', role + ' restarted; submit a new goal')

    # 只在目标、状态和地图都存在且时间新鲜时投递副本，执行中的旧工作不阻塞回调。
    def _planning(self):
        now = self.now_seconds()
        if self.motion_mode != 'NAVIGATE':
            return
        if self.goal is None or self.state is None or self.grid is None:
            return
        if not fresh(self.state.stamp, now, self.config['navigation.state_timeout']) or not fresh(
                self.grid.stamp, now, self.config['navigation.map_timeout']):
            return
        try:
            self.worker.offer(PlanningRequest(self.version, self.goal, self.state, self.grid,
                                               self.route.copy(), dict(self.config),
                                               self.desired_speed_mps,
                                               self.route_progress.remaining if self.route_progress else (),
                                               self.route_progress.radius if self.route_progress else 1.5))
        except Exception as error:
            self.trajectory = None
            self.mode, self.reason = 'PLANNING_ERROR', str(error)
            self.counters['worker_errors'] += 1
            self._publish_reference()

    # 拒绝过期版本结果，并用最新地图复检工作进程产生的扫掠轨迹后才发布。
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
        if result.version != self.version or self.goal is None or self.motion_mode != 'NAVIGATE':
            self.counters['stale_results'] += 1
            return
        self.counters['global_searches'] += int(result.searched)
        self.reason = result.message
        stamp = self.get_clock().now().to_msg()
        # 轨迹时间从全局搜索结束这一刻起算，不沿用搜索开始时的船位时间。
        trajectory = restart_trajectory_clock(result.trajectory, self.now_seconds())
        if result.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            self.mode, self.trajectory = result.status.value.upper(), None
            # 局部轨迹失败可能仍带有已验证全局航路，保留候选供下一拍严格复检。
            # 候选缓存不代表可执行参考；先清空显示，成功恢复后再发布一次。
            self.route = result.route
            self._publish_global_path(stamp, path=np.empty((0, 3)))
        elif not self._trajectory_clear(trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Worker reference conflicts with latest map'
            self.counters['unsafe_results'] += 1
            self._publish_global_path(stamp)
        else:
            self.route = result.route
            self.trajectory = trajectory
            self.mode = 'NAVIGATING'
            # 显示本次已验收的裁剪结果。轨迹时钟已改到搜索结束时刻。
            self._publish_global_path(stamp)
            self.trajectory_pub.publish(make_path(self.trajectory.points[:, [2, 3, 4]], self.frame, stamp))
        self._publish_reference()

    # 改变内容时递增序号，定时重传时保持序号和轨迹源时间，不能靠重发延长轨迹有效期。
    def _publish_reference(self, changed=True):
        # BRAKE/HOLD have a fresh command lease, not a regenerating trajectory.
        if changed or self.motion_mode in ('BRAKE', 'HOLD'):
            self.sequence += 1
        message = NavigationReference(
            planner_session=self.session, control_session=self.peers['control'].session,
            perception_session=self.peers['perception'].session,
            task_id=self.task_id, revision=self.version, sequence=self.sequence,
            active=self.goal is not None or self.motion_mode == 'BRAKE',
            motion_mode=self.motion_mode, state=self.mode, reason=self.reason,
            guard_on_arrival=self.guard_on_arrival,
            guard_heading_enabled=self.guard_heading_enabled, guard_heading=self.guard_heading,
            parameters_json=json.dumps(self.config, sort_keys=True))
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        if self.goal is not None:
            message.goal.x, message.goal.y, message.goal.theta = self.goal
        set_message_trajectory(message, self.trajectory)
        if self.route_progress is not None:
            # Even after actual final-leg entry, a retained intermediate
            # trajectory cannot acquire terminal completion rights retroactively.
            message.pass_through = not (self.route_progress.terminal
                and self.trajectory is not None and self.trajectory.goal is not None)
            if not message.pass_through and self._terminal_reference_sequence is None:
                self._terminal_reference_sequence = self.sequence
            message.guard_on_arrival = self.guard_on_arrival and not message.pass_through
        self.reference_pub.publish(message)

    # 先验证会话并处理取消确认，再允许当前任务报告更新决策状态。
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
        if self.motion_mode == 'BRAKE' and message.task_finished:
            if (message.state != 'STOPPED' or message.reference_sequence <= 0
                    or message.reference_sequence > self.sequence
                    or message.left != 0. or message.right != 0. or message.lateral != 0.):
                self.counters['stale_control_reports'] += 1
                return
        if self.motion_mode == 'HOLD' and message.task_finished:
            # Delayed terminal navigation reports cannot close persistent guard.
            self.counters['stale_control_reports'] += 1
            return
        if message.state in ('GUARDING', 'GUARD_BRAKING', 'GUARD_BLOCKED') and self.motion_mode == 'NAVIGATE':
            if (not self.guard_on_arrival or self.route_progress is None
                    or not self.route_progress.terminal
                    or self._terminal_reference_sequence is None
                    or not self._terminal_reference_sequence <= message.reference_sequence <= self.sequence
                    or message.task_finished or message.clock_reset):
                self.counters['stale_control_reports'] += 1
                return
            self.route_progress.finish()
            self._snapshot_route_progress()
            self.route_progress = None
            self.motion_mode, self.mode, self.reason = 'HOLD', 'GUARDING', ''
            self.version += 1
            self.worker.invalidate()
            self.trajectory = None
            self.route = np.empty((0, 3))
            self._publish_global_path(self.get_clock().now().to_msg())
            self.trajectory_pub.publish(make_path(self.route, self.frame, self.get_clock().now().to_msg()))
            self._publish_reference()
        if self.guard_on_arrival and message.task_finished:
            # An older/non-guard-capable controller must not silently turn a
            # persistent task into an ordinary completed voyage.
            self.counters['unexpected_guard_completion'] += 1
            return
        if message.task_finished and self.route_progress is not None:
            if (not self.route_progress.terminal or self._terminal_reference_sequence is None
                    or message.reference_sequence < self._terminal_reference_sequence
                    or message.reference_sequence > self.sequence):
                self.counters['stale_control_reports'] += 1
                return
        self.control_report, self.control_report_wall = message, time.monotonic()
        if message.clock_reset:
            self._reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
        elif message.task_finished and self.motion_mode == 'BRAKE':
            self._reset_task('STOPPED', 'Measured low speed and yaw rate confirmed; thrust is zero')
        elif message.task_finished and self.goal is not None:
            if self.route_progress is not None:
                self.route_progress.finish()
            self._reset_task('GOAL_REACHED')
        elif (message.revoke_reference and message.reference_sequence == self.sequence
              and self.motion_mode == 'NAVIGATE'):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = message.state, message.reason
            self._publish_global_path(self.get_clock().now().to_msg())
            self._publish_reference()

    # 每个取消请求分别核对任务号、最低参考序号和三路零推力，不能用后续取消替代先前确认。
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

    # 稳态定时回调处理时间倒退、参数、里程计、规划结果、参考重发与取消超时。
    def _tick(self):
        now = self.now_seconds()
        if now < self.last_clock - 1e-6:
            self._reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
            self.counters['clock_resets'] += 1
        self.last_clock = now
        self._apply_parameters()
        self._consume_odom()
        self._update_route_progress()
        self._consume_plan()
        self._publish_reference(changed=False)
        self._resolve_cancellations()
        if time.perf_counter() - self.last_status_wall >= 1.:
            self._publish_status()

    # 汇总自身和新鲜控制报告的统计信息，发布 JSON 导航状态便于运行观察。
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
            if self.goal is not None or self.motion_mode == 'BRAKE':
                if report.state != 'WAITING_REFERENCE' or self.trajectory is not None or not reason:
                    mode, reason = report.state, report.reason
        if (self.goal is not None or self.motion_mode == 'BRAKE') and not self.peers_ready('control', 'perception'):
            mode, reason = 'WAITING_MODULE', 'Waiting for onboard modules'
        self.status_pub.publish(String(data=json.dumps(dict(
            state=mode, reason=reason, version=self.version, stamp=self.now_seconds(),
            request_id=self.request_id, task_id=self.accepted_task_id,
            current_task_id=self.task_id,
            motion_mode=self.motion_mode, guard_on_arrival=self.guard_on_arrival,
            guard_heading_enabled=self.guard_heading_enabled, guard_heading=self.guard_heading,
            desired_speed_mps=self.desired_speed_mps,
            **self.route_summary,
            controller=self.controller_name, goal=self.goal, metrics=metrics, counters=counters,
            process=process, modules={'decision': self.session, 'control': self.peers['control'].session,
                                      'perception': self.peers['perception'].session,
                                      'control_process': control_process}))))

    # 退出时发停用参考、结束未完成取消请求，并关闭规划执行器。
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
