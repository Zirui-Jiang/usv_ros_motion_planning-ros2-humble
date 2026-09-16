# 普通导航控制 ROS 节点：接收决策参考和感知快照，执行选定控制器，通过唯一输出适配器发布桨命令。
"""Ordinary navigation control process and the sole propulsion output adapter.

Planning and reference collision checks belong to the decision process. This
node retains the original per-tick measured-hull check and PID/LQR lifecycle.
"""
import json
import math
import time

from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from usv_navigation_interfaces.msg import NavigationReference, ControlReport

from common.models import AlgorithmStatus, ControlCommand
from controller.controller_factory import create_controller
from usv_navigation.configuration import C, DEFAULTS, HOT_PARAMETERS, resolve_parameters, section, validate_update
from usv_navigation.protocol import trajectory_from_message
from usv_navigation.state_rules import readiness
from usv_navigation.ros_support import NavigationInputs
from usv_navigation.node_runner import run_node


# A repeated /clock sample is normal when Gazebo runs below real time. Only a
# sustained wall-time stall counts as a pause; checked on every steady timer tick.
CLOCK_STALL_TIMEOUT = 0.5


# 会话号区分进程重启，任务号区分目标，序号拒绝旧参考；控制循环独立于规划进程运行。
class ControlNode(NavigationInputs):
    def __init__(self):
        super().__init__('usv_control', role='control')
        self.controller_name = self.config[C + 'controller_name']
        self.controller = create_controller(self.controller_name, self.model,
                            section(self.config, C + self.controller_name + '_controller.'))
        self.task_id = 0
        self.planner_session = ''
        self.reference_sequence = 0
        self.revision = 0
        self.goal = self.trajectory = None
        self.mode, self.reason = 'IDLE', ''
        self.reference_state, self.reference_reason = 'IDLE', ''
        self.revoke_reference = self.task_finished = self.clock_reset = False
        self._closed_tasks = {}
        self._pending_config = None
        self._reference_error = ''
        self.last_clock = self.last_control = self.now_seconds()
        self.last_clock_progress_wall = time.monotonic()
        self.last_published_command = ControlCommand()
        self.thrust_pubs = [self.create_publisher(Float32, 'thrusters/' + side + '_thrust_cmd', 1)
                           for side in ('left', 'right', 'lateral')]
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.reference_subscription = self.create_subscription(
            NavigationReference, 'navigation/reference', self._on_reference, reliable)
        self.report_pub = self.create_publisher(ControlReport, 'navigation/control_report', reliable)
        self.control_timer = self.create_timer(1. / self.config[C + 'control_frequency'],
                                              self._control, clock=self.steady_clock)

    # 登记已关闭任务的最大编号、重置算法并立即发零，阻止同一任务的迟到活动参考重新启动。
    def _close_task(self, mode, reason='', clear_inputs=False):
        if self.planner_session:
            self._closed_tasks[self.planner_session] = max(
                self.task_id, self._closed_tasks.get(self.planner_session, -1))
        self.goal = self.trajectory = None
        self.controller.reset()
        self.mode, self.reason = mode, reason
        self.reference_state, self.reference_reason = mode, reason
        self._reference_error = ''
        self._pending_config = None
        if clear_inputs:
            self._clear_inputs()
        self._publish_command(ControlCommand())

    # 决策或感知进程重启会关闭当前任务，要求上层重新下达目标。
    def _on_peer_restart(self, role):
        if role not in ('decision', 'perception'):
            return
        self._close_task('MODULE_RESTART', role + ' restarted; a new goal is required')
        self.revoke_reference = True
        self.counters['module_restarts'] += 1
        self._publish_report()

    # 参考必须携带完整有效参数快照；只允许 HOT_PARAMETERS 中的原有热参数与本地不同。
    def _reference_parameters(self, message):
        supplied = json.loads(message.parameters_json)
        if not isinstance(supplied, dict) or set(supplied) != set(DEFAULTS):
            raise ValueError('A reference must include the complete effective parameter snapshot')
        candidate = resolve_parameters(supplied)
        changes = {key: value for key, value in candidate.items() if value != self.config[key]}
        # YAML and immutable algorithm/model settings must match both modules.
        # Only the original hot parameters may differ after startup.
        return validate_update(self.config, changes)

    # 先核验三方会话与任务时序，再校验参数、目标与轨迹；无效参考立即停止输出。
    def _on_reference(self, message):
        decision, perception = self.peers.get('decision'), self.peers.get('perception')
        if (message.control_session != self.session or decision is None or perception is None
                or not decision.session or not perception.session
                or message.planner_session != decision.session
                or message.perception_session != perception.session):
            self.counters['wrong_session_references'] += 1
            return
        same_planner = message.planner_session == self.planner_session
        if same_planner and (message.sequence <= self.reference_sequence or message.task_id < self.task_id):
            self.counters['stale_references'] += 1
            return
        if message.active and message.task_id <= self._closed_tasks.get(message.planner_session, -1):
            self.counters['closed_task_references'] += 1
            return
        new_task = not same_planner or message.task_id != self.task_id
        if new_task:
            self.controller.reset()
            self.goal = self.trajectory = None
            self.task_finished = self.clock_reset = False
            self._publish_command(ControlCommand())
        self.planner_session, self.task_id = message.planner_session, message.task_id
        self.reference_sequence, self.revision = message.sequence, message.revision
        self.reference_state, self.reference_reason = message.state, message.reason
        self.revoke_reference = False
        self._reference_error = ''
        try:
            self._pending_config = self._reference_parameters(message)
        except (ValueError, TypeError, KeyError) as error:
            self.goal = self.trajectory = None
            self._pending_config = None
            self._reference_error = 'CONFIG_MISMATCH'
            self.mode, self.reason = self._reference_error, str(error)
            self.counters['invalid_reference_config'] += 1
            self.controller.on_control_hold()
            self._publish_command(ControlCommand())
            self._publish_report()
            return
        # 非活动快照用于取消或结束任务，也可携带已接受的在线参数变更。
        if not message.active:
            pending_config = self._pending_config
            self._close_task(message.state or 'IDLE', message.reason)
            # An inactive reference also transports accepted online parameters.
            self._pending_config = pending_config
            self._publish_report()
            return
        try:
            goal = (float(message.goal.x), float(message.goal.y), float(message.goal.theta))
            if not all(math.isfinite(value) for value in goal):
                raise ValueError('Navigation goal is nonfinite')
            trajectory = trajectory_from_message(message)
            if trajectory is not None and trajectory.frame_id != self.frame:
                raise ValueError('Navigation reference frame differs from control map frame')
            self.goal, self.trajectory = goal, trajectory
            self.mode, self.reason = message.state or 'WAITING_REFERENCE', message.reason
            if trajectory is None:
                self.controller.on_control_hold()
                self._publish_command(ControlCommand())
        except (ValueError, TypeError, AttributeError) as error:
            self.goal = self.trajectory = None
            self._reference_error = 'INVALID_REFERENCE'
            self.mode, self.reason = self._reference_error, str(error)
            self.counters['invalid_references'] += 1
            self.controller.on_control_hold()
            self._publish_command(ControlCommand())
        self._publish_report()

    # 在控制拍边界原子应用待处理参数，PID 在线增益更新保留积分状态。
    def _apply_parameters(self):
        if self._pending_config is not None:
            candidate = self._pending_config
            updates = [Parameter(key, value=candidate[key]) for key in HOT_PARAMETERS
                       if self.get_parameter(key).value != candidate[key]]
            if updates:
                result = self.set_parameters_atomically(updates)
                if not result.successful:
                    raise ValueError(result.reason)
            self._pending_config = None
        changes = {key: self.get_parameter(key).value for key in HOT_PARAMETERS
                   if self.get_parameter(key).value != self.config[key]}
        if not changes:
            return
        candidate = validate_update(self.config, changes)
        if self.controller_name == 'pid':
            self.controller.update_config(section(candidate, C + 'pid_controller.'))
        self.config = candidate
        self.counters['parameter_updates'] += 1

    # 按左桨、右桨、侧推顺序发布 Float32，同时保存实际发送值供报告使用。
    def _publish_command(self, command):
        for publisher, value in zip(self.thrust_pubs, (command.left, command.right, command.lateral)):
            publisher.publish(Float32(data=float(value)))
        self.last_published_command = command

    # 每拍默认零命令，只有数据新鲜、模块在线、船体无碰撞且 dt>0 时才运行跟踪算法。
    def _control(self):
        begun = time.perf_counter()
        now = self.now_seconds()
        wall_now = time.monotonic()
        if now != self.last_clock:
            self.last_clock_progress_wall = wall_now
        # 仿真时间倒退时关闭任务并清除旧输入，避免跨时间轴继续使用原轨迹。
        if now < self.last_clock - 1e-6:
            self._close_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
            self.clock_reset = True
            self.counters['clock_resets'] += 1
        dt = now - self.last_control
        self.last_clock = self.last_control = now
        command = ControlCommand()
        tracking = False
        try:
            self._apply_parameters()
            self._consume_odom()
            if self._reference_error:
                self.mode = self._reference_error
            elif not self.peers_ready('decision', 'perception'):
                self.mode, self.reason = 'WAITING_MODULES', 'Waiting for fresh decision and perception heartbeats'
            elif self.goal is not None:
                ready = readiness(now, self.state, self.grid, self.trajectory, self.config)
                # 即使参考尚未到来，也检查最新实测船体；碰撞撤销参考但不将任务标为完成。
                if ready in ('NAVIGATING', 'WAITING_REFERENCE') and not self._current_footprint_clear():
                    self.trajectory = None
                    self.revoke_reference = True
                    ready = 'CURRENT_FOOTPRINT_BLOCKED'
                    self.reason = 'Current vessel footprint intersects lethal or unknown map cells'
                    self.counters['current_footprint_blocks'] += 1
                if ready == 'NAVIGATING' and dt > 0:
                    self.mode, self.reason = ready, ''
                    command = self.controller.compute(self.state, self.trajectory, dt)
                    if command.status == AlgorithmStatus.GOAL_REACHED:
                        self.task_finished = True
                        self._close_task('GOAL_REACHED')
                        command = ControlCommand()
                    elif command.status == AlgorithmStatus.SETTLING:
                        self.mode, self.reason = 'SETTLING', command.message
                    elif command.status != AlgorithmStatus.SUCCESS or not command.is_valid():
                        self.mode, self.reason = 'CONTROL_ERROR', command.message
                        self.counters['control_result_' + command.status.value] += 1
                        command = ControlCommand()
                    else:
                        tracking = True
                elif ready == 'NAVIGATING' and dt == 0:
                    if wall_now - self.last_clock_progress_wall < CLOCK_STALL_TIMEOUT:
                        # 等待下一条 /clock：保留已发布命令，不重复计算/积分。
                        # 就绪、碰撞和对端检查仍在前面执行，取消/无效参考也会立即归零。
                        self.mode, self.reason = 'NAVIGATING', ''
                        command = self.last_published_command
                        tracking = True
                    else:
                        self.mode, self.reason = 'SIMULATION_PAUSED', 'Simulation clock stalled for at least 0.5 wall seconds'
                else:
                    if ready == 'WAITING_REFERENCE' and self.reference_state not in ('NAVIGATING', 'SETTLING'):
                        self.mode, self.reason = self.reference_state or ready, self.reference_reason
                    else:
                        self.mode = ready
            else:
                self.mode, self.reason = self.reference_state, self.reference_reason
        except Exception as error:
            self.mode, self.reason = 'CONTROL_ERROR', str(error)
            self.counters['control_errors'] += 1
            command = ControlCommand()
        if not tracking:
            # 等待/暂停/失败期间不携带 ADRC 的旧积分和滤波历史。
            # 基类为空操作，原 PID/LQR 的暂停行为保持。
            self.controller.on_control_hold()
        elapsed = (time.perf_counter() - begun) * 1000
        if elapsed >= 1000. / self.config[C + 'control_frequency']:
            self.counters['control_overruns'] += 1
        self.timings['control_ms'].append(elapsed)
        self._publish_command(command)
        self._publish_report()

    # 回报实际推力、任务/参考身份与撤销标志，决策端据此确认取消或真实到达。
    def _publish_report(self):
        metrics, counters, process = self.statistics()
        command = self.last_published_command
        message = ControlReport()
        message.header.stamp = self.get_clock().now().to_msg()
        message.control_session, message.planner_session = self.session, self.planner_session
        message.task_id, message.reference_sequence = self.task_id, self.reference_sequence
        message.state, message.reason = self.mode, self.reason
        message.revoke_reference = self.revoke_reference
        message.task_finished, message.clock_reset = self.task_finished, self.clock_reset
        message.left, message.right, message.lateral = command.left, command.right, command.lateral
        message.metrics_json = json.dumps(metrics)
        message.counters_json = json.dumps(counters)
        message.process_json = json.dumps(process)
        self.report_pub.publish(message)

    # ROS 上下文仍可用时先发布零推力，再销毁节点。
    def destroy_node(self):
        if self.context.ok():
            self._publish_command(ControlCommand())
        return super().destroy_node()


def main(args=None):
    run_node(ControlNode, args)


if __name__ == '__main__':
    main()
