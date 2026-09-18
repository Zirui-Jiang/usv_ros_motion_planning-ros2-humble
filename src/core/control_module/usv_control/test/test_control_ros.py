# 使用真实 ROS 消息和人工传感器输入验证独立控制节点的会话、停车、参数与时钟边界。
"""Control-side ROS contracts using real messages, including ADRC lifecycle."""
import importlib
import json
import time
import uuid
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32


class ControlHarness:
    def __init__(self, simulated=True, extra_args=()):
        try:
            control_module = importlib.import_module('usv_control.control')
        except ModuleNotFoundError as error:
            pytest.fail('Independent control node is not available: ' + str(error))
        from usv_navigation_interfaces.msg import NavigationReference, ModuleHeartbeat, ControlReport, TrajectoryPoint
        self.Reference, self.Heartbeat, self.Point = NavigationReference, ModuleHeartbeat, TrajectoryPoint
        self.namespace = 'control_' + uuid.uuid4().hex[:10]
        rclpy.init(args=['--ros-args', '-r', '__ns:=/' + self.namespace,
                        '-p', 'use_sim_time:=' + str(simulated).lower(), *extra_args])
        self.control = control_module.ControlNode()
        self.driver = Node('test_driver')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.control)
        self.executor.add_node(self.driver)
        self.reference_pub = self.driver.create_publisher(NavigationReference, 'navigation/reference', 10)
        self.heartbeat_pub = self.driver.create_publisher(ModuleHeartbeat, 'navigation/heartbeat', 10)
        self.odom_pub = self.driver.create_publisher(Odometry, 'odom', 1)
        self.map_pub = self.driver.create_publisher(Costmap, 'costmap_raw',
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.clock_pub = self.driver.create_publisher(Clock, '/clock', 10)
        self.left, self.right, self.reports = [], [], []
        self.subscriptions = [
            self.driver.create_subscription(Float32, 'thrusters/left_thrust_cmd', self.left.append, 10),
            self.driver.create_subscription(Float32, 'thrusters/right_thrust_cmd', self.right.append, 10),
            self.driver.create_subscription(ControlReport, 'navigation/control_report', self.reports.append, 10),
        ]
        self.sessions = {'decision': 'planner-one', 'perception': 'perception-one'}
        self.heartbeat_sequence = 0
        self.sequence = 0
        self.simulated, self.simulated_time = simulated, 10.0
        self.x, self.y, self.speed = 0., 0., 0.
        self.yaw_rate = 0.
        self.block = False
        self.blocked_cells = []
        self.active_reference = None

    def close(self):
        self.executor.shutdown()
        self.control.destroy_node()
        self.driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def spin(self, duration=.035):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.002)

    def step(self, odom=True, grid=True, clock=True, heartbeat=True, reference=False):
        if self.simulated and clock:
            self.simulated_time += .05
            msg = Clock()
            msg.clock.sec = int(self.simulated_time)
            msg.clock.nanosec = round((self.simulated_time % 1) * 1e9)
            self.clock_pub.publish(msg)
            self.spin(.005)
        stamp = self.control.get_clock().now().to_msg()
        if heartbeat:
            self.heartbeat_sequence += 1
            for role, session in self.sessions.items():
                msg = self.Heartbeat(module=role, session=session, sequence=self.heartbeat_sequence, ready=True)
                self.heartbeat_pub.publish(msg)
        if odom:
            msg = Odometry()
            msg.header.frame_id = self.namespace + '/map'
            msg.header.stamp = stamp
            msg.child_frame_id = self.namespace + '/base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
            msg.pose.pose.orientation.w = 1.
            msg.twist.twist.linear.x = self.speed
            msg.twist.twist.angular.z = self.yaw_rate
            self.odom_pub.publish(msg)
        if grid:
            msg = Costmap()
            msg.header.frame_id = self.namespace + '/map'
            msg.header.stamp = stamp
            msg.metadata.size_x = msg.metadata.size_y = 80
            msg.metadata.resolution = .25
            msg.metadata.origin.position.x = msg.metadata.origin.position.y = -10.
            msg.metadata.origin.orientation.w = 1.
            msg.data = [0] * 6400
            if self.block:
                msg.data[44 * 80 + 44] = 254
            for column, row in self.blocked_cells:
                msg.data[row * 80 + column] = 254
            self.map_pub.publish(msg)
        if isinstance(reference, str):
            self.send_reference(motion_reference(self, reference, task=self.control.task_id or 1))
        elif reference:
            self.send_reference()
        self.spin()

    def until(self, predicate, timeout=4., **kwargs):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.step(**kwargs)
        assert predicate(), (self.control.mode, self.control.reason, dict(self.control.counters))

    def reference(self, task=1, active=True, parameters=None, goal=(5., 0.), points=True):
        self.sequence += 1
        msg = self.Reference()
        msg.header.stamp = self.control.get_clock().now().to_msg()
        msg.header.frame_id = self.namespace + '/map'
        msg.planner_session = self.sessions['decision']
        msg.control_session = self.control.session
        msg.perception_session = self.sessions['perception']
        msg.task_id, msg.revision, msg.sequence = task, 1, self.sequence
        msg.active = active
        msg.goal.x, msg.goal.y = goal
        msg.state = 'WAITING_REFERENCE' if active else 'CANCELLED'
        msg.parameters_json = json.dumps(self.control.config if parameters is None else parameters)
        msg.trajectory_stamp = self.control.now_seconds()
        msg.trajectory_frame = self.namespace + '/map'
        if points:
            msg.points = [self.Point(time=0., s=0., x=0., y=0., theta=0., kappa=0., velocity=.8, acceleration=0.),
                          self.Point(time=10., s=8., x=8., y=0., theta=0., kappa=0., velocity=.8, acceleration=0.)]
        return msg

    def send_reference(self, message=None):
        message = self.reference() if message is None else message
        self.active_reference = message
        self.reference_pub.publish(message)
        return message

    def ready(self):
        self.until(lambda: self.control.state is not None and self.control.grid is not None
                   and self.control.peers_ready('decision', 'perception'))


@contextmanager
def control_harness(**kwargs):
    harness = None
    try:
        harness = ControlHarness(**kwargs)
        yield harness
    finally:
        if harness is not None:
            harness.close()
        elif rclpy.ok():
            rclpy.shutdown()


# 验证跟踪后取消归零，旧序号或错误控制会话不能恢复任务。
@pytest.mark.parametrize("algorithm", ["pid", "adrc"])
def test_control_tracks_then_cancels_and_rejects_old_identity_or_sequence(algorithm):
    with control_harness(extra_args=('-p', 'usv_simulation.controller.controller_name:=' + algorithm)) as h:
        h.ready()
        original = h.send_reference()
        h.until(lambda: any(x.data > 0 for x in h.left))
        assert h.control.trajectory.stamp == original.trajectory_stamp
        stop = h.reference(task=2, active=False, points=False)
        h.send_reference(stop)
        h.until(lambda: h.reports and h.reports[-1].reference_sequence == stop.sequence
                and h.reports[-1].state == 'CANCELLED')
        h.left.clear()
        h.send_reference(original)
        wrong = h.reference(task=3)
        wrong.control_session = 'retired-control'
        h.send_reference(wrong)
        for _ in range(8):
            h.step()
        assert h.left and all(x.data == 0. for x in h.left)
        assert h.reports[-1].task_id == 2


# Station keeping must consume its own read-only ADRC parameter section,
# independent of the ordinary navigation controller selection.
def test_station_adrc_parameters_are_injected_into_holding_controller():
    with control_harness(extra_args=(
            '-p', 'usv_simulation.controller.station_adrc_controller.k_p_speed:=17.0',
            '-p', 'usv_simulation.controller.station_adrc_controller.command_limit:=0.25')) as h:
        assert h.control.station_controller.config['k_p_speed'] == pytest.approx(17.0)
        assert h.control.station_controller.config['command_limit'] == pytest.approx(.25)


# 热参数在控制拍应用，PID 积分历史保持不变。
def test_control_applies_hot_parameters_on_tick_without_resetting_pid():
    with control_harness() as h:
        from usv_navigation.configuration import C, T
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.control.control_timer.cancel()
        h.control.controller.integral_speed = .123
        cfg = dict(h.control.config)
        cfg[C + 'pid_controller.k_p_speed'] = 4.
        cfg[T + 'los.lookahead_distance'] = 2.
        updated = h.reference(parameters=cfg, points=False)
        h.send_reference(updated)
        h.until(lambda: h.control.reference_sequence == updated.sequence)
        assert h.control.controller.config['k_p_speed'] == 2.
        assert h.control.controller.integral_speed == .123
        h.control.control_timer.reset()
        h.until(lambda: h.control.controller.config['k_p_speed'] == 4.)
        assert h.control.controller.integral_speed == .123
        assert h.control.trajectory is None


# 当前船体碰撞撤销参考但保留任务，安全后新参考可恢复。
def test_current_footprint_block_revokes_reference_without_finishing_task():
    with control_harness() as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.y, h.block = .6, True
        h.until(lambda: h.reports and h.reports[-1].revoke_reference)
        report = h.reports[-1]
        assert report.state == 'CURRENT_FOOTPRINT_BLOCKED'
        assert not report.task_finished and h.control.goal is not None
        assert report.left == report.right == 0.
        h.y, h.block = 0., False
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')


# 完成任务与对端重启后旧任务号不能重新激活。
def test_finished_task_and_peer_restart_require_new_task():
    with control_harness() as h:
        h.ready()
        completed = h.reference(goal=(.1, 0.))
        h.send_reference(completed)
        h.until(lambda: h.reports and h.reports[-1].task_finished)
        h.left.clear()
        h.send_reference(h.reference(task=1))
        for _ in range(6):
            h.step()
        assert h.left and all(x.data == 0. for x in h.left)
        h.send_reference(h.reference(task=2))
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.sessions['perception'] = 'perception-two'
        h.until(lambda: h.control.goal is None)
        h.left.clear()
        h.send_reference(h.reference(task=2))
        for _ in range(6):
            h.step()
        assert h.left and all(x.data == 0. for x in h.left)
        h.send_reference(h.reference(task=3))
        h.until(lambda: h.control.mode == 'NAVIGATING')


# 临时状态过期和仿真暂停归零，时钟倒退清理任务。
@pytest.mark.parametrize("algorithm", ["pid", "adrc"])
def test_control_pauses_clock_and_recovers_temporary_state_loss(algorithm):
    with control_harness(simulated=True, extra_args=('-p', 'usv_simulation.controller.controller_name:=' + algorithm)) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.until(lambda: h.control.mode == 'WAITING_STATE', odom=False, reference=True)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        if algorithm == 'adrc':
            assert h.control.controller.speed_loop.integral_speed == 0.
            assert h.control.controller.heading_loop.heading_filter.position_history == []
        h.until(lambda: h.control.mode == 'NAVIGATING', reference=True)
        h.until(lambda: h.control.mode == 'SIMULATION_PAUSED', clock=False)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.until(lambda: h.control.mode == 'NAVIGATING', reference=True)
        h.simulated_time = 1.
        h.until(lambda: h.reports and h.reports[-1].clock_reset, odom=False, grid=False)
        assert h.control.goal is None and h.control.trajectory is None


@pytest.mark.parametrize('algorithm', ['pid', 'lqr', 'adrc'])
def test_control_waits_for_clock_sample_without_cutting_thrust_or_reintegrating(algorithm):
    """A slow Gazebo clock may stay unchanged across several wall timer ticks."""
    with control_harness(extra_args=('-p', 'usv_simulation.controller.controller_name:=' + algorithm)) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.control.control_timer.cancel()
        for _ in range(5):
            h.step()
            h.control._control()
            previous = h.control.last_published_command
            assert previous.left > 0 and previous.right > 0
            if algorithm == 'adrc':
                core = h.control.controller
                integral = core.speed_loop.integral_speed
                history = list(core.heading_loop.heading_filter.position_history)
            # Same ROS time, no new observation: keep output, never run the core twice.
            h.control._control()
            assert h.control.mode == 'NAVIGATING'
            assert h.control.last_published_command == previous
            if algorithm == 'adrc':
                assert core.speed_loop.integral_speed == integral
                assert core.heading_loop.heading_filter.position_history == history
        # A sustained clock stop still zeros thrust and resets ADRC history.
        h.control.last_clock_progress_wall -= .6
        h.control._control()
        assert h.control.mode == 'SIMULATION_PAUSED'
        assert h.control.last_published_command.left == h.control.last_published_command.right == 0.
        if algorithm == 'adrc':
            assert core.speed_loop.integral_speed == 0.
            assert core.heading_loop.heading_filter.position_history == []
        h.step()
        h.control._control()
        assert h.control.mode == 'NAVIGATING'
        assert h.control.last_published_command.left > 0


# 对端发现前的空会话参考不能在之后自动生效。
def test_reference_before_peer_discovery_cannot_activate_when_peers_appear():
    with control_harness() as h:
        h.spin(.1)
        message = h.reference()
        message.planner_session = message.perception_session = ''
        h.send_reference(message)
        h.spin(.1)
        h.ready()
        h.left.clear()
        for _ in range(6):
            h.step()
        assert h.left and all(x.data == 0. for x in h.left)
        assert h.control.goal is None


# 不可热改参数不一致导致停止，不能继续旧参考。
def test_immutable_configuration_mismatch_stops_instead_of_using_old_reference():
    with control_harness() as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        cfg = dict(h.control.config)
        cfg['navigation.footprint_length'] = 3.
        h.send_reference(h.reference(parameters=cfg))
        h.until(lambda: h.control.mode == 'CONFIG_MISMATCH')
        h.left.clear()
        for _ in range(6):
            h.step()
        assert h.left and all(x.data == 0. for x in h.left)
        assert h.control.config['navigation.footprint_length'] == 2.2


# 通过 ROS 初始参数选择 LQR 并保留指定预瞄值。
@pytest.mark.parametrize("algorithm", ["lqr", "adrc"])
def test_initial_ros_parameters_select_controller(algorithm):
    from usv_navigation.configuration import C
    with control_harness(extra_args=('-p', C + 'controller_name:=' + algorithm,
                                     '-p', C + algorithm + '_controller.lookahead_time:=1.2')) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: any(x.data > 0. for x in h.left))
        assert h.control.controller_name == algorithm
        assert h.control.controller.config['lookahead_time'] == 1.2


def test_adrc_duplicate_odometry_does_not_advance_core_and_new_task_resets():
    with control_harness(extra_args=('-p', 'usv_simulation.controller.controller_name:=adrc')) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.control.control_timer.cancel()
        algorithm = h.control.controller
        # 先消费当前输入并拍摄历史；随后仅推时钟，故同一 odom 会被多拍读取。
        h.step()
        h.control._control()
        history = list(algorithm.heading_loop.heading_filter.position_history)
        integral = algorithm.speed_loop.integral_speed
        for _ in range(3):
            h.step(odom=False)
            h.control._control()
        assert algorithm.heading_loop.heading_filter.position_history == history
        assert algorithm.speed_loop.integral_speed == integral
        rolling = h.reference(task=1)
        h.send_reference(rolling)
        h.until(lambda: h.control.reference_sequence == rolling.sequence)
        assert algorithm.heading_loop.heading_filter.position_history == history
        fresh = h.reference(task=2)
        h.send_reference(fresh)
        h.until(lambda: h.control.reference_sequence == fresh.sequence)
        assert algorithm.heading_loop.heading_filter.position_history == []
        assert algorithm.speed_loop.integral_speed == 0.


@pytest.mark.parametrize('interruption', ['waiting', 'invalid', 'config'])
def test_adrc_brief_reference_interruption_resets_before_next_tick(interruption):
    with control_harness(extra_args=('-p', 'usv_simulation.controller.controller_name:=adrc')) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.control.control_timer.cancel()
        algorithm = h.control.controller
        assert algorithm.heading_loop.heading_filter.position_history
        message = h.reference(points=interruption != 'waiting')
        if interruption == 'invalid':
            message.goal.x = float('nan')
        elif interruption == 'config':
            candidate = dict(h.control.config)
            candidate['navigation.footprint_length'] = 3.
            message.parameters_json = json.dumps(candidate)
        # 两个参考回调之间没有控制 timer：短暂停推也必须立即清历史。
        h.control._on_reference(message)
        assert algorithm.heading_loop.heading_filter.position_history == []
        assert algorithm.speed_loop.integral_speed == 0.
        assert h.control.last_published_command.left == h.control.last_published_command.right == 0.
        h.control._on_reference(h.reference())
        assert algorithm.last_sample_stamp is None


def motion_reference(h, mode, task=1, goal=(0., 0.), heading=None):
    message = h.reference(task=task, goal=goal, points=False)
    message.motion_mode = mode
    message.guard_heading_enabled = heading is not None
    message.guard_heading = 0. if heading is None else heading
    return message


@pytest.mark.parametrize('speed', [.7, -.7])
def test_active_brake_without_trajectory_opposes_motion_and_confirms_stop(speed):
    with control_harness() as h:
        h.speed = speed
        h.ready()
        h.send_reference(motion_reference(h, 'BRAKE'))
        h.until(lambda: h.reports and h.reports[-1].state == 'BRAKING'
                and speed * (h.reports[-1].left + h.reports[-1].right) < 0)
        assert speed * (h.reports[-1].left + h.reports[-1].right) < 0
        h.speed = 0.
        h.yaw_rate = .03
        for _ in range(24):
            h.send_reference(motion_reference(h, 'BRAKE'))
            h.step()
        assert not h.reports[-1].task_finished
        h.yaw_rate = 0.
        for _ in range(8):
            h.send_reference(motion_reference(h, 'BRAKE'))
            h.step()
        assert not h.reports[-1].task_finished
        for _ in range(24):
            h.send_reference(motion_reference(h, 'BRAKE'))
            h.step()
        assert h.reports[-1].state == 'STOPPED' and h.reports[-1].task_finished
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.send_reference(h.reference())
        h.step()
        assert h.reports[-1].state == 'STOPPED'


def test_brake_missing_map_never_certifies_stopped_and_reference_expiry_stops_output():
    with control_harness() as h:
        h.until(lambda: h.control.state is not None and h.control.peers_ready('decision', 'perception'), grid=False)
        for _ in range(25):
            h.send_reference(motion_reference(h, 'BRAKE'))
            h.step(grid=False)
        assert h.reports[-1].state == 'WAITING_MAP'
        assert not h.reports[-1].task_finished
        h.speed = .7
        h.send_reference(motion_reference(h, 'BRAKE'))
        h.until(lambda: h.reports and h.reports[-1].state == 'BRAKING' and h.reports[-1].left < 0)
        h.until(lambda: h.reports and h.reports[-1].state == 'WAITING_REFERENCE')
        assert h.reports[-1].left == h.reports[-1].right == 0.


def test_terminal_guard_latches_target_until_new_task_and_does_not_finish():
    with control_harness() as h:
        h.ready()
        terminal = h.reference(goal=(0., 0.))
        terminal.guard_on_arrival = True
        terminal.guard_heading_enabled = True
        terminal.guard_heading = .4
        h.send_reference(terminal)
        h.until(lambda: h.reports and h.reports[-1].state == 'GUARDING')
        assert not h.reports[-1].task_finished
        # A delayed navigation snapshot cannot move the locked centre or yaw.
        h.send_reference(h.reference(goal=(8., 8.)))
        h.step()
        assert h.control.goal[:2] == (0., 0.)
        assert h.control._hold_target == (0., 0., .4)
        h.send_reference(motion_reference(h, 'HOLD', goal=(0., 0.), heading=.4))
        h.until(lambda: h.reports and h.reports[-1].right > h.reports[-1].left)
        cancel = h.reference(task=2, active=False, points=False)
        h.send_reference(cancel)
        h.until(lambda: h.reports and h.reports[-1].state == 'CANCELLED')
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.send_reference(h.reference(task=3))
        h.until(lambda: h.reports and h.reports[-1].state == 'NAVIGATING')


def test_intermediate_pass_through_does_not_trigger_guard_and_hold_checks_obstacles():
    with control_harness() as h:
        h.ready()
        intermediate = h.reference(goal=(0., 0.))
        intermediate.pass_through = True
        intermediate.guard_on_arrival = True
        h.send_reference(intermediate)
        h.until(lambda: h.reports and h.reports[-1].state == 'NAVIGATING')
        assert not h.reports[-1].task_finished
        h.send_reference(motion_reference(h, 'HOLD', task=2))
        h.until(lambda: h.reports and h.reports[-1].state == 'GUARDING')
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.y, h.block = .6, True
        h.send_reference(motion_reference(h, 'HOLD', task=2))
        h.until(lambda: h.reports and h.reports[-1].state == 'CURRENT_FOOTPRINT_BLOCKED')
        assert h.reports[-1].left == h.reports[-1].right == 0.
        assert not h.reports[-1].task_finished


@pytest.mark.parametrize('mode', ['BRAKE', 'HOLD'])
def test_motion_modes_keep_state_module_clock_and_frame_safety_gates(mode):
    with control_harness() as h:
        h.ready()
        h.speed = .2
        h.send_reference(motion_reference(h, mode))
        expected = 'BRAKING' if mode == 'BRAKE' else 'GUARDING'
        h.until(lambda: h.reports and h.reports[-1].state == expected, reference=mode)
        h.until(lambda: h.reports and h.reports[-1].state == 'WAITING_STATE', odom=False, reference=mode)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        assert not h.reports[-1].task_finished
        h.until(lambda: h.reports and h.reports[-1].state == expected, reference=mode)
        h.until(lambda: h.reports and h.reports[-1].state == 'SIMULATION_PAUSED', clock=False, reference=mode)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.until(lambda: h.reports and h.reports[-1].state == expected, reference=mode)
        invalid = motion_reference(h, mode)
        invalid.header.frame_id = 'other/map'
        h.send_reference(invalid)
        h.until(lambda: h.reports and h.reports[-1].state == 'INVALID_REFERENCE')
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.send_reference(motion_reference(h, mode))
        h.until(lambda: h.reports and h.reports[-1].state == expected, reference=mode)
        h.sessions['perception'] = 'perception-restarted'
        h.until(lambda: h.reports and h.reports[-1].state == 'MODULE_RESTART')
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.send_reference(motion_reference(h, mode))
        h.step()
        assert h.reports[-1].state == 'MODULE_RESTART'


def test_guard_handover_is_reported_even_when_first_holding_command_is_blocked():
    with control_harness() as h:
        h.speed = .2
        # x=1.25 m is outside the measured hull, but inside its stopping sweep.
        h.blocked_cells = [(45, 40)]
        h.ready()
        terminal = h.reference(goal=(1.4, 0.))
        terminal.guard_on_arrival = True
        h.send_reference(terminal)
        h.until(lambda: any(report.state == 'GUARD_BLOCKED' for report in h.reports))
        assert any(report.state == 'GUARDING' and not report.task_finished for report in h.reports)
        blocked = next(report for report in h.reports if report.state == 'GUARD_BLOCKED')
        assert blocked.left == blocked.right == 0. and not blocked.task_finished


@pytest.mark.parametrize('mode', ['BRAKE', 'HOLD'])
@pytest.mark.parametrize('gate', ['WAITING_MAP', 'WAITING_REFERENCE'])
def test_motion_lease_refresh_does_not_hide_last_control_tick_safety_gate(mode, gate):
    with control_harness() as h:
        h.speed = .2
        if gate == 'WAITING_MAP':
            h.until(lambda: h.control.state is not None and h.control.peers_ready('decision', 'perception'),
                    grid=False)
        else:
            h.ready()
        h.send_reference(motion_reference(h, mode))
        h.until(lambda: h.reports and h.reports[-1].state == gate, grid=gate != 'WAITING_MAP')
        previous_reason = h.reports[-1].reason
        h.control.control_timer.cancel()
        updated = motion_reference(h, mode)
        h.send_reference(updated)
        h.until(lambda: h.reports and h.reports[-1].reference_sequence == updated.sequence,
                grid=gate != 'WAITING_MAP')
        assert h.reports[-1].state == gate
        assert h.reports[-1].reason == previous_reason
        assert h.reports[-1].left == h.reports[-1].right == 0.
        assert not h.reports[-1].task_finished
