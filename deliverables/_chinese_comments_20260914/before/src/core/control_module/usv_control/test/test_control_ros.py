"""Control-side ROS contracts, using real messages and the unchanged PID/LQR."""
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
        self.block = False
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
            self.map_pub.publish(msg)
        if reference:
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


def test_control_tracks_then_cancels_and_rejects_old_identity_or_sequence():
    with control_harness() as h:
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


def test_control_pauses_clock_and_recovers_temporary_state_loss():
    with control_harness(simulated=True) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: h.control.mode == 'NAVIGATING')
        h.until(lambda: h.control.mode == 'WAITING_STATE', odom=False, reference=True)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.until(lambda: h.control.mode == 'NAVIGATING', reference=True)
        h.until(lambda: h.control.mode == 'SIMULATION_PAUSED', clock=False)
        assert h.reports[-1].left == h.reports[-1].right == 0.
        h.until(lambda: h.control.mode == 'NAVIGATING', reference=True)
        h.simulated_time = 1.
        h.until(lambda: h.reports and h.reports[-1].clock_reset, odom=False, grid=False)
        assert h.control.goal is None and h.control.trajectory is None


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


def test_initial_ros_parameters_select_existing_lqr_controller():
    from usv_navigation.configuration import C
    with control_harness(extra_args=('-p', C + 'controller_name:=lqr',
                                     '-p', C + 'lqr_controller.lookahead_time:=1.2')) as h:
        h.ready()
        h.send_reference()
        h.until(lambda: any(x.data > 0. for x in h.left))
        assert h.control.controller_name == 'lqr'
        assert h.control.controller.config['lookahead_time'] == 1.2
