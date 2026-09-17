"""Decision/control integration over real ROS messages, with synthetic sensors."""
import json
import time
import uuid
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from usv_navigation_interfaces.msg import ModuleHeartbeat, ControlReport
from usv_navigation.configuration import C, T
from usv_control.control import ControlNode
from usv_decision.decision import DecisionNode


class DistributedHarness:
    def __init__(self):
        self.namespace = 'distributed_' + uuid.uuid4().hex[:10]
        rclpy.init(args=['--ros-args', '-r', '__ns:=/' + self.namespace,
                        '-p', 'use_sim_time:=true'])
        self.decision = DecisionNode()
        self.control = ControlNode()
        self.driver = Node('simulation_inputs')
        self.executor = SingleThreadedExecutor()
        for node in (self.decision, self.control, self.driver):
            self.executor.add_node(node)
        self.goal_pub = self.driver.create_publisher(PoseStamped, 'goal', 1)
        self.odom_pub = self.driver.create_publisher(Odometry, 'odom', 1)
        self.map_pub = self.driver.create_publisher(Costmap, 'costmap_raw',
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.heartbeat_pub = self.driver.create_publisher(ModuleHeartbeat, 'navigation/heartbeat', 10)
        self.clock_pub = self.driver.create_publisher(Clock, '/clock', 10)
        self.cancel_client = self.driver.create_client(Trigger, 'cancel_navigation')
        self.left, self.reports, self.statuses = [], [], []
        self.subscriptions = [
            self.driver.create_subscription(Float32, 'thrusters/left_thrust_cmd', self.left.append, 10),
            self.driver.create_subscription(ControlReport, 'navigation/control_report', self.reports.append, 10),
            self.driver.create_subscription(String, 'navigation_status',
                                            lambda msg: self.statuses.append(json.loads(msg.data)), 10),
        ]
        self.x, self.y, self.speed = 2.5, 10.5, 0.
        self.heartbeat_sequence = 0
        self.simulated_time = 10.

    def spin(self, duration=.035):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.002)

    def step(self):
        self.simulated_time += .05
        clock = Clock()
        clock.clock.sec = int(self.simulated_time)
        clock.clock.nanosec = round((self.simulated_time % 1) * 1e9)
        self.clock_pub.publish(clock)
        self.spin(.005)
        stamp = clock.clock
        self.heartbeat_sequence += 1
        self.heartbeat_pub.publish(ModuleHeartbeat(module='perception', session='sensor-process-one',
                                    sequence=self.heartbeat_sequence, ready=True))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.namespace + '/map'
        odom.child_frame_id = self.namespace + '/base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)
        grid = Costmap()
        grid.header.stamp = stamp
        grid.header.frame_id = self.namespace + '/map'
        grid.metadata.size_x = grid.metadata.size_y = 30
        grid.metadata.resolution = 1.
        grid.metadata.origin.orientation.w = 1.
        grid.data = [0] * 900
        self.map_pub.publish(grid)
        self.spin()

    def until(self, predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.step()
        assert predicate(), {'decision': (self.decision.mode, self.decision.reason, self.decision.task_id,
                                         dict(self.decision.counters)),
                             'control': (self.control.mode, self.control.reason, self.control.task_id,
                                         dict(self.control.counters))}

    def ready(self):
        self.until(lambda: self.decision.state is not None and self.decision.grid is not None
                   and self.control.state is not None and self.control.grid is not None
                   and self.decision.peers_ready('control', 'perception')
                   and self.control.peers_ready('decision', 'perception'))

    def goal(self, x=18.5, y=10.5, valid=True):
        msg = PoseStamped()
        msg.header.frame_id = self.namespace + '/map' if valid else ''
        msg.header.stamp = self.driver.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.
        self.goal_pub.publish(msg)

    def moving(self):
        self.until(lambda: self.control.mode == 'NAVIGATING'
                   and any(msg.data > .01 for msg in self.left), timeout=10.)

    def close(self):
        self.executor.shutdown()
        self.decision.destroy_node()
        self.control.destroy_node()
        self.driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@contextmanager
def distributed_harness():
    harness = None
    try:
        harness = DistributedHarness()
        yield harness
    finally:
        if harness is not None:
            harness.close()
        elif rclpy.ok():
            rclpy.shutdown()


def test_new_goal_runs_and_cancel_waits_for_control_zero_ack():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        # A goal must never be preceded by a STOP bearing that same task ID.
        assert h.control.goal == h.decision.goal
        assert h.driver.count_publishers('thrusters/left_thrust_cmd') == 1
        h.executor.remove_node(h.control)
        future = h.cancel_client.call_async(Trigger.Request())
        h.until(lambda: h.decision.goal is None)
        h.spin(.15)
        assert not future.done(), 'Cancellation succeeded without any controller acknowledgement'
        h.executor.add_node(h.control)
        h.until(future.done)
        assert future.result().success
        h.until(lambda: h.control.goal is None and h.control.mode == 'CANCELLED')
        h.left.clear()
        for _ in range(6):
            h.step()
        assert h.left and all(msg.data == 0. for msg in h.left)


def test_overlapping_cancels_each_require_their_own_control_zero_ack():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        h.executor.remove_node(h.control)
        first = h.cancel_client.call_async(Trigger.Request())
        h.until(lambda: len(h.decision.cancel_waiters) == 1)
        first_task, first_sequence = h.decision.cancel_waiters[0][:2]
        second = h.cancel_client.call_async(Trigger.Request())
        h.until(lambda: len(h.decision.cancel_waiters) == 2)
        second_task, second_sequence = h.decision.cancel_waiters[1][:2]
        assert not first.done() and not second.done()
        h.executor.add_node(h.control)
        h.until(lambda: first.done() and second.done())
        # Success requires an actual zero report for each cancelled task; a
        # later cancellation cannot stand in for a missing earlier ACK.
        for task, sequence in ((first_task, first_sequence), (second_task, second_sequence)):
            assert any(report.task_id == task and report.reference_sequence >= sequence
                       and report.left == report.right == report.lateral == 0.
                       for report in h.reports), 'The controller did not acknowledge this cancellation'
        assert first.result().success
        assert second.result().success
        assert h.decision.goal is None and h.control.goal is None


def test_cancel_ack_after_replacement_completes_request_without_changing_current_task(monkeypatch):
    old_report_states = []
    original_callback = DecisionNode._on_control_report

    def observe_report(node, message):
        # Observe the real subscription callback so even a transient overwrite
        # of the replacement's state or cached control report is visible.
        before = (node.task_id, node.goal, node.mode, node.reason,
                  node.sequence, id(node.trajectory), id(node.control_report))
        original_callback(node, message)
        if message.task_id < before[0] and message.state == 'CANCELLED':
            after = (node.task_id, node.goal, node.mode, node.reason,
                     node.sequence, id(node.trajectory), id(node.control_report))
            old_report_states.append((before, after))

    monkeypatch.setattr(DecisionNode, '_on_control_report', observe_report)
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        h.executor.remove_node(h.control)
        future = h.cancel_client.call_async(Trigger.Request())
        h.until(lambda: len(h.decision.cancel_waiters) == 1)
        cancelled_task, cancelled_sequence = h.decision.cancel_waiters[0][:2]
        h.goal(16.5)
        h.until(lambda: h.decision.task_id > cancelled_task and h.decision.goal == (16.5, 10.5, 0.))
        replacement_task = h.decision.task_id
        assert not future.done()
        h.executor.add_node(h.control)
        h.until(future.done)
        assert any(report.task_id == cancelled_task and report.reference_sequence >= cancelled_sequence
                   and report.left == report.right == report.lateral == 0.
                   for report in h.reports), 'The controller did not acknowledge the cancelled task'
        assert future.result().success
        assert old_report_states and all(before == after for before, after in old_report_states)
        assert h.decision.task_id == replacement_task
        assert h.decision.goal == (16.5, 10.5, 0.)
        h.left.clear()
        h.moving()
        assert h.control.task_id == replacement_task


def test_superseded_cancel_times_out_without_any_control_ack():
    with distributed_harness() as h:
        h.ready()
        h.executor.remove_node(h.control)
        future = h.cancel_client.call_async(Trigger.Request())
        h.until(lambda: len(h.decision.cancel_waiters) == 1)
        cancelled_task = h.decision.task_id
        h.goal(16.5)
        h.until(lambda: h.decision.task_id > cancelled_task and h.decision.goal is not None)
        assert not future.done()
        h.until(future.done)
        assert not future.result().success
        assert not any(report.task_id == cancelled_task for report in h.reports)
        assert h.decision.goal == (16.5, 10.5, 0.)


def test_replacement_settling_and_completion_are_shared_between_modules():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        previous = h.control.task_id
        h.goal(16.5)
        h.until(lambda: h.control.task_id > previous and h.control.goal == (16.5, 10.5, 0.))
        h.x, h.speed = 16.4, 1.49
        h.until(lambda: h.control.mode == 'SETTLING')
        assert h.decision.goal is not None
        assert h.control.last_published_command.left == h.control.last_published_command.right == 0.
        task = h.control.task_id
        h.speed = .1
        h.until(lambda: h.decision.goal is None and h.decision.mode == 'GOAL_REACHED')
        assert any(msg.task_id == task and msg.task_finished for msg in h.reports)
        assert h.control.goal is None


def test_los_and_pid_hot_updates_keep_same_control_task_and_integral():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        # Do not allow normal integration to hide an accidental reset. Zero
        # integral gain makes the retained value observable during real ticks.
        assert h.decision.set_parameters_atomically([
            Parameter(C + 'pid_controller.k_i_speed', value=0.)]).successful
        h.until(lambda: h.control.controller.config['k_i_speed'] == 0.)
        h.control.controller.integral_speed = .123
        task = h.control.task_id
        assert h.decision.set_parameters_atomically([
            Parameter(T + 'los.lookahead_distance', value=2.),
            Parameter(C + 'pid_controller.k_p_speed', value=4.)]).successful
        h.until(lambda: h.control.config[T + 'los.lookahead_distance'] == 2.
                and h.control.controller.config['k_p_speed'] == 4.)
        h.until(lambda: h.control.trajectory is not None and h.control.mode == 'NAVIGATING')
        assert h.control.task_id == task == h.decision.task_id
        assert h.control.controller.integral_speed == pytest.approx(.123)


def test_restarted_control_cannot_resume_until_a_new_goal_is_published():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        old_session = h.control.session
        h.executor.remove_node(h.control)
        h.control.destroy_node()
        h.control = ControlNode()
        h.executor.add_node(h.control)
        h.until(lambda: h.decision.peers['control'].session != old_session
                and h.decision.goal is None)
        h.left.clear()
        for _ in range(8):
            h.step()
        assert h.left and all(msg.data == 0. for msg in h.left)
        assert h.control.goal is None
        h.goal()
        h.left.clear()
        h.moving()


def test_invalid_replacement_goal_stops_both_modules():
    with distributed_harness() as h:
        h.ready()
        h.goal()
        h.moving()
        h.goal(valid=False)
        h.until(lambda: h.control.mode == 'INVALID_GOAL' and h.control.goal is None)
        assert h.decision.goal is None
        h.left.clear()
        for _ in range(6):
            h.step()
        assert h.left and all(msg.data == 0. for msg in h.left)
