"""Real rclpy/process-worker integration without Gazebo; isolated ROS domain."""
import time
import json
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from sim_env.simulation import Simulator
from sim_env.configuration import C, T


def test_process_planning_parameters_stale_inputs_cancel_and_clock_reset():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/navigation_smoke', '-p', 'use_sim_time:=true'])
    simulator = driver = executor = None
    try:
        simulator = Simulator()
        driver = Node('navigation_test_driver')
        executor = SingleThreadedExecutor()
        executor.add_node(simulator)
        executor.add_node(driver)
        clock_pub = driver.create_publisher(Clock, '/clock', 10)
        odom_pub = driver.create_publisher(Odometry, 'odom', 1)
        map_pub = driver.create_publisher(Costmap, 'costmap_raw',
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        goal_pub = driver.create_publisher(PoseStamped, 'goal', 1)
        left, statuses = [], []
        subscriptions = [driver.create_subscription(Float32, 'thrusters/left_thrust_cmd', left.append, 10),
                         driver.create_subscription(String, 'navigation_status', lambda m: statuses.append(json.loads(m.data)), 10)]
        simulated_time = 10.0

        def spin(duration=.04):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.002)

        def inputs(odom=True, grid=True):
            nonlocal simulated_time
            simulated_time += .05
            clock = Clock()
            clock.clock.sec = int(simulated_time)
            clock.clock.nanosec = round((simulated_time - int(simulated_time)) * 1e9)
            clock_pub.publish(clock)
            if odom:
                message = Odometry()
                message.header.stamp = clock.clock
                message.header.frame_id = 'navigation_smoke/map'
                message.child_frame_id = 'navigation_smoke/base_link'
                message.pose.pose.position.x, message.pose.pose.position.y = 2.5, 10.5
                message.pose.pose.orientation.w = 1.
                odom_pub.publish(message)
            if grid:
                message = Costmap()
                message.header.stamp = message.metadata.update_time = clock.clock
                message.header.frame_id = 'navigation_smoke/map'
                message.metadata.size_x = message.metadata.size_y = 30
                message.metadata.resolution = 1.
                message.metadata.origin.orientation.w = 1.
                message.data = [0] * 900
                map_pub.publish(message)
            spin()

        def until(predicate, timeout=5, **kwargs):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                inputs(**kwargs)
            assert predicate(), (simulator.mode, simulator.reason, dict(simulator.counters))

        until(lambda: simulator.state is not None and simulator.grid is not None)
        goal = PoseStamped()
        goal.header.frame_id = 'navigation_smoke/map'
        goal.pose.position.x, goal.pose.position.y = 18.5, 10.5
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        until(lambda: simulator.trajectory is not None and any(m.data > .01 for m in left), timeout=10)
        assert simulator.worker.future is not None or simulator.counters['global_searches'] >= 1
        assert simulator.trajectory.frame_id == 'navigation_smoke/map'
        assert not simulator.timings['control_ms'] or max(simulator.timings['control_ms']) < 100.

        # Validation does not alter a live PID until the next control cycle.
        simulator.controller.integral_speed = .123
        response = simulator.set_parameters_atomically([Parameter(C+'pid_controller.k_p_speed', value=4.)])
        assert response.successful and simulator.controller.config['k_p_speed'] == 2.
        assert simulator.controller.integral_speed == .123
        until(lambda: simulator.controller.config['k_p_speed'] == 4.)
        assert simulator.counters['parameter_updates'] == 1
        assert not simulator.set_parameters_atomically([Parameter(T+'los.lookahead_distance', value=-1.)]).successful
        assert not simulator.set_parameters_atomically([Parameter(C+'control_frequency', value=20.)]).successful

        until(lambda: simulator.mode == 'WAITING_STATE', odom=False)
        left.clear()
        for _ in range(8):
            inputs(odom=False)
        assert left and all(m.data == 0. for m in left)
        until(lambda: simulator.mode == 'NAVIGATING')

        client = driver.create_client(Trigger, 'cancel_navigation')
        until(lambda: client.service_is_ready())
        future = client.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success and simulator.goal is None
        left.clear()
        for _ in range(8):
            inputs()
        assert left and all(m.data == 0. for m in left)

        exporter = driver.create_client(Trigger, 'export_parameters')
        until(lambda: exporter.service_is_ready())
        future = exporter.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success and 'k_p_speed: 4.0' in future.result().message

        goal_pub.publish(goal)
        until(lambda: simulator.goal is not None)
        simulated_time = 1.
        inputs(odom=False, grid=False)
        until(lambda: simulator.counters['clock_resets'] == 1, odom=False, grid=False)
        assert simulator.goal is None and simulator.trajectory is None and simulator.grid is None
        assert statuses
    finally:
        if executor is not None:
            executor.shutdown()
        if simulator is not None:
            simulator.destroy_node()
        if driver is not None:
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
