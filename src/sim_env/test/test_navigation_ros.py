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


def test_transient_controller_timeout_is_counted_and_publishes_zero(monkeypatch):
    import numpy as np
    from common.models import AlgorithmStatus, ControlCommand, GridSnapshot, VesselState
    from trajectory_planner import Trajectory
    rclpy.init(args=['--ros-args', '-r', '__ns:=/navigation_timeout_smoke'])
    simulator = None
    try:
        simulator = Simulator()
        now = simulator.now_seconds()
        simulator.grid = GridSnapshot(np.zeros((40, 40), np.uint8), 1., -10., -10., now, simulator.frame)
        simulator.state = VesselState(stamp=now, frame_id=simulator.frame)
        simulator.trajectory = Trajectory([[0., 0., 0., 0., 0., 0., 1., 0.],
                                           [1., 1., 1., 0., 0., 0., 1., 0.]], now, simulator.frame, (5., 0.))
        simulator.goal = (5., 0., 0.)
        commands = []
        monkeypatch.setattr(simulator, '_publish_command', commands.append)
        monkeypatch.setattr(simulator.controller, 'compute', lambda *args: ControlCommand(
            left=1., right=1., status=AlgorithmStatus.TIMEOUT, message='injected one-step timeout'))
        simulator.last_control = now - .1
        simulator._control()
        assert simulator.mode == 'CONTROL_ERROR' and simulator.goal is not None
        assert simulator.counters['control_result_timeout'] == 1
        assert commands[-1].left == commands[-1].right == 0.
    finally:
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_current_hull_blocks_control_even_when_reference_and_centre_are_clear(monkeypatch):
    import numpy as np
    from common.models import GridSnapshot, VesselState
    from common.collision import footprint_path_is_clear
    from trajectory_planner import Trajectory
    rclpy.init(args=['--ros-args', '-r', '__ns:=/navigation_footprint_smoke'])
    simulator = None
    try:
        simulator = Simulator()
        now = simulator.now_seconds()
        data = np.zeros((80, 80), np.uint8)
        data[44, 44] = 254  # Square [1,1.25] x [1,1.25].
        simulator.grid = GridSnapshot(data, .25, -10., -10., now, simulator.frame)
        simulator.state = VesselState(0., .6, 0., stamp=now, frame_id=simulator.frame)
        simulator.trajectory = Trajectory([[0., 0., 0., 0., 0., 0., 1., 0.],
                                           [1., 1., 1., 0., 0., 0., 1., 0.]], now, simulator.frame, (5., 0.))
        simulator.goal = (5., 0., 0.)
        assert simulator.grid.data[42, 40] == 0
        assert footprint_path_is_clear(simulator.grid, simulator.trajectory.points[:, [2, 3, 4]])
        assert not simulator._trajectory_clear(simulator.trajectory)
        commands = []
        monkeypatch.setattr(simulator, '_publish_command', commands.append)
        simulator.last_control = now - .1
        simulator._control()
        assert simulator.mode == 'CURRENT_FOOTPRINT_BLOCKED'
        assert simulator.trajectory is None and simulator.goal is not None
        assert commands and commands[-1].left == commands[-1].right == 0.
    finally:
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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
        block_grid = False

        def spin(duration=.04):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.002)

        def inputs(odom=True, grid=True, speed=0.):
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
                message.twist.twist.linear.x = speed
                odom_pub.publish(message)
            if grid:
                message = Costmap()
                # Humble's topic publisher leaves metadata.update_time zero;
                # only the GetCostmap service fills it. Freshness uses header.
                message.header.stamp = clock.clock
                message.header.frame_id = 'navigation_smoke/map'
                message.metadata.size_x = message.metadata.size_y = 30
                message.metadata.resolution = 1.
                message.metadata.origin.orientation.w = 1.
                message.data = [0] * 900
                if block_grid:
                    for row in range(30):
                        message.data[row * 30 + 10] = 254
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

        block_grid = True
        until(lambda: simulator.counters['map_invalidations'] >= 1)
        until(lambda: simulator.mode == 'NO_PATH')
        left.clear()
        for _ in range(10):
            inputs()
        assert simulator.trajectory is None and left and all(m.data == 0. for m in left)
        block_grid = False
        until(lambda: simulator.mode == 'NAVIGATING')

        until(lambda: simulator.mode == 'WAITING_STATE', odom=False)
        left.clear()
        for _ in range(8):
            inputs(odom=False)
        assert left and all(m.data == 0. for m in left)
        until(lambda: simulator.mode == 'NAVIGATING')

        # With fresh odometry but a missing map stream, navigation must stop
        # independently of the state timeout and resume on a new map.
        until(lambda: simulator.mode == 'WAITING_MAP', grid=False)
        left.clear()
        for _ in range(8):
            inputs(grid=False)
        assert left and all(m.data == 0. for m in left)
        until(lambda: simulator.mode == 'NAVIGATING')

        # The steady control timer still runs when /clock stops. Let the
        # transition command drain before checking sustained zero output.
        spin(.35)
        left.clear()
        spin(.35)
        assert simulator.mode == 'SIMULATION_PAUSED'
        assert left and all(m.data == 0. for m in left)
        until(lambda: simulator.mode == 'NAVIGATING')

        # An invalid replacement goal must cancel the previous moving task.
        invalid_goal = PoseStamped()
        invalid_goal.pose.orientation.w = 1.
        old_version = simulator.version
        goal_pub.publish(invalid_goal)
        until(lambda: simulator.mode == 'INVALID_GOAL')
        assert simulator.version > old_version and simulator.goal is None
        left.clear()
        for _ in range(8):
            inputs()
        assert left and all(m.data == 0. for m in left)
        goal_pub.publish(goal)
        until(lambda: simulator.mode == 'NAVIGATING')

        near_goal = PoseStamped()
        near_goal.header.frame_id = 'navigation_smoke/map'
        near_goal.pose.position.x, near_goal.pose.position.y = 2.6, 10.5
        near_goal.pose.orientation.w = 1.
        # Publish moving odometry before the close goal to avoid an earlier
        # stationary sample legitimately completing the task.
        inputs(speed=1.49)
        goal_pub.publish(near_goal)
        until(lambda: simulator.mode == 'SETTLING', speed=1.49)
        assert simulator.goal is not None
        left.clear()
        for _ in range(8):
            inputs(speed=1.49)
        assert simulator.goal is not None and left and all(m.data == 0. for m in left)
        until(lambda: simulator.mode == 'GOAL_REACHED', speed=.1)
        assert simulator.goal is None
        goal_pub.publish(goal)
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
