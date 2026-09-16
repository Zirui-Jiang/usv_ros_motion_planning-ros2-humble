# 任务 ROS 适配回归：覆盖地理服务、替换事务、控制器交接、安全制动与发布权冲突。
"""ROS adapter contracts, using real services and controlled state/map inputs."""
import json
import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from usv_mission_interfaces.srv import SetMission
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from common.geodesy import GeoReference
from common.models import AlgorithmStatus, ControlCommand, GridSnapshot, VesselState
from trajectory_planner import Trajectory
from usv_mission.runtime import PlanningResponse
from usv_mission.configuration import C, T
from usv_mission.node import MissionNode


@pytest.fixture
def simulator():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/mission_test',
                    '-p', 'mission.geo.enabled:=true',
                    '-p', 'mission.geo.latitude:=-33.7242',
                    '-p', 'mission.geo.longitude:=150.68'])
    node = MissionNode()
    try:
        now = node.now_seconds()
        node.state = VesselState(x=23., y=-10., stamp=now, frame_id=node.frame)
        node.grid = GridSnapshot(np.zeros((160, 160), np.uint8), .5, -20., -40., now, node.frame)
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def request(mode='station_keep', x=23., y=-10., heading=90.):
    lat, lon, _ = GeoReference(-33.7242, 150.68).to_geographic(x, y)
    return SetMission.Request(mode=mode, latitude=lat, longitude=lon, heading_deg=heading,
                              speed=.5, position_tolerance=.5, watch_radius=10.,
                              orbit_radius=10., clockwise=True, laps=0, duration=0.)


def start(node, **kwargs):
    response = node.mission.set_mission(request(**kwargs), SetMission.Response())
    assert response.accepted, response.reason
    return response


# 验证：类型化 SetMission 服务接受地理目标并转换到正确局部坐标。
def test_typed_service_accepts_geographic_target(simulator):
    driver = Node('mission_client')
    executor = SingleThreadedExecutor()
    executor.add_node(simulator)
    executor.add_node(driver)
    try:
        client = driver.create_client(SetMission, 'set_mission')
        assert client.wait_for_service(timeout_sec=3.)
        future = client.call_async(request())
        deadline = time.monotonic() + 3.
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert future.done() and future.result().accepted
        target = simulator.mission.manager.target_pose
        assert target == pytest.approx((23., -10., 0.), abs=1e-5)
        assert simulator.mission.active
    finally:
        executor.shutdown()
        driver.destroy_node()


# 验证：非法替换保留旧任务及版本，成功替换才递增任务编号。
def test_invalid_replacement_preserves_running_task(simulator):
    first = start(simulator)
    version = simulator.version
    bad = request()
    bad.latitude = float('nan')
    response = simulator.mission.set_mission(bad, SetMission.Response())
    assert not response.accepted
    assert simulator.version == version and response.mission_id == first.mission_id
    assert simulator.mission.active
    second = start(simulator, heading=0.)
    assert second.mission_id == first.mission_id + 1


# 验证：定点保持持续活动且生成有限的推进器命令。
def test_stationary_hold_remains_active_and_outputs_finite_commands(simulator):
    start(simulator, heading=0.)
    commands = []
    simulator._publish_command = commands.append
    simulator.last_control = simulator.now_seconds() - .1
    simulator._control()
    assert simulator.mission.active
    assert commands and commands[-1].is_valid()
    assert commands[-1].lateral == 0.
    assert abs(commands[-1].left) + abs(commands[-1].right) > 0.
    payload = simulator.mission.status()
    json.dumps(payload, allow_nan=False)


# 验证：保持任务也遵守状态和地图超时，失效返回零推力。
@pytest.mark.parametrize('missing,expected', [('state', 'WAITING_STATE'), ('grid', 'WAITING_MAP')])
def test_holding_obeys_input_timeout(simulator, missing, expected):
    start(simulator, heading=0.)
    setattr(simulator, missing, None)
    command = simulator.mission.control(simulator.now_seconds(), .1)
    assert command.left == command.right == 0. and simulator.mode == expected


# 验证：导航交给保持时从实际已发命令限制推力变化率。
def test_navigation_to_holding_preserves_actual_command_slew(simulator):
    from dataclasses import replace
    start(simulator, x=27.)
    now = simulator.now_seconds()
    simulator.mission.prepare(now)
    assert simulator.mission.intent.kind == 'navigate'
    previous = ControlCommand(.1, .1)
    simulator._publish_command(previous)
    simulator.state = replace(simulator.state, x=25.01, u=.5)
    command = simulator.mission.control(now, .1)
    assert simulator.mission.intent.kind == 'hold'
    assert abs(command.left - previous.left) <= .06 + 1e-9
    assert abs(command.right - previous.right) <= .06 + 1e-9


# 验证：等待退让参考期间，对运动船舶使用已验证的制动路径。
def test_pending_escape_brakes_moving_vessel_with_clear_stop_path(simulator):
    from dataclasses import replace
    start(simulator, mode='orbit')
    simulator.state = replace(simulator.state, u=.5)
    simulator.mission.manager.request_escape('occupied circle ahead')
    command = simulator.mission.control(simulator.now_seconds(), .1)
    assert simulator.mission.intent.kind == 'escape'
    assert command.left < 0 and command.right < 0
    assert simulator.trajectory is None


# 验证：没有可验证的制动命令时返回零推力。
def test_pending_escape_never_uses_unverified_braking(simulator, monkeypatch):
    from dataclasses import replace
    start(simulator, mode='orbit')
    simulator.state = replace(simulator.state, u=.5)
    simulator.mission.manager.request_escape('occupied circle ahead')
    monkeypatch.setattr('usv_mission.mission_adapter.clear_control', lambda *a, **kw: False)
    command = simulator.mission.control(simulator.now_seconds(), .1)
    assert command.left == command.right == command.lateral == 0.


# 验证：可选择更强的安全制动，但不得缩小安全边距。
def test_escape_selects_stronger_verified_brake_without_relaxing_margin(simulator, monkeypatch):
    from dataclasses import replace
    start(simulator, mode='orbit')
    simulator.state = replace(simulator.state, u=.5)
    simulator.mission.manager.request_escape('occupied circle ahead')
    margins = []
    def gate(state, command, *args, **kwargs):
        margins.append(kwargs['margin'])
        return command.left < -.2
    monkeypatch.setattr('usv_mission.mission_adapter.clear_control', gate)
    command = simulator.mission.control(simulator.now_seconds(), .1)
    assert -.4 <= command.left < -.2 and -.4 <= command.right < -.2
    assert margins == [.1, .1]


# 验证：条件起步轨迹不能在实测停止前交给控制器。
def test_conditional_departure_plan_waits_for_measured_stop_before_adoption(simulator, monkeypatch):
    from dataclasses import replace
    start(simulator, mode='orbit')
    simulator.state = replace(simulator.state, u=.5)
    simulator.mission.manager.request_escape('LOS needs stationary reorientation')
    now = simulator.now_seconds()
    simulator.mission.prepare(now)
    trajectory = Trajectory([[0., 0., 23., -10., 0., 0., .5, 0.],
                             [1., .5, 23.5, -10., 0., 0., .5, 0.]],
                            simulator.state.stamp, simulator.frame, (33., -10.))
    response = PlanningResponse(simulator.version, AlgorithmStatus.SUCCESS,
                                np.array([[23., -10., 0.], [33., -10., 0.]]),
                                trajectory, .01, selected_goal=(33., -10., 0.),
                                metadata={'departure_pose': (23.3, -10., math.pi/2)})
    simulator.mission.consume_plan(response, now)
    assert simulator.mission.intent.kind == 'brake'
    assert simulator.trajectory is None
    monkeypatch.setattr(simulator.worker, 'offer', lambda *a: pytest.fail('braking must not request a route'))
    simulator.mission.planning(now)
    command = simulator.mission.control(now, .1)
    assert command.left < 0 and command.right < 0


# 验证：退让命令不安全时改用单独通过预测校验的制动命令。
def test_unsafe_detour_command_uses_separately_verified_braking(simulator, monkeypatch):
    from dataclasses import replace
    start(simulator, mode='orbit')
    simulator.state = replace(simulator.state, u=.5)
    simulator.mission.manager.request_escape('occupied circle ahead')
    now = simulator.now_seconds()
    simulator.mission.prepare(now)
    trajectory = Trajectory([[0., 0., 23., -10., 0., 0., .5, 0.],
                             [1., .5, 23.5, -10., 0., 0., .5, 0.]],
                            simulator.state.stamp, simulator.frame, (33., -10.))
    response = PlanningResponse(simulator.version, AlgorithmStatus.SUCCESS,
                                np.array([[23., -10., 0.], [33., -10., 0.]]),
                                trajectory, .01, selected_goal=(33., -10., 0.))
    simulator.mission.consume_plan(response, now)
    checked = []
    def gate(state, command, *args, **kwargs):
        checked.append(command)
        return command.left < 0 and command.right < 0
    monkeypatch.setattr('usv_mission.mission_adapter.clear_control', gate)
    monkeypatch.setattr(simulator.mission.nav_controller, 'compute',
                        lambda *args: ControlCommand(.2, .2))
    command = simulator.mission.control(now, .1)
    assert simulator.mode == 'ESCAPE_CONTROL_BLOCKED'
    assert len(checked) == 2 and checked[0].left == .2
    assert command.left < 0 and command.right < 0


# 验证：取消后旧任务或旧规划响应不会重新启动导航。
def test_cancel_does_not_allow_old_mission_to_restart(simulator):
    start(simulator)
    simulator._cancel(Trigger.Request(), Trigger.Response())
    assert not simulator.mission.active and simulator.goal is None
    commands = []
    simulator._publish_command = commands.append
    for _ in range(3):
        simulator.last_control = simulator.now_seconds() - .1
        simulator._control()
    assert all(c.left == c.right == c.lateral == 0. for c in commands)


# 验证：当前船体占用阻止保持控制，不能豁免为退让。
def test_occupied_hull_stops_hold_without_escape_exemption(simulator):
    start(simulator, heading=0.)
    grid = simulator.grid
    data = grid.data.copy()
    ix, iy = grid.world_to_map(simulator.state.x, simulator.state.y)
    data[iy, ix] = 254
    simulator.grid = GridSnapshot(data, grid.resolution, grid.origin_x, grid.origin_y,
                                 simulator.now_seconds(), grid.frame_id)
    command = simulator.mission.control(simulator.now_seconds(), .1)
    assert simulator.mode == 'CURRENT_FOOTPRINT_BLOCKED'
    assert command.left == command.right == 0.


# 验证：没有普通目标的待命任务仍主动监测近场入侵。
def test_standby_without_goal_monitors_intrusion(simulator):
    start(simulator, mode='standby')
    base = simulator.now_seconds()
    # Let the task's centre arrival dwell complete without stopping monitoring.
    for delta in (0., .5, 1.1):
        simulator.state = VesselState(x=23., y=-10., stamp=base + delta, frame_id=simulator.frame)
        simulator.grid = GridSnapshot(simulator.grid.data, .5, -20., -40., base + delta, simulator.frame)
        simulator.mission.prepare(base + delta)
    assert simulator.mission.intent.kind == 'stop'
    data = simulator.grid.data.copy()
    ix, iy = simulator.grid.world_to_map(23., -7.)
    data[iy, ix] = 254
    simulator.grid = GridSnapshot(data, .5, -20., -40., base + 1.2, simulator.frame)
    simulator.state = VesselState(x=23., y=-10., stamp=base + 1.2, frame_id=simulator.frame)
    simulator.mission.prepare(base + 1.2)
    assert simulator.mission.intent.kind == 'escape'


# 验证：选中切向航路在参考版本切换后仍能保留，旧版本结果被统计为过期。
def test_selected_tangent_route_survives_reference_generation_change(simulator):
    start(simulator, mode='orbit')
    simulator.mission.prepare(simulator.now_seconds())
    old_version = simulator.version
    selected = (33., -10., -math.pi / 2)
    route = np.array([[23., -10., 0.], [33., -7., -math.pi / 2], selected])
    trajectory = Trajectory([[0., 0., 23., -10., 0., 0., .5, 0.],
                             [1., .5, 23.5, -10., 0., 0., .5, 0.]],
                            simulator.state.stamp, simulator.frame, selected[:2])
    response = PlanningResponse(old_version, AlgorithmStatus.SUCCESS, route, trajectory, .1,
                                selected_goal=selected)
    simulator.mission.consume_plan(response)
    assert simulator.version > old_version
    np.testing.assert_array_equal(simulator.route, route)
    simulator.mission.prepare(simulator.now_seconds())
    np.testing.assert_array_equal(simulator.route, route)
    assert simulator.trajectory is trajectory
    # A pending result from the pre-selection generation cannot overwrite it.
    simulator.mission.consume_plan(response)
    assert simulator.counters['stale_results'] == 1


# 验证：在线增益更新传递到任务控制器且保留任务精度及积分状态。
def test_hot_gain_update_reaches_mission_controller_without_overwriting_precision(simulator):
    start(simulator, mode='orbit')
    controller = simulator.mission.nav_controller
    controller.integral_speed = .12
    result = simulator.set_parameters_atomically([Parameter(C + 'pid_controller.k_p_speed', value=3.)])
    assert result.successful
    simulator._apply_parameters()
    assert controller.config['k_p_speed'] == 3.
    assert controller.config['goal_distance_tolerance'] == .5
    assert controller.integral_speed == .12
    old_limit = simulator.config[T + 'los.yaw_rate_max']
    result = simulator.set_parameters_atomically([Parameter(T + 'los.yaw_rate_max', value=.01)])
    assert not result.successful
    assert simulator.get_parameter(T + 'los.yaw_rate_max').value == old_limit


# 验证：竞争发布者阻止任务、控制和取消输出并锁存冲突。
def test_another_publisher_blocks_commands_tasks_and_cancel_output(simulator):
    first = start(simulator)
    driver = Node('other_controller')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    # Deliberately do not run the mission control timers while this test drives
    # their callbacks directly. The real DDS graph still reports the publishers.
    received = []
    statuses = []
    driver.create_subscription(Float32, simulator.thrust_pubs[0].topic_name,
                               received.append, 10)
    driver.create_subscription(String, 'navigation_status',
                               lambda message: statuses.append(json.loads(message.data)), 10)
    competitor = driver.create_publisher(Float32, simulator.thrust_pubs[0].topic_name, 1)
    try:
        deadline = time.monotonic() + 3.
        while simulator.count_publishers(competitor.topic_name) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert simulator.count_publishers(competitor.topic_name) == 2
        version = simulator.version
        simulator._publish_command(ControlCommand(.2, .2))
        assert simulator.mode == 'CONTROL_CONFLICT'
        assert simulator.version == version + 1
        assert simulator.trajectory is None
        rejected = simulator.mission.set_mission(request(), SetMission.Response())
        assert not rejected.accepted and rejected.mission_id == first.mission_id
        assert 'restart' in rejected.reason
        simulator._on_goal(PoseStamped())
        assert simulator.mode == 'CONTROL_CONFLICT'
        simulator._control()
        result = simulator._cancel(Trigger.Request(), Trigger.Response())
        assert result.success and 'inhibited' in result.message
        assert not simulator.mission.active
        driver.destroy_publisher(competitor)
        # Removal does not grant authority back, including shutdown/zero paths.
        simulator._publish_command(ControlCommand())
        simulator._control()
        simulator._publish_status()
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert not received
        assert statuses and statuses[-1]['state'] == 'CONTROL_CONFLICT'
        assert statuses[-1]['control_authority']['restart_required'] is True
        assert simulator.counters['control_conflicts'] == 1
    finally:
        executor.shutdown()
        driver.destroy_node()


# 验证：发布权门控检查真实重映射话题，未知 ROS 图状态禁止发布。
def test_node_uses_actual_remapped_topic_names_and_inhibits_unknown_graph(simulator, monkeypatch):
    seen = []
    monkeypatch.setattr(simulator, 'count_publishers', lambda topic: seen.append(topic) or 1)
    assert simulator.command_authority_ready()
    assert seen == [publisher.topic_name for publisher in simulator.thrust_pubs]
    monkeypatch.setattr(simulator, 'count_publishers', lambda topic: 0)
    assert not simulator.command_authority_ready()
    assert simulator.mode == 'WAITING_CONTROL_AUTHORITY'
    assert not simulator.control_authority.conflicted
    monkeypatch.setattr(simulator, 'count_publishers', lambda topic: 1)
    assert simulator.command_authority_ready()
    assert simulator.mode == 'IDLE'


# 验证：交接零命令时新发现冲突也不能接受并安装新任务。
def test_conflict_discovered_during_handover_zero_does_not_accept_new_task(simulator, monkeypatch):
    first = start(simulator)
    reset = simulator.reset_task

    def reset_with_new_writer(*args, **kwargs):
        monkeypatch.setattr(simulator, 'count_publishers', lambda topic: 2)
        return reset(*args, **kwargs)

    monkeypatch.setattr(simulator, 'reset_task', reset_with_new_writer)
    result = simulator.mission.set_mission(request(), SetMission.Response())
    assert not result.accepted and result.mission_id == first.mission_id
    assert simulator.mode == 'CONTROL_CONFLICT'
    assert not simulator.mission.active
    assert simulator.goal is None
