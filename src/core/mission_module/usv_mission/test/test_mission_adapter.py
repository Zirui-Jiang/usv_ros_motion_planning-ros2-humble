# 任务适配层离线回归：用轻量节点替身检验条件起步参考的隔离及新地图后的重试。
"""Adapter/state-machine handover contracts without starting ROS or a worker."""
from dataclasses import replace
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import pytest

from common.collision import footprint_path_is_clear
from common.models import AlgorithmStatus, ControlCommand, GridSnapshot, USVModel, VesselState
from controller.controller_factory import create_controller
from controller.station_keeping_controller import StationKeepingController
from usv_mission.configuration import C, resolve_parameters, section
from usv_mission.missions import MissionManager, MissionSpec
from usv_mission.runtime import PlanningResponse
from trajectory_planner.trajectory import Trajectory


@pytest.fixture
def adapter(monkeypatch):
    # Only the ROS import/publishing boundary is replaced. The adapter, manager,
    # footprint gate and controllers are the production implementations.
    for name, attributes in {
        'geometry_msgs.msg': {'Point': object},
        'visualization_msgs.msg': {'Marker': object, 'MarkerArray': object},
        'rclpy.qos': {'QoSProfile': object, 'DurabilityPolicy': object},
        'usv_mission_interfaces.srv': {'SetMission': object},
        'common.visualizer': {'make_path': lambda *args: None},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / 'usv_mission' / 'mission_adapter.py'
    spec = importlib.util.spec_from_file_location('usv_mission._adapter_handover_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.MissionAdapter.__new__(module.MissionAdapter)
    config = resolve_parameters({})
    model = USVModel()
    offered = []
    noop = lambda *args: None
    node = SimpleNamespace(
        state=VesselState(u=.2), grid=GridSnapshot(np.zeros((400, 400), np.uint8), .25, -50., -50.),
        model=model, config=config, version=1, goal=None, trajectory=None, route=np.empty((0, 3)),
        last_published_command=ControlCommand(), frame='otter/map',
        counters={'stale_results': 0, 'global_searches': 0, 'unsafe_results': 0},
        worker=SimpleNamespace(invalidate=noop, offer=offered.append), offered=offered,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        path_pub=SimpleNamespace(publish=noop), trajectory_pub=SimpleNamespace(publish=noop))
    node.current_footprint_clear = lambda: footprint_path_is_clear(node.grid, [node.state.pose])
    node.trajectory_clear = lambda trajectory: (node.current_footprint_clear()
        and footprint_path_is_clear(node.grid, trajectory.points[:, [2, 3, 4]]))
    result.node = node
    result.manager = MissionManager(MissionSpec('orbit', 0., 0.), (0., 10., 0.), 0., config=config)
    result.manager.request_escape('conditional handover regression')
    result.intent = result.signature = None
    result.parameters = config
    result.holding = StationKeepingController(model)
    result.nav_controller = create_controller('pid', model, section(config, C+'pid_controller.'))
    result.prepare(0.)
    return result


def conditional_response(adapter, metadata):
    trajectory = Trajectory([[0., 0., 0., 0., 0., 0., .5, 0.],
                             [1., .5, .5, 0., 0., 0., .5, 0.]],
                            adapter.node.state.stamp, adapter.node.frame, (8., 0.))
    return PlanningResponse(adapter.node.version, AlgorithmStatus.SUCCESS,
                            np.array([[0., 0., 0.], [8., 0., 0.]]), trajectory, .01,
                            selected_goal=(8., 0., 0.), metadata=metadata)


# 验证：重复条件方案不可执行，候选阻塞直到新地图到达后才重新派发。
@pytest.mark.parametrize('metadata', [
    {'departure_pose': (0., 0., 0.)},
    {'departure_pose': (0., 0., 0.), 'departure_requires_replan': True},
])
def test_repeated_conditional_result_blocks_candidate_until_new_map(adapter, metadata):
    node = adapter.node
    response = conditional_response(adapter, dict(metadata, map_stamp=0.))
    adapter.consume_plan(response, now=0.)
    assert adapter.intent.kind == 'brake' and node.trajectory is None
    for now in (.1, .2, .3, .4, .5, .6):
        node.state = VesselState(stamp=now)
        node.grid = replace(node.grid, stamp=now)
        adapter.prepare(now)
    assert adapter.intent.kind == 'escape'

    # A fresh worker generation can require the same departure yaw after a
    # disturbance. Its hypothetical zero-velocity trajectory is still evidence.
    node.state = VesselState(psi=.2, u=.2, stamp=.7)
    node.grid = replace(node.grid, stamp=.7)
    adapter.consume_plan(replace(response, version=node.version), now=.7)
    assert node.trajectory is None
    assert not len(node.route)
    assert adapter.intent.phase == 'AVOIDANCE_BLOCKED'
    assert adapter.manager.temporary_goal is None
    command = adapter.control(.7, .1)
    assert command.left < 0 and command.right < 0
    for now in (.7, .8, .9):
        node.state = replace(node.state, stamp=now)
        adapter.planning(now)
    assert not node.offered

    node.state = replace(node.state, stamp=1.)
    node.grid = replace(node.grid, stamp=1.)
    adapter.planning(1.)
    assert adapter.intent.kind == 'escape'
    assert node.offered[-1].mission_intent.candidates


# 验证：只有重新规划标志而没有起步位姿的条件结果也不能被执行。
def test_conditional_replan_flag_without_departure_pose_cannot_be_executed(adapter):
    response = conditional_response(adapter, {'departure_requires_replan': True})
    adapter.consume_plan(response, now=0.)
    assert adapter.node.trajectory is None
    assert not len(adapter.node.route)
    assert adapter.intent.phase == 'AVOIDANCE_BLOCKED'
