# 解析决策模块 launch 动作，验证业务节点、兼容节点名与共享参数，不实际启动子进程。
"""Exercise the usv_decision launch actions without starting subprocesses."""
import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

PACKAGE = Path(__file__).resolve().parents[1]
SHARED = PACKAGE.parents[1] / 'shared' / 'usv_navigation'
CONTROLLER_YAML = SHARED / 'config' / 'robots' / 'otter' / 'controller_params_otter.yaml'
YAML_CONTROLLER = yaml.safe_load(CONTROLLER_YAML.read_text(encoding='utf-8'))['controller']['controller_name']


# 验证只拥有决策节点、保留 usv_simulation 名称并传递命名空间与时钟参数。
@pytest.mark.parametrize('overrides, namespace, simulated, expected_controller', [
    ({}, 'otter', True, YAML_CONTROLLER),
    ({'robot_namespace': '/boat_two/', 'use_sim_time': 'false'}, 'boat_two', False, YAML_CONTROLLER),
    ({'controller_name': 'pid'}, 'otter', True, 'pid'),
])
def test_standalone_decision_launch_owns_only_its_business_node(
        monkeypatch, overrides, namespace, simulated, expected_controller):
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
    from launch_ros.actions import Node
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.utilities import evaluate_parameters

    path = PACKAGE / 'launch' / 'decision.launch.py'
    assert path.is_file(), 'The business package must own its launch entry'
    original_share = ament_index_python.packages.get_package_share_directory
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda package: str(SHARED) if package == 'usv_navigation' else original_share(package))
    spec = importlib.util.spec_from_file_location('decision_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)

    actions = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            actions.extend(action.execute(context) or [])
        else:
            actions.append(action)
    assert all(isinstance(action, (Node, LogInfo)) for action in actions)
    node, = (action for action in actions if isinstance(action, Node))
    assert sum(isinstance(action, LogInfo) for action in actions) == 5

    def value(raw):
        return perform_substitutions(context, normalize_to_list_of_substitutions(raw))

    assert value(node.node_package) == 'usv_decision'
    assert value(node.node_executable) == 'decision_node'
    assert value(node._Node__node_name) == 'usv_simulation'
    assert value(node._Node__node_namespace) == namespace
    parameters = {}
    for values in evaluate_parameters(context, node._Node__parameters):
        parameters.update(values)
    assert parameters['use_sim_time'] is simulated
    assert parameters['dynamics.ns'] == namespace
    assert parameters['usv_simulation.controller.control_frequency'] == 10.
    assert parameters['navigation.planning_frequency'] == 5.
    assert parameters['usv_simulation.controller.controller_name'] == expected_controller
