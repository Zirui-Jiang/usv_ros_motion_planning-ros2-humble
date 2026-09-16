# 检查业务目录下的 ROS 包声明、依赖闭包与无环性，确保业务包不反向依赖仿真。
"""The business modules must be discoverable and independent of simulation."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
BUSINESS = ('usv_perception', 'usv_decision', 'usv_control')
MODULE_PACKAGES = {
    'perception_module': ('usv_perception', 'perception'),
    'decision_module': ('usv_decision', 'path_planner', 'path_processor', 'trajectory_planner'),
    'control_module': ('usv_control', 'controller'),
    'shared': ('usv_navigation', 'usv_navigation_interfaces', 'common'),
    'mission_module': ('usv_mission', 'usv_mission_interfaces'),
}


# 业务分组目录不是包，真正的子包名由各 package.xml 声明。
@pytest.mark.parametrize('module, packages', MODULE_PACKAGES.items())
def test_module_directory_is_not_a_ros_package_and_children_have_declared_names(module, packages):
    directory = ROOT / 'src/core' / module
    assert not (directory / 'package.xml').exists()
    for name in packages:
        assert ET.parse(directory / name / 'package.xml').getroot().findtext('name') == name


# 公共包依赖闭包不能反向包含业务实现或仿真。
@pytest.mark.parametrize('package', ('usv_navigation',))
def test_shared_package_does_not_depend_on_business_or_simulation(package):
    graph = package_graph()
    pending, seen = [package], set()
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        pending.extend(graph.get(current, set()))
    assert not ({'sim_env', *BUSINESS} & seen)


def package_graph():
    graph = {}
    for path in (ROOT / 'src').rglob('package.xml'):
        document = ET.parse(path).getroot()
        name = document.findtext('name')
        assert name not in graph, f'Duplicate ROS package: {name}'
        graph[name] = {element.text for element in document
                       if element.tag.endswith('depend') and element.text}
    return graph


# 单个业务包的依赖闭包不包含其他业务包或 sim_env。
@pytest.mark.parametrize('package', BUSINESS)
def test_business_can_be_built_without_simulation_or_peer_business_packages(package):
    graph = package_graph()
    assert package in graph, f'Independent business package is missing: {package}'
    pending, seen = [package], set()
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        pending.extend(graph.get(current, set()))
    assert 'sim_env' not in seen, f'{package} depends on the simulation environment'
    assert not (set(BUSINESS) - {package}) & seen, f'{package} depends on peer business implementation'


# 对本地包依赖图做环检测，防止目录整理引入循环依赖。
def test_local_package_dependency_graph_is_acyclic():
    graph = package_graph()
    completed = set()

    def visit(package, ancestors):
        assert package not in ancestors, 'Package dependency cycle: ' + ' -> '.join((*ancestors, package))
        if package in completed:
            return
        for dependency in graph[package] & graph.keys():
            visit(dependency, (*ancestors, package))
        completed.add(package)

    for package in graph:
        visit(package, ())
