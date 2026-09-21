# 展开真实 launch 动作检查模块归属与参数优先级；测试不执行节点启动动作。
"""Exercise real ROS launch actions without starting any subprocesses."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

pytest.importorskip("launch")
pytest.importorskip("launch_ros")


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = {ET.parse(path).getroot().findtext("name"): path.parent
            for path in PACKAGE.parent.rglob("package.xml")}
CONFIG_PATHS = {
    "dynamics_config": "robots/otter/dynamics_otter.yaml",
    "controller_config": "robots/otter/controller_params_otter.yaml",
    "trajectory_config": "trajectory_planner/trajectory_planner_params.yaml",
    "planner_config": "path_planner/common_planner_params.yaml",
}


def _launch_graph(monkeypatch, filename, overrides=None, *, expand_modules=True):
    pytest.importorskip("launch")
    pytest.importorskip("launch_ros")
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    original_share = ament_index_python.packages.get_package_share_directory
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory",
                        lambda name: str(PACKAGES[name]) if name in PACKAGES else original_share(name))
    path = PACKAGE / "launch" / filename
    assert path.is_file(), f"Independent launch entry is missing: {filename}"
    spec = importlib.util.spec_from_file_location("navigation_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    actions = []

    def value(raw):
        return perform_substitutions(context, normalize_to_list_of_substitutions(raw))

    def visit(entities):
        for action in entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, OpaqueFunction):
                visit(action.execute(context) or [])
            elif isinstance(action, IncludeLaunchDescription) and expand_modules:
                source = action.launch_description_source
                location = value(source._LaunchDescriptionSource__location)
                if Path(location).parent in {package / "launch" for package in PACKAGES.values()}:
                    for name, setting in action.launch_arguments:
                        context.launch_configurations[value(name)] = value(setting)
                    visit(source.get_launch_description(context).entities)
                else:
                    actions.append(action)
            else:
                actions.append(action)

    visit(description.entities)
    return context, actions


# 在实际 launch 上下文中求值，不能把 LaunchConfiguration 对象直接当作参数字符串。
def _value(context, raw):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    return perform_substitutions(context, normalize_to_list_of_substitutions(raw))


def _nodes(context, actions):
    from launch_ros.actions import Node
    return {(_value(context, node.node_package), _value(context, node.node_executable)): node
            for node in actions if isinstance(node, Node)}


def _parameters(context, node):
    from launch_ros.utilities import evaluate_parameters
    result = {}
    for values in evaluate_parameters(context, node._Node__parameters):
        result.update(values)
    return result


# 独立入口只包含自己的业务节点，不偷偷启动另一个业务模块。
@pytest.mark.parametrize("entry,executable,name", [
    ("decision", "decision_node", "usv_simulation"),
    ("control", "control_node", "usv_control"),
])
def test_standalone_navigation_starts_only_its_business_module(monkeypatch, entry, executable, name):
    context, actions = _launch_graph(monkeypatch, entry + ".launch.py")
    nodes = _nodes(context, actions)
    assert set(nodes) == {("usv_" + entry, executable)}
    node = nodes[("usv_" + entry, executable)]
    assert _value(context, node._Node__node_name) == name
    parameters = _parameters(context, node)
    assert parameters["use_sim_time"] is True
    assert parameters["usv_simulation.controller.control_frequency"] == 10.0
    assert parameters["navigation.planning_frequency"] == 5.0
    assert parameters["usv_simulation.controller.controller_name"] == "adrc"
    assert parameters["usv_simulation.trajectory_planner.los.lookahead_distance"] == 1.0


# 感知入口只拥有建图、代价地图和身份监视所需节点。
def test_perception_entry_has_only_its_mapping_costmap_and_restart_monitor(monkeypatch):
    context, actions = _launch_graph(monkeypatch, "perception.launch.py", {
        "robot_namespace": "/boat_two/", "use_sim_time": "false",
        "robot_x": "41.0", "robot_y": "-17.0"})
    nodes = _nodes(context, actions)
    assert set(nodes) == {("usv_perception", "perception_mapping_node"),
                          ("usv_perception", "costmap_support_node"), ("usv_perception", "perception_monitor")}
    for node in nodes.values():
        assert _value(context, node._Node__node_namespace) == "boat_two"
        assert _parameters(context, node)["use_sim_time"] is False
    monitor = _parameters(context, nodes[("usv_perception", "perception_monitor")])
    assert list(monitor["watched_topics"]) == ["odom", "costmap_raw", "map"]
    assert set(monitor) == {"watched_topics", "use_sim_time"}
    costmap = _parameters(context, nodes[("usv_perception", "costmap_support_node")])
    assert costmap["global_frame"] == "boat_two/map"
    assert costmap["robot_base_frame"] == "boat_two/base_link"
    mapping = _parameters(context, nodes[("usv_perception", "perception_mapping_node")])
    assert (mapping["offset_x"], mapping["offset_y"]) == (41.0, -17.0)
    assert list(costmap["plugins"]) == ["obstacle_layer", "inflation_layer"]
    assert costmap["obstacle_layer.scan.topic"] == "/boat_two/sensors/lidars/lidar/scan"
    assert costmap["obstacle_layer.scan.sensor_frame"] == "boat_two/lidar_link"


# 主入口保留原命令形式，并组合仿真支撑及三个独立业务入口。
def test_main_keeps_legacy_command_and_composes_four_independent_entries(monkeypatch):
    from launch.actions import IncludeLaunchDescription
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {
        "headless": "true"}, expand_modules=False)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert {Path(_value(context, action.launch_description_source._LaunchDescriptionSource__location)).name for action in includes} == {
        "simulation_support.launch.py", "perception.launch.py", "decision.launch.py", "control.launch.py"}
    assert {Path(_value(context, action.launch_description_source._LaunchDescriptionSource__location)).parents[1]
            for action in includes} == {PACKAGES[name] for name in
                                       ("sim_env", "usv_perception", "usv_decision", "usv_control")}
    context, expanded = _launch_graph(monkeypatch, "main.launch.py", {
        "headless": "true", "robot_namespace": "boat_two"})
    nodes = _nodes(context, expanded)
    assert ("usv_decision", "decision_node") in nodes
    assert ("usv_control", "control_node") in nodes
    assert ("usv_perception", "perception_monitor") in nodes
    assert ("sim_env", "fake_localization_node") in nodes
    assert ("sim_env", "sim_env_node") not in nodes
    assert not any(package in ("usv_mission", "rviz2") for package, _ in nodes)
    from launch_ros.actions import Node
    assert sum(isinstance(action, Node) and _value(context, action.node_executable) == "fake_localization_node"
               for action in expanded) == 1
    assert sum(isinstance(action, Node) and _value(context, action.node_executable) == "control_node"
               for action in expanded) == 1
    decision = _parameters(context, nodes[("usv_decision", "decision_node")])
    control = _parameters(context, nodes[("usv_control", "control_node")])
    assert decision == control


# 无界面开关由仿真支撑管理，该入口不创建业务节点。
@pytest.mark.parametrize("headless,gui,rviz,want_gui,want_rviz", [
    ("false", "true", "true", "true", True),
    ("true", "true", "true", "false", False),
    ("false", "false", "false", "false", False),
])
def test_simulation_support_owns_gui_policy_without_business_nodes(monkeypatch, headless, gui, rviz, want_gui, want_rviz):
    from launch.actions import IncludeLaunchDescription
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    context, actions = _launch_graph(monkeypatch, "simulation_support.launch.py", {
        "headless": headless, "gui": gui, "rviz": rviz, "world": "example_course"})
    nodes = _nodes(context, actions)
    assert {("gazebo_ros", "spawn_entity.py"), ("robot_state_publisher", "robot_state_publisher"),
            ("joint_state_publisher", "joint_state_publisher"),
            ("sim_env", "fake_localization_node")}.issubset(nodes)
    assert not any(package in ("usv_decision", "usv_control", "usv_perception", "usv_navigation")
                   for package, _ in nodes)
    localization = _parameters(context, nodes[("sim_env", "fake_localization_node")])
    assert localization == {"model_name": "otter", "use_sim_time": True}
    assert (("rviz2", "rviz2") in nodes) is want_rviz
    gazebo, = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    arguments = {_value(context, name): _value(context, setting) for name, setting in gazebo.launch_arguments}
    assert arguments["gui"] == want_gui
    assert arguments["world"] == str(PACKAGE / "worlds/example_course.world")


# 探测到 Windows X 时 GUI 子进程改 DISPLAY；无界面启动不改。
def test_simulation_support_points_gui_at_windows_x_when_reachable(monkeypatch):
    from launch.actions import SetEnvironmentVariable
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "172.27.32.1:0.0")
    context, gui_actions = _launch_graph(monkeypatch, "simulation_support.launch.py", {
        "headless": "false", "gui": "true", "rviz": "true", "world": "example_course"})
    assigned = {
        _value(context, action.name): _value(context, action.value)
        for action in gui_actions if isinstance(action, SetEnvironmentVariable)
    }
    assert assigned["DISPLAY"] == "172.27.32.1:0.0"
    assert assigned["QT_X11_NO_MITSHM"] == "1"
    _, headless_actions = _launch_graph(monkeypatch, "simulation_support.launch.py", {
        "headless": "true", "gui": "true", "rviz": "true", "world": "example_course"})
    assert not any(isinstance(action, SetEnvironmentVariable) for action in headless_actions)


# 自定义 YAML 和启动覆盖在决策、航控入口得到一致有效参数。
@pytest.mark.parametrize("override,expected_planner,expected_controller", [
    ({}, "dijkstra", "lqr"),
    ({"planner_name": "astar", "controller_name": "pid", "trajectory_name": "los"}, "astar", "pid"),
    ({"controller_name": "adrc"}, "dijkstra", "adrc"),
])
def test_custom_project_yaml_and_launch_precedence_match_across_business_entries(monkeypatch, tmp_path, override, expected_planner, expected_controller):
    paths = {}
    for argument, relative in CONFIG_PATHS.items():
        values = yaml.safe_load((PACKAGES["usv_navigation"] / "config" / relative).read_text())
        if argument == "dynamics_config":
            values["dynamics"]["ns"] = "from_yaml"
            values["dynamics"]["cruising_speed"] = 0.65
        elif argument == "controller_config":
            values["controller"]["controller_name"] = "lqr"
            values["controller"]["control_frequncy"] = 12.0
            values["controller"]["pid_controller"]["k_p_speed"] = 3.25
            # Station ADRC is active for every navigation algorithm, so its
            # fixed startup sample period must follow the shared control rate.
            values["controller"]["station_adrc_controller"]["sample_time"] = 1.0 / 12.0
            if expected_controller == "adrc":
                values["controller"].setdefault("adrc_controller", {})["sample_time"] = 1.0 / 12.0
        elif argument == "trajectory_config":
            values["trajectory_planner"]["los"]["lookahead_distance"] = 2.75
        else:
            values["planner_name"] = "dijkstra"
            values["navigation"]["planning_frequency"] = 7.0
        path = tmp_path / (argument + " custom.yaml")
        path.write_text(yaml.safe_dump(values))
        paths[argument] = str(path)
    overrides = dict(paths, robot_namespace="custom_boat", use_sim_time="false", **override)
    resolved = []
    for entry, executable in [("decision", "decision_node"), ("control", "control_node")]:
        context, actions = _launch_graph(monkeypatch, entry + ".launch.py", overrides)
        values = _parameters(context, _nodes(context, actions)[("usv_" + entry, executable)])
        values.pop("watched_topics", None)
        resolved.append(values)
    assert resolved[0] == resolved[1]
    values = resolved[0]
    assert values["planner_name"] == expected_planner
    assert values["usv_simulation.controller.controller_name"] == expected_controller
    assert values["usv_simulation.controller.control_frequency"] == 12.0
    assert values["usv_simulation.controller.pid_controller.k_p_speed"] == 3.25
    if expected_controller == "adrc":
        assert values["usv_simulation.controller.adrc_controller.sample_time"] == pytest.approx(1.0 / 12.0)
    assert values["usv_simulation.trajectory_planner.los.lookahead_distance"] == 2.75
    assert values["navigation.planning_frequency"] == 7.0
    assert values["dynamics.cruising_speed"] == 0.65
    assert values["dynamics.ns"] == "custom_boat"
    assert values["use_sim_time"] is False
    context, actions = _launch_graph(monkeypatch, "main.launch.py", dict(overrides, headless="true"))
    nodes = _nodes(context, actions)
    assert _parameters(context, nodes[("usv_decision", "decision_node")]) == values
    assert _parameters(context, nodes[("usv_control", "control_node")]) == values


# 对应入口解析到非法 YAML 时，不返回可执行的节点动作。
@pytest.mark.parametrize("entry", ["main", "perception", "decision", "control", "simulation_support"])
def test_invalid_project_yaml_fails_before_launch_actions_are_returned(monkeypatch, tmp_path, entry):
    invalid = tmp_path / "invalid controller.yaml"
    invalid.write_text("controller:\n  control_frequncy: 0.0\n")
    with pytest.raises(ValueError, match="must be positive"):
        _launch_graph(monkeypatch, entry + ".launch.py", {"controller_config": str(invalid)})


# 主入口把感知配置路径传给拥有该配置的感知包。
def test_main_forwards_perception_config_paths_to_owning_package(monkeypatch, tmp_path):
    source = PACKAGES["usv_perception"] / "config"
    mapping = yaml.safe_load((source / "costmap/mapping.yaml").read_text())
    costmap = yaml.safe_load((source / "robots/otter/global_costmap_params_otter.yaml").read_text())
    mapping["resolution"] = 0.7
    costmap["costmap"]["update_frequency"] = 4.0
    mapping_path, costmap_path = tmp_path / "custom mapping.yaml", tmp_path / "custom costmap.yaml"
    mapping_path.write_text(yaml.safe_dump(mapping))
    costmap_path.write_text(yaml.safe_dump(costmap))
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {
        "headless": "true", "mapping_config": str(mapping_path), "costmap_config": str(costmap_path)})
    nodes = _nodes(context, actions)
    assert _parameters(context, nodes[("usv_perception", "perception_mapping_node")])["resolution"] == 0.7
    assert _parameters(context, nodes[("usv_perception", "costmap_support_node")])["update_frequency"] == 4.0


# 空命名空间在所有入口中均被拒绝，防止话题和坐标系身份含糊。
@pytest.mark.parametrize("entry", ["main", "perception", "decision", "control", "simulation_support"])
def test_all_entries_reject_empty_namespace_before_starting_nodes(monkeypatch, entry):
    with pytest.raises(ValueError, match="robot_namespace must be nonempty"):
        _launch_graph(monkeypatch, entry + ".launch.py", {"robot_namespace": "///"})
