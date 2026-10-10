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
    assert parameters["usv_simulation.trajectory_planner.los.lookahead_distance"] == 3.0


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


# 记录 simulation_support 构造的 xacro 命令行，不实际展开船模。
def _record_xacro_commands(monkeypatch):
    import launch.substitutions
    commands = []

    class RecordingCommand(launch.substitutions.Command):
        def __init__(self, command, **kwargs):
            commands.append("".join(command))
            super().__init__(command, **kwargs)

    monkeypatch.setattr(launch.substitutions, "Command", RecordingCommand)
    return commands


# 船型切换仿真船模、默认的船模/控制器/代价地图 YAML，并把命名空间默认值改成同名的 wamv；决策与航控仍得到同一份参数。
def test_robot_type_selects_wamv_model_namespace_and_default_configuration(monkeypatch):
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    commands = _record_xacro_commands(monkeypatch)
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {"headless": "true", "robot_type": "wamv"})
    nodes = _nodes(context, actions)
    # 船模 YAML 为两台全回转桨，仿真用双艉推的 H 布局。
    assert commands == ["xacro " + str(PACKAGE / "urdf/wamv/wamv.urdf.xacro") + " namespace:=wamv"
                        " imu_enabled:=false gps_enabled:=false lidar_enabled:=true thruster_config:=H"]
    for key in (("usv_decision", "decision_node"), ("usv_control", "control_node"),
                ("usv_perception", "perception_mapping_node"), ("usv_perception", "costmap_support_node"),
                ("usv_perception", "perception_monitor"), ("sim_env", "fake_localization_node")):
        assert _value(context, nodes[key]._Node__node_namespace) == "wamv"
    decision = _parameters(context, nodes[("usv_decision", "decision_node")])
    assert decision == _parameters(context, nodes[("usv_control", "control_node")])
    assert (decision["dynamics.name"], decision["dynamics.type"]) == ("wamv", "Azimuth")
    assert (decision["dynamics.max_force"], decision["dynamics.max_lateral_force"]) == (250.0, 200.0)
    assert decision["dynamics.thruster_x"] == -2.374
    assert decision["usv_simulation.controller.lateral_controller.enabled"] is True
    assert (decision["navigation.footprint_length"], decision["navigation.footprint_width"]) == (5.7, 2.5)
    assert decision["dynamics.ns"] == "wamv"
    costmap = _parameters(context, nodes[("usv_perception", "costmap_support_node")])
    assert costmap["robot_radius"] == 3.2 and costmap["inflation_layer.inflation_radius"] == 4.0
    assert (costmap["global_frame"], costmap["robot_base_frame"]) == ("wamv/map", "wamv/base_link")
    assert costmap["obstacle_layer.scan.topic"] == "/wamv/sensors/lidars/lidar/scan"
    # Gazebo 实体名与真值定位读取的模型名都跟随命名空间。
    localization = _parameters(context, nodes[("sim_env", "fake_localization_node")])
    assert localization == {"model_name": "wamv", "use_sim_time": True}
    spawn = [_value(context, argument) for argument in nodes[("gazebo_ros", "spawn_entity.py")]._Node__arguments]
    assert spawn[spawn.index("-entity") + 1] == "wamv"


# 仿真推进器布局跟随船模 YAML 的推进形式：固定桨差动和全回转用双艉推（H），独立侧推器用 T 型。
@pytest.mark.parametrize("propulsion, lateral_force, layout", [
    ("Differential", 0.0, "H"), ("Azimuth", 200.0, "H"), ("Omni", 250.0, "T")])
def test_wamv_thruster_layout_follows_vessel_propulsion_type(monkeypatch, tmp_path, propulsion, lateral_force, layout):
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    config = PACKAGES["usv_navigation"] / "config" / "robots" / "wamv"
    dynamics = yaml.safe_load((config / "dynamics_wamv.yaml").read_text(encoding="utf-8"))
    dynamics["dynamics"].update(type=propulsion, max_lateral_force=lateral_force)
    controller = yaml.safe_load((config / "controller_params_wamv.yaml").read_text(encoding="utf-8"))
    # 不能产生横向力的推进形式不得启用横向环。
    controller["controller"]["lateral_controller"]["enabled"] = lateral_force > 0
    paths = {"dynamics_config": tmp_path / "dynamics.yaml", "controller_config": tmp_path / "controller.yaml"}
    paths["dynamics_config"].write_text(yaml.safe_dump(dynamics))
    paths["controller_config"].write_text(yaml.safe_dump(controller))
    commands = _record_xacro_commands(monkeypatch)
    _launch_graph(monkeypatch, "simulation_support.launch.py", {
        "headless": "true", "robot_type": "wamv", **{name: str(path) for name, path in paths.items()}})
    assert commands and commands[0].endswith(" lidar_enabled:=true thruster_config:=" + layout)


# 显式给出的命名空间优先于船型默认值：WAM-V 也可以放在别的命名空间下。
def test_explicit_namespace_overrides_robot_type_default(monkeypatch):
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {
        "headless": "true", "robot_type": "wamv", "robot_namespace": "boat_two"})
    nodes = _nodes(context, actions)
    control = _parameters(context, nodes[("usv_control", "control_node")])
    assert (control["dynamics.name"], control["dynamics.ns"]) == ("wamv", "boat_two")
    assert _value(context, nodes[("usv_control", "control_node")]._Node__node_namespace) == "boat_two"
    assert _parameters(context, nodes[("usv_perception", "costmap_support_node")])["global_frame"] == "boat_two/map"


# 非 otter 命名空间启动 RViz 时使用替换过前缀的配置副本；otter 仍直接用随包安装的文件。
def test_rviz_configuration_follows_namespace(monkeypatch, tmp_path):
    import tempfile
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    monkeypatch.setattr(tempfile, "gettempdir", lambda: str(tmp_path))

    def rviz_config(overrides):
        context, actions = _launch_graph(monkeypatch, "simulation_support.launch.py", {
            "headless": "false", "gui": "false", "rviz": "true", **overrides})
        node = _nodes(context, actions)[("rviz2", "rviz2")]
        arguments = [_value(context, argument) for argument in node._Node__arguments]
        return Path(arguments[arguments.index("-d") + 1])

    assert rviz_config({}) == PACKAGE / "rviz/sim_env.rviz"
    generated = rviz_config({"robot_type": "wamv"})
    assert generated.parent == tmp_path
    text = generated.read_text(encoding="utf-8")
    assert "Fixed Frame: wamv/map" in text and "TF Prefix: wamv" in text and "/wamv/global_path" in text
    assert "otter" not in text
    # 无界面启动不生成副本。
    generated.unlink()
    _launch_graph(monkeypatch, "simulation_support.launch.py", {"headless": "true", "robot_type": "wamv"})
    assert not generated.exists()


# 默认船型仍是 Otter：xacro 命令行、差动船模和关闭的横向环都与原行为一致。
def test_default_robot_type_keeps_otter_model_and_configuration(monkeypatch):
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    commands = _record_xacro_commands(monkeypatch)
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {"headless": "true"})
    nodes = _nodes(context, actions)
    assert commands == ["xacro " + str(PACKAGE / "urdf/otter/otter.urdf.xacro") + " namespace:=otter"
                        " imu_enabled:=false gps_enabled:=false lidar_enabled:=true"]
    control = _parameters(context, nodes[("usv_control", "control_node")])
    assert _value(context, nodes[("usv_control", "control_node")]._Node__node_namespace) == "otter"
    assert (control["dynamics.name"], control["dynamics.type"], control["dynamics.ns"]) == ("otter", "Differential", "otter")
    assert control["dynamics.max_lateral_force"] == 0.0
    assert control["usv_simulation.controller.lateral_controller.enabled"] is False
    assert _parameters(context, nodes[("usv_perception", "costmap_support_node")])["robot_radius"] == 1.3


# 显式给出的 YAML 路径优先于船型默认值：WAM-V 船模配 Otter 控制器 YAML 时，±537 rpm 不覆盖 250 N，启动前即报错。
# 独立感知入口同样按船型选择代价地图。
def test_explicit_config_path_overrides_robot_type_default(monkeypatch):
    otter = PACKAGES["usv_navigation"] / "config" / CONFIG_PATHS["controller_config"]
    with pytest.raises(ValueError, match="rpm limits reach only 180"):
        _launch_graph(monkeypatch, "control.launch.py", {"robot_type": "wamv", "controller_config": str(otter)})
    context, actions = _launch_graph(monkeypatch, "perception.launch.py", {"robot_type": "wamv"})
    costmap = _parameters(context, _nodes(context, actions)[("usv_perception", "costmap_support_node")])
    assert costmap["robot_radius"] == 3.2


# 写一份只含场景和船型的用户场景配置，并让各入口通过环境变量读到它。
def _user_config(monkeypatch, tmp_path, robot_type, world="ocean"):
    path = tmp_path / "user_config.yaml"
    path.write_text(yaml.safe_dump({"world": world, "robots_config": [{"robot1_type": robot_type}]}))
    monkeypatch.setenv("USV_USER_CONFIG", str(path))
    return path


# user_config.yaml 的 robot1_type 决定默认船型：主入口和三个独立业务入口都随之切到 WAM-V 及其命名空间，world 取自同一文件。
def test_user_config_selects_default_vessel_for_every_entry(monkeypatch, tmp_path):
    monkeypatch.setattr("sim_env.windows_display.windows_x_display", lambda: "")
    path = _user_config(monkeypatch, tmp_path, "wamv", world="example_course")
    commands = _record_xacro_commands(monkeypatch)
    context, actions = _launch_graph(monkeypatch, "main.launch.py", {"headless": "true"})
    nodes = _nodes(context, actions)
    assert context.launch_configurations["robot_type"] == "wamv"
    assert commands and "urdf/wamv/wamv.urdf.xacro namespace:=wamv" in commands[0]
    control = _parameters(context, nodes[("usv_control", "control_node")])
    assert (control["dynamics.name"], control["dynamics.ns"]) == ("wamv", "wamv")
    assert _parameters(context, nodes[("usv_perception", "costmap_support_node")])["global_frame"] == "wamv/map"
    from launch.actions import IncludeLaunchDescription, LogInfo
    gazebo, = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    arguments = {_value(context, name): _value(context, setting) for name, setting in gazebo.launch_arguments}
    assert arguments["world"] == str(PACKAGE / "worlds/example_course.world")
    # 启动日志写明船型和所用的配置文件。
    logs = [_value(context, action.msg) for action in actions if isinstance(action, LogInfo)]
    assert f"[usv_navigation] robot_type: wamv (user config: {path})" in logs
    for entry, key in (("decision", ("usv_decision", "decision_node")), ("control", ("usv_control", "control_node")),
                       ("perception", ("usv_perception", "costmap_support_node"))):
        context, actions = _launch_graph(monkeypatch, entry + ".launch.py")
        assert _value(context, _nodes(context, actions)[key]._Node__node_namespace) == "wamv"


# 命令行的 robot_type:= 优先于 user_config.yaml。
def test_launch_argument_overrides_user_config_vessel(monkeypatch, tmp_path):
    _user_config(monkeypatch, tmp_path, "wamv")
    context, actions = _launch_graph(monkeypatch, "control.launch.py", {"robot_type": "otter"})
    control = _parameters(context, _nodes(context, actions)[("usv_control", "control_node")])
    assert (control["dynamics.name"], control["dynamics.ns"]) == ("otter", "otter")


# user_config.yaml 里写了未登记的船型时，各入口在返回任何节点动作之前报错并指出文件。
@pytest.mark.parametrize("entry", ["main", "perception", "decision", "control", "simulation_support"])
def test_unknown_vessel_in_user_config_fails_before_launch_actions(monkeypatch, tmp_path, entry):
    _user_config(monkeypatch, tmp_path, "catamaran")
    with pytest.raises(ValueError, match="user_config.yaml: robot1_type must be one of otter, wamv"):
        _launch_graph(monkeypatch, entry + ".launch.py")


# 未登记的船型在声明参数时即被拒绝。
@pytest.mark.parametrize("entry", ["main", "perception", "decision", "control", "simulation_support"])
def test_all_entries_reject_unknown_robot_type(monkeypatch, entry):
    with pytest.raises(RuntimeError, match="robot_type"):
        _launch_graph(monkeypatch, entry + ".launch.py", {"robot_type": "catamaran"})


# 空命名空间在所有入口中均被拒绝，防止话题和坐标系身份含糊。
@pytest.mark.parametrize("entry", ["main", "perception", "decision", "control", "simulation_support"])
def test_all_entries_reject_empty_namespace_before_starting_nodes(monkeypatch, entry):
    with pytest.raises(ValueError, match="robot_namespace must be nonempty"):
        _launch_graph(monkeypatch, entry + ".launch.py", {"robot_namespace": "///"})
