"""The task application owns its launch graph and requires explicit geodesy."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

from usv_mission.launch_support import geographic_parameters


PACKAGE = Path(__file__).resolve().parents[1]
SIM_SHARE = PACKAGE.parents[2] / "sim_env"
NAVIGATION_SHARE = PACKAGE.parents[1] / "shared/usv_navigation"
PERCEPTION_SHARE = PACKAGE.parents[1] / "perception_module/usv_perception"


def test_missing_reference_does_not_enable_zero_origin():
    assert geographic_parameters() == {"mission.geo.enabled": False}


def test_explicit_world_supplies_actual_origin():
    parameters = geographic_parameters(world_path=str(SIM_SHARE / "worlds/ocean.world"))
    assert parameters == {"mission.geo.enabled": True, "mission.geo.latitude": -33.7242,
                          "mission.geo.longitude": 150.68, "mission.geo.altitude": 0.0}


def test_explicit_zero_origin_is_valid_and_different_from_missing_reference(tmp_path):
    path = tmp_path / "reference.yaml"
    path.write_text("latitude: 0.0\nlongitude: 0.0\naltitude: 2.0\n")
    assert geographic_parameters(str(path)) == {
        "mission.geo.enabled": True, "mission.geo.latitude": 0.0,
        "mission.geo.longitude": 0.0, "mission.geo.altitude": 2.0}


@pytest.mark.parametrize("contents", [
    "latitude: 1.0\n", "latitude: 91.0\nlongitude: 1.0\n",
    "latitude: true\nlongitude: 1.0\n", "latitude: .nan\nlongitude: 1.0\n",
    "latitude: 1.0\nlongitude: 1.0\nheading: 0.0\n",
    "latitude: 1.0\nlatitude: 2.0\nlongitude: 1.0\n",
])
def test_invalid_explicit_reference_is_rejected(tmp_path, contents):
    path = tmp_path / "reference.yaml"
    path.write_text(contents)
    with pytest.raises(ValueError):
        geographic_parameters(str(path))


def test_ambiguous_reference_sources_are_rejected():
    with pytest.raises(ValueError, match="not both"):
        geographic_parameters("reference.yaml", "ocean.world")


def test_missing_explicit_world_fails_closed(tmp_path):
    with pytest.raises(ValueError, match="Cannot read world"):
        geographic_parameters(world_path=str(tmp_path / "missing.world"))


def test_package_declares_runtime_and_launch_dependencies():
    tree = ET.parse(PACKAGE / "package.xml")
    assert tree.findtext("name") == "usv_mission"
    assert tree.findtext("export/build_type") == "ament_python"
    dependencies = {node.text for node in tree.findall("exec_depend")}
    assert {"rclpy", "tf2_ros", "sensor_msgs", "nav2_msgs", "usv_mission_interfaces",
            "controller", "trajectory_planner", "path_planner", "common",
            "sim_env", "usv_navigation", "usv_perception", "launch", "launch_ros"}.issubset(dependencies)


def _launch_graph(monkeypatch, filename, overrides=None):
    pytest.importorskip("launch")
    pytest.importorskip("launch_ros")
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    original_share = ament_index_python.packages.get_package_share_directory

    def share(package):
        if package == "sim_env":
            return str(SIM_SHARE)
        if package == "usv_mission":
            return str(PACKAGE)
        if package == "usv_navigation":
            return str(NAVIGATION_SHARE)
        if package == "usv_perception":
            return str(PERCEPTION_SHARE)
        return original_share(package)

    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory", share)
    spec = importlib.util.spec_from_file_location("mission_launch_under_test", PACKAGE / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    actions = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            actions.extend(action.execute(context))
        else:
            actions.append(action)
    return context, actions


def _nodes(context, actions):
    from launch_ros.actions import Node
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    def value(raw):
        return perform_substitutions(context, normalize_to_list_of_substitutions(raw))

    return [(value(node.node_package), value(node.node_executable))
            for node in actions if isinstance(node, Node)]


def test_standalone_launch_starts_only_mission_node(monkeypatch):
    context, actions = _launch_graph(monkeypatch, "mission.launch.py")
    assert _nodes(context, actions) == [("usv_mission", "mission_node")]
    from launch.actions import IncludeLaunchDescription
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)


def test_simulation_launch_has_one_task_controller_and_public_support_nodes(monkeypatch):
    # Simulation resources are sufficient; task launch must not import its runtime.
    for name in list(sys.modules):
        if name.startswith("sim_env."):
            monkeypatch.delitem(sys.modules, name)
    monkeypatch.setitem(sys.modules, "sim_env", None)
    context, actions = _launch_graph(monkeypatch, "mission_sim.launch.py", {
        "robot_namespace": "boat_two", "headless": "true"})
    nodes = _nodes(context, actions)
    assert nodes.count(("usv_mission", "mission_node")) == 1
    assert ("sim_env", "sim_env_node") not in nodes
    assert {("sim_env", "fake_localization_node"), ("usv_perception", "costmap_support_node"),
            ("gazebo_ros", "spawn_entity.py")}.issubset(nodes)
    assert ("usv_perception", "perception_mapping_node") in nodes
    assert not any(package == "rviz2" for package, _ in nodes)
    from launch.actions import IncludeLaunchDescription
    assert sum(isinstance(action, IncludeLaunchDescription) for action in actions) == 1


def test_standalone_launch_rejects_invalid_origin_before_starting_node(monkeypatch, tmp_path):
    invalid = tmp_path / "origin.yaml"
    invalid.write_text("latitude: 100\nlongitude: 0\n")
    with pytest.raises(ValueError):
        _launch_graph(monkeypatch, "mission.launch.py", {"geo_reference": str(invalid)})
