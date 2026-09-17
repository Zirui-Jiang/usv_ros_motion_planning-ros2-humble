"""Exercise the independent perception launch without starting subprocesses."""
import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

PACKAGE = Path(__file__).resolve().parents[1]


def _launch_graph(monkeypatch, overrides=None):
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    def share(name):
        assert name == "usv_perception", "The standalone perception launch must use its own package resources"
        return str(PACKAGE)

    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory", share)
    path = PACKAGE / "launch/perception.launch.py"
    assert path.is_file(), "Independent perception launch entry is missing"
    spec = importlib.util.spec_from_file_location("perception_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    actions = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            actions.extend(action.execute(context) or [])
        else:
            actions.append(action)
    return context, actions


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


def test_standalone_perception_starts_only_its_three_business_nodes(monkeypatch):
    from launch.actions import IncludeLaunchDescription
    context, actions = _launch_graph(monkeypatch)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)
    nodes = _nodes(context, actions)
    assert set(nodes) == {
        ("usv_perception", "perception_mapping_node"),
        ("usv_perception", "costmap_support_node"),
        ("usv_perception", "perception_monitor"),
    }
    for executable, name in (("perception_mapping_node", "mapping"),
                             ("costmap_support_node", "costmap"),
                             ("perception_monitor", "usv_perception")):
        node = nodes[("usv_perception", executable)]
        assert _value(context, node._Node__node_name) == name
        assert _value(context, node._Node__node_namespace) == "otter"
        assert _parameters(context, node)["use_sim_time"] is True
    mapping = _parameters(context, nodes[("usv_perception", "perception_mapping_node")])
    assert (mapping["width"], mapping["resolution"], mapping["offset_x"], mapping["offset_y"]) == (
        1024, 0.5, 23.0, -10.0)
    monitor = _parameters(context, nodes[("usv_perception", "perception_monitor")])
    assert set(monitor) == {"watched_topics", "use_sim_time"}
    assert list(monitor["watched_topics"]) == ["odom", "costmap_raw", "map"]


@pytest.mark.parametrize("executable", ["perception_mapping_node", "perception_monitor"])
def test_python_nodes_start_with_incompatible_python_on_path(monkeypatch, tmp_path, executable):
    """A Conda-first PATH must not select a different Python for ROS wrappers."""
    import os
    import subprocess
    import sys
    from launch.actions import ExecuteProcess

    context, actions = _launch_graph(monkeypatch)
    node = _nodes(context, actions)[("usv_perception", executable)]
    other_python = tmp_path / "python3"
    other_python.write_text("#!/bin/sh\necho incompatible-python >&2\nexit 86\n")
    other_python.chmod(0o755)
    wrapper = tmp_path / "node_entry"
    wrapper.write_text("#!/usr/bin/env python3\nimport sys\nprint(sys.executable)\n")
    wrapper.chmod(0o755)
    context.environment["PATH"] = str(tmp_path) + os.pathsep + context.environment["PATH"]
    # Resolve the real node's launch prefix and execute a lightweight wrapper
    # with the same shebang as the installed node, without starting ROS.
    probe = ExecuteProcess(cmd=[str(wrapper)], prefix=node.process_description.prefix)
    probe.process_description.prepare(context, probe)
    result = subprocess.run(probe.process_description.final_cmd,
                            env=probe.process_description.final_env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == sys.executable


def test_custom_yaml_and_namespace_reach_lidar_mapping_and_costmap(monkeypatch, tmp_path):
    mapping = yaml.safe_load((PACKAGE / "config/costmap/mapping.yaml").read_text())
    mapping.update(width=256, resolution=0.25, tf_wait_timeout=0.15, offset_x=-8.0, offset_y=-9.0)
    mapping_path = tmp_path / "custom mapping.yaml"
    mapping_path.write_text(yaml.safe_dump(mapping))
    costmap = yaml.safe_load((PACKAGE / "config/robots/otter/global_costmap_params_otter.yaml").read_text())
    costmap["costmap"]["update_frequency"] = 3.0
    costmap_path = tmp_path / "custom costmap.yaml"
    costmap_path.write_text(yaml.safe_dump(costmap))
    context, actions = _launch_graph(monkeypatch, {
        "mapping_config": str(mapping_path), "costmap_config": str(costmap_path),
        "robot_namespace": "/boat_two/", "use_sim_time": "false",
        "robot_x": "41.0", "robot_y": "-17.0",
    })
    nodes = _nodes(context, actions)
    for node in nodes.values():
        assert _value(context, node._Node__node_namespace) == "boat_two"
        assert _parameters(context, node)["use_sim_time"] is False
    mapper = nodes[("usv_perception", "perception_mapping_node")]
    mapping = _parameters(context, mapper)
    assert (mapping["width"], mapping["resolution"], mapping["tf_wait_timeout"]) == (256, 0.25, 0.15)
    assert (mapping["offset_x"], mapping["offset_y"]) == (41.0, -17.0)
    assert [(_value(context, source), _value(context, target))
            for source, target in mapper._Node__remappings] == [("scan", "sensors/lidars/lidar/scan")]
    costmap_node = nodes[("usv_perception", "costmap_support_node")]
    costmap = _parameters(context, costmap_node)
    assert costmap["update_frequency"] == 3.0
    assert (costmap["global_frame"], costmap["robot_base_frame"]) == ("boat_two/map", "boat_two/base_link")
    assert list(costmap["plugins"]) == ["obstacle_layer", "inflation_layer"]
    assert costmap["obstacle_layer.scan.topic"] == "/boat_two/sensors/lidars/lidar/scan"
    assert costmap["obstacle_layer.scan.sensor_frame"] == "boat_two/lidar_link"
    assert [(_value(context, source), _value(context, target))
            for source, target in costmap_node._Node__remappings] == [
        ("costmap_raw", "/boat_two/costmap_raw"), ("costmap", "/boat_two/costmap"),
        ("costmap_updates", "/boat_two/costmap_updates"),
    ]


def test_empty_namespace_fails_before_returning_nodes(monkeypatch):
    with pytest.raises(ValueError, match="robot_namespace must be nonempty"):
        _launch_graph(monkeypatch, {"robot_namespace": "///"})


@pytest.mark.parametrize("plugins", [None, [], "obstacle_layer"])
def test_costmap_requires_explicit_nonempty_plugin_list(monkeypatch, tmp_path, plugins):
    values = yaml.safe_load((PACKAGE / "config/robots/otter/global_costmap_params_otter.yaml").read_text())
    values["costmap"]["plugins"] = plugins
    path = tmp_path / "invalid costmap.yaml"
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="explicit nonempty plugins list"):
        _launch_graph(monkeypatch, {"costmap_config": str(path)})


def test_costmap_cannot_enable_static_layer(monkeypatch, tmp_path):
    values = yaml.safe_load((PACKAGE / "config/robots/otter/global_costmap_params_otter.yaml").read_text())
    values["costmap"]["plugins"].insert(0, "static_layer")
    path = tmp_path / "static costmap.yaml"
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="static_layer to remain disabled"):
        _launch_graph(monkeypatch, {"costmap_config": str(path)})


@pytest.mark.parametrize("argument,text", [
    ("mapping_config", "width: 8\nwidth: 16\n"),
    ("costmap_config", "costmap:\n  ros__parameters:\n    resolution: 0.5\n"),
])
def test_config_rejects_ambiguous_yaml_or_parameter_dump(monkeypatch, tmp_path, argument, text):
    path = tmp_path / "invalid project.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match="Duplicate YAML key|ROS parameter dumps"):
        _launch_graph(monkeypatch, {argument: str(path)})
