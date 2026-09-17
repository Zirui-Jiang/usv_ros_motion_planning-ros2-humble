"""LiDAR perception launch configuration, independent of planning and control."""
import os
from pathlib import Path
import sys

import yaml


def _load_yaml(path):
    class UniqueKeyLoader(yaml.SafeLoader):
        pass

    def unique_mapping(loader, node, deep=False):
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise ValueError(f"Duplicate YAML key {key!r} in {path}")
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping

    UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        result = yaml.load(stream, Loader=UniqueKeyLoader)
    if not isinstance(result, dict):
        raise ValueError(f"Expected project YAML mapping: {path}")
    if any(isinstance(value, dict) and "ros__parameters" in value for value in result.values()):
        raise ValueError("ROS parameter dumps are snapshots, not project configuration files")
    return result


def _value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    from launch.actions import LogInfo
    from launch_ros.actions import Node

    namespace = _value(context, "robot_namespace").strip("/")
    if not namespace:
        raise ValueError("robot_namespace must be nonempty")
    clock = _value(context, "use_sim_time").lower()
    if clock not in ("true", "false"):
        raise ValueError("use_sim_time must be true or false")
    use_sim_time = clock == "true"
    mapping_path = os.path.abspath(os.path.expanduser(_value(context, "mapping_config")))
    costmap_path = os.path.abspath(os.path.expanduser(_value(context, "costmap_config")))
    mapping = _load_yaml(mapping_path)
    costmap = _load_yaml(costmap_path)["costmap"]
    plugins = costmap.get("plugins")
    if not isinstance(plugins, list) or not plugins or not all(isinstance(name, str) for name in plugins):
        raise ValueError("Costmap YAML must provide an explicit nonempty plugins list")
    for name in plugins:
        config = costmap.get(name, {})
        is_static = name == "static_layer" or str(config.get("plugin", "")).endswith("StaticLayer")
        if is_static and config.get("enabled", True):
            raise ValueError("This launch requires static_layer to remain disabled; costmap YAML enables it")
    costmap.update(global_frame=namespace + "/map", robot_base_frame=namespace + "/base_link",
                   use_sim_time=use_sim_time)
    costmap["obstacle_layer"]["scan"].update(
        topic="/" + namespace + "/sensors/lidars/lidar/scan", sensor_frame=namespace + "/lidar_link")
    return [
        LogInfo(msg=f"[usv_perception] mapping_config: {mapping_path}"),
        LogInfo(msg=f"[usv_perception] costmap_config: {costmap_path}"),
        Node(package="usv_perception", executable="perception_mapping_node", name="mapping",
             prefix=[sys.executable], namespace=namespace, output="screen",
             remappings=[("scan", "sensors/lidars/lidar/scan")],
             parameters=[mapping, {"offset_x": float(_value(context, "robot_x")),
                                   "offset_y": float(_value(context, "robot_y")), "use_sim_time": use_sim_time}]),
        Node(package="usv_perception", executable="costmap_support_node", name="costmap",
             namespace=namespace, output="screen", parameters=[costmap],
             remappings=[(name, "/" + namespace + "/" + name)
                         for name in ("costmap_raw", "costmap", "costmap_updates")]),
        Node(package="usv_perception", executable="perception_monitor", name="usv_perception",
             prefix=[sys.executable], namespace=namespace, output="screen",
             parameters=[{"watched_topics": ["odom", "costmap_raw", "map"], "use_sim_time": use_sim_time}]),
    ]


def launch_description():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    share = get_package_share_directory("usv_perception")
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="otter"),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("robot_x", default_value="23.0"),
        DeclareLaunchArgument("robot_y", default_value="-10.0"),
        DeclareLaunchArgument("mapping_config", default_value=os.path.join(share, "config", "costmap", "mapping.yaml"),
                              description="Project mapping YAML path (not a ros2 param dump)"),
        DeclareLaunchArgument("costmap_config", default_value=os.path.join(
            share, "config", "robots", "otter", "global_costmap_params_otter.yaml"),
                              description="Project costmap YAML path (not a ros2 param dump)"),
        OpaqueFunction(function=_launch_setup),
    ])
