# 独立感知启动支持：读取本包建图和代价地图参数，组合建图、Nav2 地图和进程监控三个节点。
"""LiDAR perception launch configuration, independent of planning and control."""
import os
from pathlib import Path
import sys

import yaml


# 安全读取工程 YAML，拒绝重复键、非字典根以及 ROS 参数导出快照。
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


# 在 launch 上下文中取得最终参数字符串。
def _value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)


# 验证命名空间和图层配置，再将统一坐标系、扫描话题及出生偏移传入节点。
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
    # plugins 才是实际装载清单；未列出的 static_layer 段不会启用，显式启用静态层则拒绝启动。
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
    logs = []
    # LaDDS 联调时把 0141 障碍栅格作为最前面的静态层；它只订接收节点的栅格，不是 /map 累积地图。
    if _value(context, "ladds_obstacles").lower() == "true":
        layer = costmap.get("ladds_obstacle_layer")
        if not isinstance(layer, dict) or not str(layer.get("plugin", "")).endswith("StaticLayer"):
            raise ValueError("ladds_obstacles requires a StaticLayer section named ladds_obstacle_layer")
        grid_topic = "/" + namespace + "/ladds/obstacle_grid"
        layer.update(enabled=True, map_topic=grid_topic)
        # A static layer overwrites the cells it covers, so it must precede the LiDAR layer.
        costmap["plugins"] = ["ladds_obstacle_layer", *plugins]
        logs.append(LogInfo(msg=f"[usv_perception] 0141 obstacle layer: {grid_topic}"))
    return [
        LogInfo(msg=f"[usv_perception] mapping_config: {mapping_path}"),
        LogInfo(msg=f"[usv_perception] costmap_config: {costmap_path}"),
        *logs,
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


# 声明感知所需的配置路径与时钟/命名空间参数，实际节点由 OpaqueFunction 延迟创建。
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
        DeclareLaunchArgument("ladds_obstacles", default_value="false", choices=["true", "false"],
                              description="Prepend the LaDDS 0141 obstacle grid as a static costmap layer"),
        DeclareLaunchArgument("mapping_config", default_value=os.path.join(share, "config", "costmap", "mapping.yaml"),
                              description="Project mapping YAML path (not a ros2 param dump)"),
        DeclareLaunchArgument("costmap_config", default_value=os.path.join(
            share, "config", "robots", "otter", "global_costmap_params_otter.yaml"),
                              description="Project costmap YAML path (not a ros2 param dump)"),
        OpaqueFunction(function=_launch_setup),
    ])
