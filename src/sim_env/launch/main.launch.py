#!/usr/bin/env python3
"""Gazebo, LiDAR mapping, C++ ROS support and Python navigation launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from sim_env.configuration import load_project_parameters, load_yaml, resolve_parameters


def load_user_config():
    """Retain the original optional per-user world selection."""
    path = os.path.join(os.path.expanduser("~"), ".rmp_user_config.yaml")
    return load_yaml(path) if os.path.isfile(path) else {}


def _launch_setup(context):
    """Resolve paths and typed parameters after launch overrides are available."""
    share = get_package_share_directory("sim_env")

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return text == "true"

    namespace = value("robot_namespace").strip("/")
    if not namespace:
        raise ValueError("robot_namespace must be nonempty")
    prefix = "/" + namespace
    use_sim_time = boolean("use_sim_time")
    headless = boolean("headless")
    gazebo_gui = boolean("gui") and not headless
    show_rviz = boolean("rviz") and not headless
    pose = {axis: float(value("robot_" + axis)) for axis in ("x", "y", "z", "yaw")}

    config_paths = {
        name: os.path.abspath(os.path.expanduser(value(name)))
        for name in ("dynamics_config", "controller_config", "trajectory_config", "planner_config")
    }
    navigation_params = load_project_parameters(*config_paths.values())
    configuration_logs = [LogInfo(msg=f"[sim_env] {name}: {path}")
                          for name, path in config_paths.items()]
    configuration_logs.append(LogInfo(
        msg="[sim_env] graph_planner_params.yaml is an inactive template and is not loaded."))
    # Empty launch overrides preserve explicit choices from project YAML.
    for argument, parameter in (
        ("planner_name", "planner_name"),
        ("controller_name", "usv_simulation.controller.controller_name"),
        ("trajectory_name", "usv_simulation.trajectory_planner.planner_name"),
    ):
        if value(argument):
            navigation_params[parameter] = value(argument)
    navigation_params["dynamics.ns"] = namespace
    navigation_params = resolve_parameters(navigation_params)

    robot_description = Command([
        "xacro ", os.path.join(share, "urdf", "otter", "otter.urdf.xacro"),
        " namespace:=", namespace, " imu_enabled:=false gps_enabled:=false",
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": use_sim_time, "frame_prefix": namespace + "/",
        }], output="screen")
    joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}], output="screen")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": os.path.join(share, "worlds", value("world") + ".world"),
            "gui": "true" if gazebo_gui else "false", "verbose": "false",
        }.items())
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", namespace, "-topic", "robot_description",
                   "-x", str(pose["x"]), "-y", str(pose["y"]),
                   "-z", str(pose["z"]), "-Y", str(pose["yaw"])],
        output="screen")

    mapping_params = load_yaml(os.path.join(share, "config", "costmap", "mapping.yaml"))
    mapping = Node(
        package="perception", executable="perception_mapping_node", name="mapping",
        namespace=namespace, output="screen",
        remappings=[("scan", "sensors/lidars/lidar/scan")],
        parameters=[mapping_params, {"offset_x": pose["x"], "offset_y": pose["y"],
                                     "use_sim_time": use_sim_time}])
    fake_localization = Node(
        package="sim_env", executable="fake_localization_node", name="fake_localization",
        namespace=namespace, output="screen",
        parameters=[{"model_name": namespace, "use_sim_time": use_sim_time}])

    # Extract the existing wrapper for the independent Costmap2DROS lifecycle
    # node. The live-LiDAR policy is retained: static_layer stays disabled.
    costmap_params = load_yaml(os.path.join(
        share, "config", "robots", "otter", "global_costmap_params_otter.yaml"))["costmap"]
    plugins = costmap_params.get("plugins")
    if not isinstance(plugins, list) or not plugins or not all(isinstance(name, str) for name in plugins):
        raise ValueError("Costmap YAML must provide an explicit nonempty plugins list")
    for plugin_name in plugins:
        plugin_config = costmap_params.get(plugin_name, {})
        is_static = plugin_name == "static_layer" or str(plugin_config.get("plugin", "")).endswith("StaticLayer")
        if is_static and plugin_config.get("enabled", True):
            raise ValueError("This launch requires static_layer to remain disabled; costmap YAML enables it")
    costmap_params["global_frame"] = namespace + "/map"
    costmap_params["robot_base_frame"] = namespace + "/base_link"
    costmap_params["use_sim_time"] = use_sim_time
    scan = costmap_params["obstacle_layer"]["scan"]
    scan["topic"] = prefix + "/sensors/lidars/lidar/scan"
    scan["sensor_frame"] = namespace + "/lidar_link"
    costmap = Node(
        package="sim_env", executable="costmap_support_node", name="costmap",
        namespace=namespace, output="screen", parameters=[costmap_params],
        remappings=[("costmap_raw", prefix + "/costmap_raw"),
                    ("costmap", prefix + "/costmap"),
                    ("costmap_updates", prefix + "/costmap_updates")])

    navigation = Node(
        package="sim_env", executable="sim_env_node", name="usv_simulation",
        namespace=namespace, output="screen",
        parameters=[navigation_params, {"use_sim_time": use_sim_time}])
    actions = [*configuration_logs, gazebo, robot_state_publisher, joint_state_publisher, spawn,
               fake_localization, mapping, costmap, navigation]
    if show_rviz:
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "sim_env.rviz")],
            parameters=[{"use_sim_time": use_sim_time}], output="screen"))
    return actions


def generate_launch_description():
    share = get_package_share_directory("sim_env")
    user_cfg = load_user_config()
    arguments = [
        DeclareLaunchArgument("world", default_value=str(user_cfg.get("world", "ocean"))),
        DeclareLaunchArgument("robot_namespace", default_value="otter"),
        DeclareLaunchArgument("robot_x", default_value="23.0"),
        DeclareLaunchArgument("robot_y", default_value="-10.0"),
        DeclareLaunchArgument("robot_z", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Start the Gazebo client"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable both Gazebo GUI and RViz"),
        DeclareLaunchArgument("planner_name", default_value="", choices=["", "astar", "dijkstra", "gbfs"],
                              description="Override project planner; empty preserves YAML (default astar)"),
        DeclareLaunchArgument("controller_name", default_value="", choices=["", "pid", "lqr"],
                              description="Override project controller; empty preserves YAML (default pid)"),
        DeclareLaunchArgument("trajectory_name", default_value="", choices=["", "los"],
                              description="Override project trajectory planner; empty preserves YAML (default los)"),
    ]
    for name, relative in (
        ("dynamics_config", ("robots", "otter", "dynamics_otter.yaml")),
        ("controller_config", ("robots", "otter", "controller_params_otter.yaml")),
        ("trajectory_config", ("trajectory_planner", "trajectory_planner_params.yaml")),
        ("planner_config", ("path_planner", "common_planner_params.yaml")),
    ):
        arguments.append(DeclareLaunchArgument(
            name, default_value=os.path.join(share, "config", *relative),
            description="Project YAML path (not a ros2 param dump)"))
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
