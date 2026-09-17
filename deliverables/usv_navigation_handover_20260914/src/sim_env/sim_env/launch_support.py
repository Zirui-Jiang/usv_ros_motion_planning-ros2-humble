"""Simulation composition and forwarding entries for ordinary navigation."""
from dataclasses import dataclass
import os
import sys

from sim_env.configuration import load_yaml
from usv_navigation.launch_support import (
    ALGORITHM_CONFIGS, NAVIGATION_ARGUMENT_NAMES,
    navigation_arguments, navigation_settings,
)


PERCEPTION_CONFIGS = (
    ("mapping_config", ("costmap", "mapping.yaml")),
    ("costmap_config", ("robots", "otter", "global_costmap_params_otter.yaml")),
)
COMMON_ARGUMENT_NAMES = (
    "world", "robot_x", "robot_y", "robot_z", "robot_yaw", "gui", "rviz", "headless",
    *NAVIGATION_ARGUMENT_NAMES, *[name for name, _ in PERCEPTION_CONFIGS],
)
MODULE_ENTRIES = {
    "simulation_support": "sim_env",
    "perception": "usv_perception",
    "decision": "usv_decision",
    "control": "usv_control",
}


def load_user_config():
    """Retain the original optional per-user world selection."""
    path = os.path.join(os.path.expanduser("~"), ".rmp_user_config.yaml")
    return load_yaml(path) if os.path.isfile(path) else {}


def common_arguments():
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import DeclareLaunchArgument
    perception_share = get_package_share_directory("usv_perception")
    user_config = load_user_config()
    arguments = [
        DeclareLaunchArgument("world", default_value=str(user_config.get("world", "ocean"))),
        DeclareLaunchArgument("robot_x", default_value="23.0"),
        DeclareLaunchArgument("robot_y", default_value="-10.0"),
        DeclareLaunchArgument("robot_z", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Start the Gazebo client"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable both Gazebo GUI and RViz"),
        *navigation_arguments(),
    ]
    for name, relative in PERCEPTION_CONFIGS:
        arguments.append(DeclareLaunchArgument(
            name, default_value=os.path.join(perception_share, "config", *relative)))
    return arguments


def _value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)


def _boolean(context, name):
    value = _value(context, name).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


@dataclass
class LaunchSettings:
    share: str
    namespace: str
    use_sim_time: bool
    pose: dict
    logs: list


def _settings(context):
    from ament_index_python.packages import get_package_share_directory
    navigation = navigation_settings(context)
    pose = {axis: float(_value(context, "robot_" + axis)) for axis in ("x", "y", "z", "yaw")}
    return LaunchSettings(get_package_share_directory("sim_env"), navigation.namespace,
                          navigation.use_sim_time, pose, navigation.logs)


def _simulation_nodes(context, settings):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import Command
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    namespace, pose = settings.namespace, settings.pose
    headless = _boolean(context, "headless")
    gui = _boolean(context, "gui") and not headless
    rviz = _boolean(context, "rviz") and not headless
    description = Command([
        "xacro ", os.path.join(settings.share, "urdf", "otter", "otter.urdf.xacro"),
        " namespace:=", namespace, " imu_enabled:=false gps_enabled:=false lidar_enabled:=true",
    ])
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
            launch_arguments={"world": os.path.join(settings.share, "worlds", _value(context, "world") + ".world"),
                              "gui": "true" if gui else "false", "verbose": "false"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(description, value_type=str),
                          "use_sim_time": settings.use_sim_time, "frame_prefix": namespace + "/"}], output="screen"),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[{"use_sim_time": settings.use_sim_time}], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py", prefix=[sys.executable],
             arguments=["-entity", namespace, "-topic", "robot_description", "-timeout", "120.0",
                        "-x", str(pose["x"]), "-y", str(pose["y"]),
                        "-z", str(pose["z"]), "-Y", str(pose["yaw"])], output="screen"),
        Node(package="sim_env", executable="fake_localization_node", name="fake_localization",
             namespace=namespace, output="screen",
             parameters=[{"model_name": namespace, "use_sim_time": settings.use_sim_time}]),
    ]
    if rviz:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(settings.share, "rviz", "sim_env.rviz")],
            parameters=[{"use_sim_time": settings.use_sim_time}], output="screen"))
    return nodes


def _include_module(context, name):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    arguments = {argument: _value(context, argument) for argument in COMMON_ARGUMENT_NAMES}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(MODULE_ENTRIES[name]), "launch", name + ".launch.py")),
        launch_arguments=arguments.items())


def _launch_setup(context, module):
    settings = _settings(context)
    if module == "main":
        return [_include_module(context, name) for name in MODULE_ENTRIES]
    if module == "simulation_support":
        return [*settings.logs, *_simulation_nodes(context, settings)]
    if module in MODULE_ENTRIES:
        return [_include_module(context, module)]
    raise ValueError(f"Unknown navigation launch module: {module}")


def launch_description(module):
    from launch import LaunchDescription
    from launch.actions import OpaqueFunction
    return LaunchDescription(common_arguments() + [OpaqueFunction(function=_launch_setup, kwargs={"module": module})])
