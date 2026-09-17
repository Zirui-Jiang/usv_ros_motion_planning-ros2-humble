"""Shared project-YAML launch support, independent of business node packages."""
from dataclasses import dataclass
import os
import sys

from usv_navigation.configuration import load_project_parameters, resolve_parameters


ALGORITHM_CONFIGS = (
    ("dynamics_config", ("robots", "otter", "dynamics_otter.yaml")),
    ("controller_config", ("robots", "otter", "controller_params_otter.yaml")),
    ("trajectory_config", ("trajectory_planner", "trajectory_planner_params.yaml")),
    ("planner_config", ("path_planner", "common_planner_params.yaml")),
)
NAVIGATION_ARGUMENT_NAMES = (
    "robot_namespace", "use_sim_time", "planner_name", "controller_name",
    "trajectory_name", *[name for name, _ in ALGORITHM_CONFIGS],
)


def navigation_arguments():
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import DeclareLaunchArgument
    share = get_package_share_directory("usv_navigation")
    arguments = [
        DeclareLaunchArgument("robot_namespace", default_value="otter"),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("planner_name", default_value="", choices=["", "astar", "dijkstra", "gbfs"],
                              description="Override project planner; empty preserves YAML (default astar)"),
        DeclareLaunchArgument("controller_name", default_value="", choices=["", "pid", "lqr"],
                              description="Override project controller; empty preserves YAML (default pid)"),
        DeclareLaunchArgument("trajectory_name", default_value="", choices=["", "los"],
                              description="Override project trajectory planner; empty preserves YAML (default los)"),
    ]
    for name, relative in ALGORITHM_CONFIGS:
        arguments.append(DeclareLaunchArgument(
            name, default_value=os.path.join(share, "config", *relative),
            description="Project YAML path (not a ros2 param dump)"))
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
class NavigationSettings:
    namespace: str
    use_sim_time: bool
    navigation: dict
    logs: list


def navigation_settings(context):
    from launch.actions import LogInfo
    namespace = _value(context, "robot_namespace").strip("/")
    if not namespace:
        raise ValueError("robot_namespace must be nonempty")
    use_sim_time = _boolean(context, "use_sim_time")
    paths = {name: os.path.abspath(os.path.expanduser(_value(context, name)))
             for name, _ in ALGORITHM_CONFIGS}
    navigation = load_project_parameters(*paths.values())
    logs = [LogInfo(msg=f"[usv_navigation] {name}: {path}") for name, path in paths.items()]
    logs.append(LogInfo(msg="[usv_navigation] graph_planner_params.yaml is an inactive template and is not loaded."))
    # Preserve project YAML -> nonempty overrides -> namespace -> validation.
    for argument, parameter in (
        ("planner_name", "planner_name"),
        ("controller_name", "usv_simulation.controller.controller_name"),
        ("trajectory_name", "usv_simulation.trajectory_planner.planner_name"),
    ):
        if _value(context, argument):
            navigation[parameter] = _value(context, argument)
    navigation["dynamics.ns"] = namespace
    navigation = resolve_parameters(navigation)
    return NavigationSettings(namespace, use_sim_time, navigation, logs)


def navigation_node(settings, package, executable, name, extra=None):
    from launch_ros.actions import Node
    return Node(package=package, executable=executable, name=name,
                prefix=[sys.executable], namespace=settings.namespace, output="screen",
                parameters=[settings.navigation, {"use_sim_time": settings.use_sim_time}, extra or {}])
