"""Start the standalone ROS 2-to-LaDDS communication bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_parameters = str(
        Path(get_package_share_directory("usv_ladds_bridge"))
        / "config"
        / "bridge.yaml"
    )
    parameters_file = LaunchConfiguration("params_file")
    backend = LaunchConfiguration("backend")
    relay_host = LaunchConfiguration("relay_host")
    relay_port = LaunchConfiguration("relay_port")
    relay_timeout = LaunchConfiguration("relay_timeout")
    bridge = Node(
        package="usv_ladds_bridge",
        executable="usv_ladds_bridge_node",
        name="usv_ladds_bridge",
        output="screen",
        parameters=[
            parameters_file,
            {
                "backend": backend,
                "relay_host": relay_host,
                "relay_port": ParameterValue(relay_port, value_type=int),
                "relay_timeout": ParameterValue(relay_timeout, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="Path to the LaDDS bridge ROS parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="tcp",
                description="Transport backend: tcp or native.",
            ),
            DeclareLaunchArgument(
                "relay_host",
                default_value="127.0.0.1",
                description="TCP relay host visible from the ROS 2 environment.",
            ),
            DeclareLaunchArgument(
                "relay_port",
                default_value="17880",
                description="TCP relay port.",
            ),
            DeclareLaunchArgument(
                "relay_timeout",
                default_value="5.0",
                description="TCP relay connection timeout in seconds.",
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge,
                    on_exit=[EmitEvent(event=Shutdown(reason="LaDDS bridge exited"))],
                )
            ),
            bridge,
        ]
    )
