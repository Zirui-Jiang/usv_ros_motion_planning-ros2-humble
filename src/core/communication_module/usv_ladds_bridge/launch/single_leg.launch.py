"""Run the byte bridge and one-waypoint voyage adapter as separate nodes."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('usv_ladds_bridge'))
    parameters = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'single_leg.yaml')),
        DeclareLaunchArgument('backend', default_value='tcp'),
        DeclareLaunchArgument('relay_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('relay_port', default_value='17880'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'bridge.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ('params_file', 'backend', 'relay_host', 'relay_port')}.items()),
        Node(package='usv_ladds_bridge', executable='ladds_single_leg_node',
             name='ladds_single_leg', output='screen',
             parameters=[parameters, {'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
