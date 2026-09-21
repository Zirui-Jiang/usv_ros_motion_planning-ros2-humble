"""旧入口：转发到 route.launch.py，并保留原 launch 参数。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('usv_ladds_bridge'))
    names = (
        'params_file', 'backend', 'relay_host', 'relay_port',
        'obstacle_input_topic', 'obstacle_send_topic', 'use_sim_time',
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'route.yaml')),
        DeclareLaunchArgument('backend', default_value='tcp'),
        DeclareLaunchArgument('relay_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('relay_port', default_value='17880'),
        DeclareLaunchArgument('obstacle_input_topic', default_value='/ladds/obstacle_in'),
        DeclareLaunchArgument('obstacle_send_topic', default_value='0141'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'route.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items()),
    ])
