"""旧入口：转发到 costmap.launch.py，并保留原 launch 参数。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('usv_ladds_bridge'))
    names = (
        'input_topic', 'output_topic', 'cost_threshold', 'max_cells',
        'include_unknown', 'queue_depth', 'use_sim_time',
    )
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/otter/costmap_raw'),
        DeclareLaunchArgument('output_topic', default_value='/ladds/obstacle_in'),
        DeclareLaunchArgument('cost_threshold', default_value='254'),
        DeclareLaunchArgument('max_cells', default_value='4096'),
        DeclareLaunchArgument('include_unknown', default_value='false'),
        DeclareLaunchArgument('queue_depth', default_value='10'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'costmap.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items()),
    ])
