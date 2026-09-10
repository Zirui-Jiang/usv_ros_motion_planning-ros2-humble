"""C++ baseline adapter: selectable controller and headless defaults only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_name', default_value='pid', choices=['pid', 'lqr']),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetParameter(name='usv_simulation.controller.controller_name',
                     value=LaunchConfiguration('controller_name')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sim_env'), 'launch', 'main.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ])
