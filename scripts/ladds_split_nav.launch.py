# 分容器 LaDDS 联调的算法端组合入口：感知、决策、航控和 LaDDS 航路适配器，不启动 Gazebo。
"""Perception, decision, control and the LaDDS route adapter without simulation.

Start it through scripts/start_ladds_split_nav.bash, which supplies the LaDDS
profile YAML paths and the relay address. Launch arguments share one context,
so every included entry reads the same command-line values.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 与 sim_env main.launch.py 相同的三个业务入口，只去掉仿真支撑。
NAVIGATION_ENTRIES = (
    ("usv_perception", "perception.launch.py"),
    ("usv_decision", "decision.launch.py"),
    ("usv_control", "control.launch.py"),
)


# 按包名定位已安装的 launch 文件并原样包含。
def _include(package, launch_file, **kwargs):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kwargs)


# 先声明 use_sim_time，避免适配器入口的默认 false 先写入共享参数；三个业务模块在前，适配器可单独关闭。
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "ladds_adapter", default_value="true", choices=["true", "false"],
            description="Also start the LaDDS bridge and 0211 route adapter"),
        # 0141 障碍层只有适配器在发栅格时才装载；须在感知入口之前声明，感知读取同一个值。
        DeclareLaunchArgument(
            "ladds_obstacles", default_value=LaunchConfiguration("ladds_adapter"),
            choices=["true", "false"],
            description="Add the LaDDS 0141 obstacle grid to the perception costmap"),
        *[_include(package, launch_file) for package, launch_file in NAVIGATION_ENTRIES],
        # The bridge exit handler shuts down this whole launch, navigation included.
        _include("usv_ladds_bridge", "route.launch.py",
                 condition=IfCondition(LaunchConfiguration("ladds_adapter"))),
    ])
