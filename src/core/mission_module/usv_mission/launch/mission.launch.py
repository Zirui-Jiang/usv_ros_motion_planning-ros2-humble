# 独立任务启动入口：仅创建任务节点，可用地理参考 YAML 或 world 文件提供显式原点。
"""Mission node only; odometry, TF and occupancy data are supplied externally."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from usv_mission.launch_support import common_arguments, mission_setup


def generate_launch_description():
    return LaunchDescription(common_arguments() + [
        DeclareLaunchArgument("geo_reference", default_value="",
                              description="Explicit WGS84/ENU origin YAML: latitude, longitude, altitude"),
        DeclareLaunchArgument("world_path", default_value="",
                              description="Explicit SDF world path, as an alternative geographic reference"),
        OpaqueFunction(function=mission_setup),
    ])
