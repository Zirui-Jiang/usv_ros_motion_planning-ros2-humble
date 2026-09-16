# 完整任务仿真入口：用既有 Gazebo 资源和感知支持节点组合单一任务控制器。
"""Existing Gazebo resources and support nodes, with one mission controller."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from usv_mission.launch_support import simulation_arguments, simulation_setup


def generate_launch_description():
    return LaunchDescription(simulation_arguments() + [OpaqueFunction(function=simulation_setup)])
