"""Launch the simulation costmap to 0141 JSON adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    cost_threshold = LaunchConfiguration("cost_threshold")
    max_cells = LaunchConfiguration("max_cells")
    include_unknown = LaunchConfiguration("include_unknown")
    queue_depth = LaunchConfiguration("queue_depth")
    use_sim_time = LaunchConfiguration("use_sim_time")

    adapter = Node(
        package="usv_ladds_bridge",
        executable="costmap_0141_node",
        name="costmap_0141_publisher",
        output="screen",
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "cost_threshold": ParameterValue(cost_threshold, value_type=int),
                "max_cells": ParameterValue(max_cells, value_type=int),
                "include_unknown": ParameterValue(include_unknown, value_type=bool),
                "queue_depth": ParameterValue(queue_depth, value_type=int),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/otter/costmap_raw",
                description="Simulation nav2_msgs/Costmap topic.",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/ladds/obstacle_tx",
                description="std_msgs/String topic consumed by the 0141 bridge.",
            ),
            DeclareLaunchArgument(
                "cost_threshold",
                default_value="254",
                description="Minimum cost treated as an observed obstacle.",
            ),
            DeclareLaunchArgument(
                "max_cells",
                default_value="4096",
                description="Maximum sparse obstacle cells included in one JSON message.",
            ),
            DeclareLaunchArgument(
                "include_unknown",
                default_value="false",
                description="Include nav2 unknown cells (cost 255) as obstacles.",
            ),
            DeclareLaunchArgument(
                "queue_depth",
                default_value="10",
                description="ROS publisher/subscriber queue depth.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock for this node.",
            ),
            adapter,
        ]
    )

