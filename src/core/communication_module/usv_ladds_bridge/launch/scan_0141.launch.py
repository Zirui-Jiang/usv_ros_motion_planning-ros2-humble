"""Launch the direct LaserScan-to-0141 adapter used by the simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    target_frame = LaunchConfiguration("target_frame")
    origin_latitude = LaunchConfiguration("origin_latitude")
    origin_longitude = LaunchConfiguration("origin_longitude")
    origin_altitude = LaunchConfiguration("origin_altitude")
    cluster_range_jump = LaunchConfiguration("cluster_range_jump")
    threat_radius = LaunchConfiguration("threat_radius")
    max_targets = LaunchConfiguration("max_targets")
    tf_timeout = LaunchConfiguration("tf_timeout")
    queue_depth = LaunchConfiguration("queue_depth")
    use_sim_time = LaunchConfiguration("use_sim_time")

    adapter = Node(
        package="usv_ladds_bridge",
        executable="usv_ladds_scan_node",
        output="screen",
        parameters=[
            {
                "input_topic": ParameterValue(input_topic, value_type=str),
                "output_topic": ParameterValue(output_topic, value_type=str),
                "target_frame": ParameterValue(target_frame, value_type=str),
                "origin_latitude": ParameterValue(origin_latitude, value_type=float),
                "origin_longitude": ParameterValue(origin_longitude, value_type=float),
                "origin_altitude": ParameterValue(origin_altitude, value_type=float),
                "cluster_range_jump": ParameterValue(cluster_range_jump, value_type=float),
                "threat_radius": ParameterValue(threat_radius, value_type=float),
                "max_targets": ParameterValue(max_targets, value_type=int),
                "tf_timeout": ParameterValue(tf_timeout, value_type=float),
                "queue_depth": ParameterValue(queue_depth, value_type=int),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/otter/sensors/lidars/lidar/scan",
                description="sensor_msgs/LaserScan topic published by the simulation.",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/ladds/obstacle_in",
                description="std_msgs/String topic consumed by the 0141 bridge.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="otter/map",
                description="TF frame in which the scan points become ENU coordinates.",
            ),
            DeclareLaunchArgument("origin_latitude", default_value="30.5453"),
            DeclareLaunchArgument("origin_longitude", default_value="114.392931"),
            DeclareLaunchArgument("origin_altitude", default_value="0.0"),
            DeclareLaunchArgument(
                "cluster_range_jump",
                default_value="1.0",
                description="Split adjacent valid beams when their range jump exceeds metres.",
            ),
            DeclareLaunchArgument("threat_radius", default_value="5.0"),
            DeclareLaunchArgument("max_targets", default_value="255"),
            DeclareLaunchArgument(
                "tf_timeout",
                default_value="0.1",
                description="Maximum seconds to wait for the scan timestamp's TF.",
            ),
            DeclareLaunchArgument("queue_depth", default_value="10"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            adapter,
        ]
    )

