#!/usr/bin/env python3
"""
USV motion planning simulation environment launch (ROS 2 Humble, otter only).

Brings up: Gazebo ocean world, an otter USV (spawned from xacro),
robot_state_publisher, EKF localization, the perception mapping node and the
motion planning stack (sim_env node).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_user_config():
    """Read the user config to fetch the world name."""
    config_path = os.path.join(os.path.expanduser('~'), '.rmp_user_config.yaml')
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    return {}


def generate_launch_description():
    sim_env_share = get_package_share_directory('sim_env')
    pkg_project_share = FindPackageShare('sim_env')

    user_cfg = load_user_config()

    # ------------------------------------------------------------------ args
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=user_cfg.get('world', 'ocean'),
        description='Gazebo world file (without extension)')

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='otter',
        description='Robot namespace')

    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='23.0')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='-10.0')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.0')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Start Gazebo client')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz')

    # ------------------------------------------------------------- namespace
    robot_namespace = LaunchConfiguration('robot_namespace')

    # ------------------------------------------------------------------ xacro
    # IMU/GPS are disabled: localization is provided by the fake_localization
    # node from Gazebo ground truth instead of IMU/GPS/EKF fusion.
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_project_share, 'urdf', 'otter', 'otter.urdf.xacro']),
        ' namespace:=', robot_namespace,
        ' imu_enabled:=false',
        ' gps_enabled:=false',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Prefix every frame published from the URDF with the robot
            # namespace so the frames match the sensor plugins' frame_name
            # (e.g. otter/base_link, otter/lidar_link).
            'frame_prefix': [robot_namespace, '/'],
        }],
        output='screen',
    )

    # The two propeller joints are `continuous`, so robot_state_publisher can
    # only emit their TF (otter/left_propeller_link, otter/right_propeller_link)
    # once it receives /joint_states. Nothing in the sim publishes those joint
    # angles, so add a joint_state_publisher to feed static (zero) values.
    # It lives in the global namespace to match robot_state_publisher, which
    # subscribes to /joint_states and reads /robot_description.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # ----------------------------------------------------------------- gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_project_share, 'worlds',
                                            PythonExpression(["'", LaunchConfiguration('world'), "' + '.world'"]) ]),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'false',
        }.items(),
    )

    # ------------------------------------------------------------ spawn otter
    spawn_otter = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_namespace,
            '-topic', 'robot_description',
            '-timeout', '120',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-Y', LaunchConfiguration('robot_yaw'),
        ],
        output='screen',
    )

    # --------------------------------------------------------------- dynamics
    dynamics_params = os.path.join(sim_env_share, 'config', 'robots', 'otter',
                                  'dynamics_otter.yaml')
    with open(dynamics_params, 'r') as f:
        dynamics_cfg = yaml.safe_load(f)

    # --------------------------------------------------------------- mapping
    mapping_params = os.path.join(sim_env_share, 'config', 'costmap', 'mapping.yaml')
    with open(mapping_params, 'r') as f:
        mapping_cfg = yaml.safe_load(f)

    mapping = Node(
        package='perception',
        executable='perception_mapping_node',
        name='mapping',
        namespace=robot_namespace,
        output='screen',
        remappings=[('scan', 'sensors/lidars/lidar/scan')],
        parameters=[mapping_cfg,
                    {'offset_x': LaunchConfiguration('robot_x'),
                     'offset_y': LaunchConfiguration('robot_y'),
                     'odom_frame': 'odom',
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # -------------------------------------------------------------- location
    # Fake localization: derive a perfect map -> odom -> base_link TF chain and
    # an odometry stream directly from Gazebo ground truth, replacing the
    # IMU/GPS/EKF/navsat fusion stack.
    fake_localization = Node(
        package='sim_env',
        executable='fake_localization_node',
        name='fake_localization',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'model_name': robot_namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ------------------------------------------------------------- navigation
    controller_params = os.path.join(sim_env_share, 'config', 'robots', 'otter',
                                     'controller_params_otter.yaml')
    trajectory_params = os.path.join(sim_env_share, 'config', 'trajectory_planner',
                                     'trajectory_planner_params.yaml')
    common_planner_params = os.path.join(sim_env_share, 'config', 'path_planner',
                                         'common_planner_params.yaml')
    graph_planner_params = os.path.join(sim_env_share, 'config', 'path_planner',
                                        'graph_planner_params.yaml')
    costmap_params = os.path.join(sim_env_share, 'config', 'robots', 'otter',
                                  'global_costmap_params_otter.yaml')

    def load_params(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}

    controller_cfg = load_params(controller_params)
    trajectory_cfg = load_params(trajectory_params)
    common_planner_cfg = load_params(common_planner_params)
    graph_planner_cfg = load_params(graph_planner_params)
    costmap_cfg = load_params(costmap_params)

    sim_node = Node(
        package='sim_env',
        executable='sim_env_node',
        name='usv_simulation',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            dynamics_cfg,
            # The controller and trajectory factories declare their parameters
            # under the node-name prefix "usv_simulation." (see
            # controller_factory.cpp / trajectory_planner_factory.cpp). The
            # per-subsystem yamls use bare top keys ("controller:",
            # "trajectory_planner:"), so without this wrapper the tuned values
            # never match the declared names and the code falls back to its
            # defaults -- notably all PID gains = 0, which produces zero thrust
            # and a stationary boat. Wrap them so the flattened parameter names
            # become usv_simulation.controller.* / usv_simulation.trajectory_planner.*.
            {'usv_simulation': {**controller_cfg, **trajectory_cfg}},
            common_planner_cfg,
            graph_planner_cfg,
            costmap_cfg,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ------------------------------------------------------------------- rviz
    rviz_config = os.path.join(sim_env_share, 'rviz', 'sim_env.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        robot_namespace_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_yaw_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_otter,
        mapping,
        fake_localization,
        sim_node,
        rviz,
    ])
