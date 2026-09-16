# 任务启动组合：复用只读算法配置及仿真资源，明确单独任务入口与完整仿真入口。
"""Public ROS launch composition owned by the mission application.

Algorithm YAML and Gazebo assets are read-only inputs. No Simulator internals
are imported, and only the mission node owns navigation/control publication.
ROS launch imports are local so geographic configuration can be tested without
a ROS installation.
"""
import os
import sys

from common.geodesy import GeoReference
from usv_mission.configuration import load_project_parameters, load_yaml, resolve_parameters


ALGORITHM_CONFIGS = (
    ("dynamics_config", ("robots", "otter", "dynamics_otter.yaml")),
    ("controller_config", ("robots", "otter", "controller_params_otter.yaml")),
    ("trajectory_config", ("trajectory_planner", "trajectory_planner_params.yaml")),
    ("planner_config", ("path_planner", "common_planner_params.yaml")),
)


# 显式地理参考 YAML 与 world 文件二选一；缺省时禁用地理任务，显式零原点仍合法。
def geographic_parameters(geo_reference="", world_path=""):
    """Require an explicitly supplied WGS84/ENU reference before enabling tasks.

    A reference YAML has exactly latitude, longitude and optional altitude.
    Explicit (0, 0) is valid; absence never silently enables that origin.
    """
    if geo_reference and world_path:
        raise ValueError("Supply geo_reference or world_path, not both")
    if geo_reference:
        values = load_yaml(os.path.abspath(os.path.expanduser(geo_reference)))
        if (not isinstance(values, dict)
                or not {"latitude", "longitude"}.issubset(values)
                or set(values) - {"latitude", "longitude", "altitude"}):
            raise ValueError("geo_reference YAML requires latitude, longitude and optional altitude")
        geo = GeoReference(**values)
    elif world_path:
        geo = GeoReference.from_world(os.path.abspath(os.path.expanduser(world_path)))
    else:
        return {"mission.geo.enabled": False}
    return {"mission.geo.enabled": True, "mission.geo.latitude": geo.latitude,
            "mission.geo.longitude": geo.longitude, "mission.geo.altitude": geo.altitude}


# 在 launch 上下文中求出参数最终字符串值。
def _value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)


# 只接收 true/false 字符串，避免非空字符串误当布尔真。
def _boolean(context, name):
    value = _value(context, name).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


# 去掉命名空间两端斜线，并拒绝空船舶名称。
def _namespace(context):
    namespace = _value(context, "robot_namespace").strip("/")
    if not namespace:
        raise ValueError("robot_namespace must be nonempty")
    return namespace


# 读取工程算法配置、合并地理参考及启动覆盖值，然后再次统一校验。
def _parameters(context, geographic):
    """Load exactly the existing numerical configurations, then launch overrides."""
    from launch.actions import LogInfo
    paths = {name: os.path.abspath(os.path.expanduser(_value(context, name)))
             for name in (*[name for name, _ in ALGORITHM_CONFIGS], "mission_config")}
    parameters = load_project_parameters(*paths.values())
    parameters.update(geographic)
    for argument, parameter in (
        ("planner_name", "planner_name"),
        ("controller_name", "usv_simulation.controller.controller_name"),
        ("trajectory_name", "usv_simulation.trajectory_planner.planner_name"),
    ):
        if _value(context, argument):
            parameters[parameter] = _value(context, argument)
    parameters["dynamics.ns"] = _namespace(context)
    logs = [LogInfo(msg=f"[usv_mission] {name}: {path}") for name, path in paths.items()]
    logs.append(LogInfo(msg=("[usv_mission] Geographic reference enabled" if geographic["mission.geo.enabled"]
                            else "[usv_mission] Geographic tasks disabled: supply an explicit WGS84/ENU reference")))
    return resolve_parameters(parameters), logs


# 用当前 Python 解释器创建唯一任务节点，并传入明确的时钟模式。
def _mission_node(context, parameters):
    from launch_ros.actions import Node
    return Node(package="usv_mission", executable="mission_node", name="mission_manager",
                prefix=[sys.executable], namespace=_namespace(context), output="screen",
                parameters=[parameters, {"use_sim_time": _boolean(context, "use_sim_time")}])


# 只组合任务节点，状态、TF 与地图由外部 ROS 图提供。
def mission_setup(context):
    """Run only task dispatch/execution against an externally provided ROS graph."""
    parameters, logs = _parameters(context, geographic_parameters(
        _value(context, "geo_reference"), _value(context, "world_path")))
    return [*logs, _mission_node(context, parameters)]


# 组合 Gazebo、机器人状态、定位、建图、代价地图及任务节点，headless 同时关闭图形界面。
def simulation_setup(context):
    """Compose existing simulation support nodes and the new mission application."""
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription, LogInfo
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import Command
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    share = get_package_share_directory("sim_env")
    perception_share = get_package_share_directory("usv_perception")
    mission_share = get_package_share_directory("usv_mission")
    namespace = _namespace(context)
    prefix = "/" + namespace
    use_sim_time = _boolean(context, "use_sim_time")
    headless = _boolean(context, "headless")
    gazebo_gui = _boolean(context, "gui") and not headless
    show_rviz = _boolean(context, "rviz") and not headless
    pose = {axis: float(_value(context, "robot_" + axis)) for axis in ("x", "y", "z", "yaw")}
    world_name = os.path.expanduser(_value(context, "world"))
    if not world_name.endswith(".world"):
        world_name += ".world"
    world_path = os.path.abspath(os.path.join(share, "worlds", world_name))
    try:
        geographic = geographic_parameters(world_path=world_path)
        reference_logs = [LogInfo(msg=f"[usv_mission] Geographic reference: {world_path}")]
    except ValueError as error:
        geographic = {"mission.geo.enabled": False}
        reference_logs = [LogInfo(msg=f"[usv_mission] Geographic tasks disabled: {error}")]
    parameters, logs = _parameters(context, geographic)

    costmap_params = load_yaml(os.path.join(
        perception_share, "config", "robots", "otter", "global_costmap_params_otter.yaml"))["costmap"]

    robot_description = Command([
        "xacro ", os.path.join(share, "urdf", "otter", "otter.urdf.xacro"),
        " namespace:=", namespace, " imu_enabled:=false gps_enabled:=false lidar_enabled:=true",
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str),
                     "use_sim_time": use_sim_time, "frame_prefix": namespace + "/"}], output="screen")
    joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}], output="screen")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_path, "gui": "true" if gazebo_gui else "false",
                          "verbose": "false"}.items())
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", prefix=[sys.executable],
        arguments=["-entity", namespace, "-topic", "robot_description", "-timeout", "120.0",
                   "-x", str(pose["x"]), "-y", str(pose["y"]), "-z", str(pose["z"]),
                   "-Y", str(pose["yaw"])], output="screen")
    fake_localization = Node(
        package="sim_env", executable="fake_localization_node", name="fake_localization",
        namespace=namespace, output="screen",
        parameters=[{"model_name": namespace, "use_sim_time": use_sim_time}])

    # 只检查显式列出的有效图层，启动契约要求静态层不参与本组合。
    plugins = costmap_params.get("plugins")
    if not isinstance(plugins, list) or not plugins or not all(isinstance(name, str) for name in plugins):
        raise ValueError("Costmap YAML must provide an explicit nonempty plugins list")
    for plugin_name in plugins:
        plugin_config = costmap_params.get(plugin_name, {})
        is_static = (plugin_name == "static_layer"
                     or str(plugin_config.get("plugin", "")).endswith("StaticLayer"))
        if is_static and plugin_config.get("enabled", True):
            raise ValueError("This launch requires static_layer to remain disabled; costmap YAML enables it")
    costmap_params.update(global_frame=namespace + "/map", robot_base_frame=namespace + "/base_link",
                          use_sim_time=use_sim_time)
    scan = costmap_params["obstacle_layer"]["scan"]
    scan.update(topic=prefix + "/sensors/lidars/lidar/scan", sensor_frame=namespace + "/lidar_link")
    costmap = Node(
        package="usv_perception", executable="costmap_support_node", name="costmap",
        namespace=namespace, output="screen", parameters=[costmap_params],
        remappings=[("costmap_raw", prefix + "/costmap_raw"), ("costmap", prefix + "/costmap"),
                    ("costmap_updates", prefix + "/costmap_updates")])
    sensor = Node(
        package="usv_perception", executable="perception_mapping_node", name="mapping",
        namespace=namespace, output="screen", remappings=[("scan", "sensors/lidars/lidar/scan")],
        parameters=[load_yaml(os.path.join(perception_share, "config", "costmap", "mapping.yaml")),
                    {"offset_x": pose["x"], "offset_y": pose["y"], "use_sim_time": use_sim_time}])
    actions = [*logs, *reference_logs, gazebo, robot_state_publisher, joint_state_publisher,
               spawn, fake_localization, sensor, costmap, _mission_node(context, parameters)]
    if show_rviz:
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(mission_share, "rviz", "mission.rviz")],
            parameters=[{"use_sim_time": use_sim_time}], output="screen"))
    return actions


# 声明任务与仿真共用的算法选择、船舶命名空间和配置文件路径。
def common_arguments(simulation=False):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import DeclareLaunchArgument
    share = get_package_share_directory("usv_navigation")
    mission_share = get_package_share_directory("usv_mission")
    arguments = [
        DeclareLaunchArgument("robot_namespace", default_value="otter"),
        DeclareLaunchArgument("use_sim_time", default_value="true" if simulation else "false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("planner_name", default_value="", choices=["", "astar", "dijkstra", "gbfs"]),
        DeclareLaunchArgument("controller_name", default_value="", choices=["", "pid", "lqr"]),
        DeclareLaunchArgument("trajectory_name", default_value="", choices=["", "los"]),
    ]
    for name, relative in ALGORITHM_CONFIGS:
        arguments.append(DeclareLaunchArgument(name, default_value=os.path.join(share, "config", *relative),
                                              description="Read-only project algorithm YAML path"))
    arguments.append(DeclareLaunchArgument("mission_config", default_value=os.path.join(
        mission_share, "config", "mission", "mission_params.yaml")))
    return arguments


# 读取用户世界偏好，并声明出生位姿与 GUI/RViz 参数。
def simulation_arguments():
    from launch.actions import DeclareLaunchArgument
    user_path = os.path.join(os.path.expanduser("~"), ".rmp_user_config.yaml")
    user_config = load_yaml(user_path) if os.path.isfile(user_path) else {}
    arguments = [
        DeclareLaunchArgument("world", default_value=str(user_config.get("world", "ocean"))),
        DeclareLaunchArgument("robot_x", default_value="23.0"),
        DeclareLaunchArgument("robot_y", default_value="-10.0"),
        DeclareLaunchArgument("robot_z", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
    ]
    return common_arguments(simulation=True) + arguments
