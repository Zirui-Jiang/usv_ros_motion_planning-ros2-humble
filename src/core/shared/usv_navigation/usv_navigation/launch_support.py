# 独立业务节点共享的启动配置工具：定位四类项目 YAML、处理 launch 覆盖并构造一致命名空间节点。
"""Shared project-YAML launch support, independent of business node packages."""
from dataclasses import dataclass
import os
import sys

from usv_navigation.configuration import load_project_parameters, resolve_parameters


# 船型决定默认的船模 YAML、控制器 YAML 和仿真船模；命名空间默认与船型同名，显式给出 robot_namespace 时以其为准。
ROBOT_TYPES = ("otter", "wamv")
# 路径中的 {robot} 在声明默认值时换成 robot_type 的取值；后两份配置与船型无关。
ALGORITHM_CONFIGS = (
    ("dynamics_config", ("robots", "{robot}", "dynamics_{robot}.yaml")),
    ("controller_config", ("robots", "{robot}", "controller_params_{robot}.yaml")),
    ("trajectory_config", ("trajectory_planner", "trajectory_planner_params.yaml")),
    ("planner_config", ("path_planner", "common_planner_params.yaml")),
)
NAVIGATION_ARGUMENT_NAMES = (
    "robot_type", "robot_namespace", "use_sim_time", "planner_name", "controller_name",
    "trajectory_name", *[name for name, _ in ALGORITHM_CONFIGS],
)


# 指定用户场景配置文件的环境变量；设置后只读这一个文件。
USER_CONFIG_ENVIRONMENT = "USV_USER_CONFIG"
USER_CONFIG_RELATIVE = ("src", "user_config", "user_config.yaml")


# 按先后顺序查找用户场景配置：环境变量指定的文件、本包所在工作区里的 src/user_config/user_config.yaml、
# 用户主目录下的 .rmp_user_config.yaml；都没有时返回空字符串。
def user_config_path():
    """Locate the optional user_config.yaml that selects the world and vessel."""
    explicit = os.environ.get(USER_CONFIG_ENVIRONMENT, "")
    if explicit:
        path = os.path.abspath(os.path.expanduser(explicit))
        # An explicit choice that is missing must not silently fall back to another file.
        if not os.path.exists(path):
            raise ValueError(f"{USER_CONFIG_ENVIRONMENT} does not exist: {path}")
        return path
    # The package runs either from <workspace>/install or from <workspace>/src;
    # both lie below the workspace that owns src/user_config.
    directory = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidate = os.path.join(directory, *USER_CONFIG_RELATIVE)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    home = os.path.join(os.path.expanduser("~"), ".rmp_user_config.yaml")
    return home if os.path.isfile(home) else ""


# 返回 (文件路径, 内容字典)；没有文件或文件为空时内容为空字典。
def load_user_config():
    import yaml
    path = user_config_path()
    if not path:
        return "", {}
    with open(path, encoding="utf-8") as stream:
        values = yaml.safe_load(stream) or {}
    if not isinstance(values, dict):
        raise ValueError(f"Expected a YAML mapping in user config: {path}")
    return path, values


# 沿用原工程的写法：船型取 robots_config 列表里的 robot1_type；没有该键时为 otter，未登记的船型直接报错。
def user_robot_type(path, values):
    for entry in values.get("robots_config") or []:
        if isinstance(entry, dict) and "robot1_type" in entry:
            robot_type = str(entry["robot1_type"])
            if robot_type not in ROBOT_TYPES:
                raise ValueError(f"{path}: robot1_type must be one of {', '.join(ROBOT_TYPES)}; got {robot_type!r}")
            return robot_type
    return "otter"


# 声明船型参数；默认值来自用户场景配置的 robot1_type，命令行给出 robot_type:= 时以命令行为准。
# 各入口都要先声明它，随船型变化的默认路径才能求值。
def robot_type_argument():
    from launch.actions import DeclareLaunchArgument
    return DeclareLaunchArgument(
        "robot_type", default_value=user_robot_type(*load_user_config()), choices=list(ROBOT_TYPES),
        description="Vessel model; selects the default namespace, per-vessel YAML files and the simulated hull. "
                    "Defaults to robot1_type of user_config.yaml")


# 命名空间默认取船型名（otter → /otter，wamv → /wamv），决定话题与坐标系前缀。
def robot_namespace_argument():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    return DeclareLaunchArgument(
        "robot_namespace", default_value=LaunchConfiguration("robot_type"),
        description="Topic and frame prefix; defaults to the robot_type name")


# 把含 {robot} 占位的相对路径拼成默认值；没有占位时是普通字符串，有占位时随 robot_type 延迟求值。
def robot_config_default(share, relative):
    from launch.substitutions import LaunchConfiguration
    path = os.path.join(share, "config", *relative)
    if "{robot}" not in path:
        return path
    pieces = path.split("{robot}")
    default = [pieces[0]]
    for piece in pieces[1:]:
        default += [LaunchConfiguration("robot_type"), piece]
    return default


# 声明船型、命名空间、仿真时间、可选算法覆盖和四份配置路径；空算法覆盖保留 YAML 设置。
def navigation_arguments():
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import DeclareLaunchArgument
    share = get_package_share_directory("usv_navigation")
    arguments = [
        robot_type_argument(),
        robot_namespace_argument(),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("planner_name", default_value="", choices=["", "astar", "dijkstra", "gbfs", "hybrid_astar"],
                              description="Override project planner; empty preserves YAML (default astar)"),
        DeclareLaunchArgument("controller_name", default_value="", choices=["", "pid", "lqr", "adrc"],
                              description="Override project controller; empty preserves YAML (default pid)"),
        DeclareLaunchArgument("trajectory_name", default_value="", choices=["", "los"],
                              description="Override project trajectory planner; empty preserves YAML (default los)"),
    ]
    for name, relative in ALGORITHM_CONFIGS:
        arguments.append(DeclareLaunchArgument(
            name, default_value=robot_config_default(share, relative),
            description="Project YAML path (not a ros2 param dump)"))
    return arguments


def _value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)


# 只把 true/false 文本解释为布尔值，拒绝其它拼写。
def _boolean(context, name):
    value = _value(context, name).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


# 一次解析后的启动快照，包含命名空间、时钟选项、有效参数与来源日志。
@dataclass
class NavigationSettings:
    namespace: str
    use_sim_time: bool
    navigation: dict
    logs: list


# 按项目 YAML、非空算法覆盖、命名空间修正、完整验证的顺序生成启动设置。
def navigation_settings(context):
    from launch.actions import LogInfo
    namespace = _value(context, "robot_namespace").strip("/")
    if not namespace:
        raise ValueError("robot_namespace must be nonempty")
    use_sim_time = _boolean(context, "use_sim_time")
    paths = {name: os.path.abspath(os.path.expanduser(_value(context, name)))
             for name, _ in ALGORITHM_CONFIGS}
    navigation = load_project_parameters(*paths.values())
    # 先说明船型及其默认值取自哪份用户场景配置，便于排查"为什么不是预期的船型"。
    logs = [LogInfo(msg=f"[usv_navigation] robot_type: {_value(context, 'robot_type')} "
                        f"(user config: {user_config_path() or 'none'})")]
    logs += [LogInfo(msg=f"[usv_navigation] {name}: {path}") for name, path in paths.items()]
    logs.append(LogInfo(msg="[usv_navigation] graph_planner_params.yaml is an inactive template and is not loaded."))
    # Preserve project YAML -> nonempty overrides -> namespace -> validation.
    for argument, parameter in (
        ("planner_name", "planner_name"),
        ("controller_name", "usv_simulation.controller.controller_name"),
        ("trajectory_name", "usv_simulation.trajectory_planner.planner_name"),
    ):
        if _value(context, argument):
            navigation[parameter] = _value(context, argument)
    navigation["dynamics.ns"] = namespace
    navigation = resolve_parameters(navigation)
    return NavigationSettings(namespace, use_sim_time, navigation, logs)


# 使用当前 Python 解释器启动业务节点，注入有效参数、仿真时间和附加配置。
def navigation_node(settings, package, executable, name, extra=None):
    from launch_ros.actions import Node
    return Node(package=package, executable=executable, name=name,
                prefix=[sys.executable], namespace=settings.namespace, output="screen",
                parameters=[settings.navigation, {"use_sim_time": settings.use_sim_time}, extra or {}])
