# 任务应用的配置入口：合并工程 YAML、兼容旧参数名并校验启动及在线更新参数。
"""Project YAML compatibility and validated ROS parameter snapshots."""
from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path
import yaml
from common.models import USVModel
from controller.controller import validate_goal_distance_tolerance

C = "usv_simulation.controller."
T = "usv_simulation.trajectory_planner."

# 时间参数采用秒，频率采用 Hz；平面位置及船体尺寸采用米，ROS 航向与角速度采用弧度及弧度/秒。
DEFAULTS = {
    "planner_name": "astar", "obstacle_factor": 0.25,
    "path_planner.outline_map": True, "path_planner.enable_resample": True,
    "path_planner.enable_shortcut": False,
    "path_planner.resample_ratio": 0.5, "path_planner.convert_offset": 0.0,
    "path_planner.default_tolerance": 0.0, "path_planner.expand_zone": True,
    "path_planner.show_safety_corridor": False,
    "navigation.planning_frequency": 5.0, "navigation.state_timeout": 0.5,
    "navigation.map_timeout": 1.5, "navigation.trajectory_timeout": 1.0,
    "navigation.planning_timeout": 1.0, "navigation.max_expansions": 200000,
    "navigation.costmap_topic": "costmap_raw",
    "navigation.footprint_length": 2.2, "navigation.footprint_width": 1.2,
    "mission.geo.enabled": False, "mission.geo.latitude": 0.0,
    "mission.geo.longitude": 0.0, "mission.geo.altitude": 0.0,
    "mission.heading_tolerance_deg": 5.0, "mission.safety_margin": 0.1,
    C + "control_frequency": 10.0, C + "controller_name": "pid",
    C + "pid_controller.goal_distance_tolerance": 1.5,
    C + "pid_controller.goal_speed_tolerance": 0.25,
    C + "pid_controller.lookahead_time": 2.6,
    C + "pid_controller.k_p_speed": 2.0, C + "pid_controller.k_i_speed": 0.5,
    C + "pid_controller.k_d_speed": 0.0, C + "pid_controller.k_p_course": 2.0,
    C + "pid_controller.k_i_course": 0.0, C + "pid_controller.k_d_course": 0.0,
    C + "lqr_controller.goal_distance_tolerance": 1.0,
    C + "lqr_controller.goal_speed_tolerance": 0.25,
    C + "lqr_controller.lookahead_time": 2.0,
    C + "lqr_controller.discrete_dt": 0.1,
    C + "lqr_controller.solve_eps": 0.1,
    C + "lqr_controller.max_iterations": 5000,
    C + "lqr_controller.solve_timeout": 0.08,
    C + "lqr_controller.Q_matrix_diag": [1., 1., 100., 10., 10., 10.],
    C + "lqr_controller.R_matrix_diag": [20., 20.],
    T + "planner_name": "los",
    T + "los.trajectory_time_interval": 0.5, T + "los.trajectory_time": 20.0,
    T + "los.switch_dist": 5.0, T + "los.goal_tolerance": 1.0,
    T + "los.lookahead_distance": 1.0, T + "los.yaw_rate_max": 0.3,
    T + "los.kp_yaw": 1.0,
    T + "los.slowdown_distance": 3.0,
}
DEFAULTS.update({"dynamics." + k: v for k, v in asdict(USVModel()).items()})
# 在线更新限于不改变算法实例、定时器频率和几何契约的参数。
HOT_PARAMETERS = frozenset(
    C + "pid_controller." + k for k in
    ("k_p_speed", "k_i_speed", "k_d_speed", "k_p_course", "k_i_course", "k_d_course")
) | frozenset(T + "los." + k for k in
              ("lookahead_distance", "switch_dist", "kp_yaw", "yaw_rate_max"))


# 合并显式来源；同名参数值不一致时拒绝，避免由加载顺序悄悄决定配置。
def merge_explicit(target, source):
    for key, value in source.items():
        if key in target and target[key] != value:
            raise ValueError(f"Conflicting configuration sources for {key}")
        target[key] = value
    return target


# 把嵌套字典展开为 ROS 点分参数名，同时检查键类型和重复定义。
def flatten(mapping, prefix=""):
    if not isinstance(mapping, dict):
        raise ValueError("YAML root must be a mapping")
    result = {}
    for key, value in mapping.items():
        if not isinstance(key, str):
            raise ValueError("Configuration keys must be strings")
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            merge_explicit(result, flatten(value, name))
        else:
            if name in result:
                raise ValueError(f"Duplicate parameter {name}")
            result[name] = value
    return result


# 统一历史拼写别名；LQR 矩阵转浮点列表以匹配 ROS double array 声明。
def normalize_parameters(values):
    result = deepcopy(values)
    for canonical, aliases in (
        (C + "control_frequency", (C + "control_frequncy", C + "controller_frequncy")),
        ("obstacle_factor", ("path_planner.obstacle_factor",)),
    ):
        present = [(key, result[key]) for key in (canonical, *aliases) if key in result]
        if present:
            if any(value != present[0][1] for _, value in present):
                raise ValueError(f"Conflicting aliases for {canonical}: {present}")
            for key, _ in present:
                result.pop(key)
            result[canonical] = present[0][1]
    # Original LQR YAML uses integer lists, whereas ROS declares double arrays.
    for key in (C + "lqr_controller.Q_matrix_diag", C + "lqr_controller.R_matrix_diag"):
        if key in result:
            value = result[key]
            if not isinstance(value, list) or any(type(v) not in (int, float) for v in value):
                raise ValueError(f"{key} must be a numeric array")
            result[key] = [float(v) for v in value]
    return result


# 同时检查数据类型、有限值、可用算法及模型约束，返回前不写入节点状态。
def validate_parameters(values):
    for key, default in DEFAULTS.items():
        value = values[key]
        if isinstance(default, bool):
            valid = type(value) is bool
        elif isinstance(default, str):
            valid = isinstance(value, str) and bool(value)
        elif isinstance(default, int):
            valid = type(value) is int and value > 0
        elif isinstance(default, float):
            valid = type(value) in (int, float) and math.isfinite(value)
        else:
            valid = isinstance(value, list) and all(type(v) in (int, float) and math.isfinite(v) for v in value)
        if not valid:
            raise ValueError(f"Invalid type or nonfinite parameter: {key}={value!r}")
    for name in ("pid", "lqr"):
        key = C + name + "_controller.goal_distance_tolerance"
        validate_goal_distance_tolerance(values[key], key)
    if values["planner_name"] not in ("astar", "dijkstra", "gbfs"):
        raise ValueError("Only astar, dijkstra and gbfs are implemented")
    if values[C + "controller_name"] not in ("pid", "lqr") or values[T + "planner_name"] != "los":
        raise ValueError("Only PID/LQR controllers and LOS trajectory planner are implemented")
    if not 0 < values["obstacle_factor"] <= 1:
        raise ValueError("obstacle_factor must be in (0,1]")
    if not -90 <= values["mission.geo.latitude"] <= 90 or not -180 <= values["mission.geo.longitude"] <= 180:
        raise ValueError("mission.geo latitude/longitude outside WGS84 degree ranges")
    if not 0 < values["mission.heading_tolerance_deg"] <= 180 or values["mission.safety_margin"] < 0:
        raise ValueError("Mission heading tolerance must be in (0,180]; safety margin must be nonnegative")
    nonnegative = {k for k in HOT_PARAMETERS if ".k_" in k}
    positive = {k for k in DEFAULTS if k.startswith("navigation.") and isinstance(DEFAULTS[k], float)}
    positive |= {k for k in DEFAULTS if k.endswith(("tolerance", "control_frequency", "resample_ratio"))}
    nonnegative |= {k for k in DEFAULTS if k.endswith("lookahead_time")}
    positive.discard("path_planner.default_tolerance")
    positive |= {T + "los." + k for k in ("trajectory_time_interval", "trajectory_time", "switch_dist", "goal_tolerance", "lookahead_distance", "yaw_rate_max", "slowdown_distance")}
    positive |= {C + "lqr_controller." + k for k in ("discrete_dt", "solve_eps", "solve_timeout")}
    positive.add(T + "los.kp_yaw")
    if any(values[k] < 0 for k in nonnegative) or any(values[k] <= 0 for k in positive):
        raise ValueError("Gains must be nonnegative; frequencies, times and geometric limits must be positive")
    if values[T + "los.trajectory_time"] < values[T + "los.trajectory_time_interval"]:
        raise ValueError("Trajectory horizon must cover at least one output interval")
    for suffix, length, strictly_positive in (("Q_matrix_diag", 6, False), ("R_matrix_diag", 2, True)):
        matrix = values[C + "lqr_controller." + suffix]
        if len(matrix) != length or any(v < 0 or (strictly_positive and v == 0) for v in matrix):
            raise ValueError(f"Invalid {suffix}")
    USVModel.from_mapping(section(values, "dynamics."))
    return values


# 从完整默认值生成已验证快照；未知参数直接报错以暴露配置拼写错误。
def resolve_parameters(values):
    values = normalize_parameters(values)
    unknown = set(values) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"Unsupported navigation parameters: {sorted(unknown)}")
    result = deepcopy(DEFAULTS)
    result.update(values)
    validate_parameters(result)
    # ROS doubles must remain doubles even when project YAML used an integer.
    return {k: float(v) if isinstance(DEFAULTS[k], float) else v for k, v in result.items()}


# 按前缀提取某个算法的配置，并去掉上层命名空间。
def section(values, prefix):
    return {k[len(prefix):]: v for k, v in values.items() if k.startswith(prefix)}


# 使用拒绝重复键的安全加载器读取工程配置；ROS 参数导出快照不能直接混用。
def load_yaml(path):
    class UniqueKeyLoader(yaml.SafeLoader):
        pass

    def unique_mapping(loader, node, deep=False):
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise ValueError(f"Duplicate YAML key {key!r} in {path}")
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping

    UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.load(stream, Loader=UniqueKeyLoader)
    if not isinstance(result, dict):
        raise ValueError(f"Expected project YAML mapping: {path}")
    if any(isinstance(v, dict) and "ros__parameters" in v for v in result.values()):
        raise ValueError("ROS parameter dumps are snapshots, not project configuration files")
    return result


# 依次合并动力学、控制、轨迹、路径与可选任务参数，地理原点由启动入口单独提供。
def load_project_parameters(dynamics_path, controller_path, trajectory_path, planner_path, mission_path=None):
    values = flatten(load_yaml(dynamics_path))
    for path in (controller_path, trajectory_path):
        merge_explicit(values, flatten(load_yaml(path), "usv_simulation"))
    merge_explicit(values, flatten(load_yaml(planner_path)))
    if mission_path is not None:
        mission = load_yaml(mission_path)
        if set(mission) != {"mission"} or not isinstance(mission["mission"], dict):
            raise ValueError("Mission control YAML must contain only a mission mapping")
        flattened = flatten(mission)
        if any(key.startswith("mission.geo.") for key in flattened):
            raise ValueError("Mission geographic reference is read from the selected world, not mission YAML")
        merge_explicit(values, flattened)
    return resolve_parameters(values)


# 仅允许白名单增益在线修改；先验证候选副本，失败时保留当前有效配置。
def validate_update(current, changes):
    forbidden = set(changes) - HOT_PARAMETERS
    if forbidden:
        raise ValueError(f"Restart required for: {sorted(forbidden)}")
    candidate = dict(current, **changes)
    validate_parameters(candidate)
    return candidate
