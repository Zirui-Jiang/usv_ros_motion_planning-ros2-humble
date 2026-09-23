# 统一项目 YAML 与 ROS 参数快照：展开层级、处理兼容别名、验证物理约束并限制在线可改项。
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

DEFAULTS = {
    "planner_name": "astar", "obstacle_factor": 0.25,
    "path_planner.outline_map": True, "path_planner.enable_resample": True,
    "path_planner.enable_shortcut": True,
    "path_planner.resample_ratio": 0.5, "path_planner.convert_offset": 0.0,
    "path_planner.default_tolerance": 0.0, "path_planner.expand_zone": True,
    "path_planner.show_safety_corridor": False,
    "path_planner.minimum_turning_radius": 4.0,
    "path_planner.turning_radius_multipliers": [1.0],
    "path_planner.angle_bins": 72,
    "path_planner.heuristic_weight": 1.2,
    "path_planner.reuse_max_deviation": 2.0,
    "path_planner.reuse_max_heading_error": math.pi / 3.,
    "path_planner.reuse_search_distance": 10.0,
    "path_planner.motion_step": 1.0, "path_planner.collision_step": 0.2,
    "path_planner.analytic_expansion_max_length": 20.0,
    "path_planner.non_straight_penalty": 1.05, "path_planner.change_penalty": 0.2,
    "path_planner.cost_penalty": 0.0, "path_planner.footprint_padding": 0.0,
    "navigation.planning_frequency": 5.0,
    "navigation.state_timeout": 0.5,
    "navigation.map_timeout": 1.5, "navigation.trajectory_timeout": 1.0,
    "navigation.planning_timeout": 1.0, "navigation.max_expansions": 500000,
    "navigation.costmap_topic": "costmap_raw",
    "navigation.footprint_length": 2.2, "navigation.footprint_width": 1.2,
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
    C + "adrc_controller.goal_distance_tolerance": 1.5,
    C + "adrc_controller.goal_speed_tolerance": 0.25,
    C + "adrc_controller.lookahead_time": 2.6,
    C + "adrc_controller.command_limit": 1.0,
    C + "adrc_controller.sample_time": 0.1,
    C + "adrc_controller.max_sample_gap": 0.5,
    C + "adrc_controller.k_p_speed": 8.0,
    C + "adrc_controller.k_i_speed": 0.8,
    C + "adrc_controller.force_min": -8.0,
    C + "adrc_controller.force_max": 8.0,
    C + "adrc_controller.speed_integral_min": -8.0,
    C + "adrc_controller.speed_integral_max": 8.0,
    C + "adrc_controller.k_p_heading": 1.5,
    C + "adrc_controller.k_d_heading": 0.5,
    C + "adrc_controller.k_i_heading": 0.03,
    C + "adrc_controller.moment_min": -10.0,
    C + "adrc_controller.moment_max": 10.0,
    C + "adrc_controller.heading_integral_min": -0.25,
    C + "adrc_controller.heading_integral_max": 0.25,
    C + "adrc_controller.error_reset_deg": 20.0,
    C + "adrc_controller.acceleration_gain": 0.0,
    C + "adrc_controller.filter_window_length": 10,
    C + "adrc_controller.filter_history_length": 50,
    C + "station_adrc_controller.position_tolerance": .5,
    C + "station_adrc_controller.heading_tolerance_deg": 5.,
    C + "station_adrc_controller.position_speed_gain": .5,
    C + "station_adrc_controller.recover_speed": .3,
    C + "station_adrc_controller.recover_speed_tolerance": .04,
    C + "station_adrc_controller.alignment_time": 1.,
    C + "station_adrc_controller.alignment_yaw_rate": .02,
    C + "station_adrc_controller.free_heading_yaw_damping": 10.,
    C + "station_adrc_controller.k_p_speed": 30.,
    C + "station_adrc_controller.k_i_speed": .8,
    C + "station_adrc_controller.force_min": -20.,
    C + "station_adrc_controller.force_max": 20.,
    C + "station_adrc_controller.speed_integral_min": -8.,
    C + "station_adrc_controller.speed_integral_max": 8.,
    C + "station_adrc_controller.k_p_heading": 12.,
    C + "station_adrc_controller.k_d_heading": 10.,
    C + "station_adrc_controller.k_i_heading": .03,
    C + "station_adrc_controller.moment_min": -8.,
    C + "station_adrc_controller.moment_max": 8.,
    C + "station_adrc_controller.heading_integral_min": -.25,
    C + "station_adrc_controller.heading_integral_max": .25,
    C + "station_adrc_controller.sample_time": .1,
    C + "station_adrc_controller.max_sample_gap": .5,
    C + "station_adrc_controller.error_reset_deg": 20.,
    C + "station_adrc_controller.acceleration_gain": 0.,
    C + "station_adrc_controller.filter_window_length": 10,
    C + "station_adrc_controller.filter_history_length": 50,
    C + "station_adrc_controller.command_limit": .4,
    C + "station_adrc_controller.command_slew_rate": .6,
    T + "planner_name": "los",
    T + "los.trajectory_time_interval": 0.5, T + "los.trajectory_time": 20.0,
    T + "los.switch_dist": 5.0, T + "los.goal_tolerance": 1.0,
    T + "los.lookahead_distance": 1.0, T + "los.yaw_rate_max": 0.3,
    T + "los.kp_yaw": 1.0,
    T + "los.slowdown_distance": 3.0,
}
DEFAULTS.update({"dynamics." + k: v for k, v in asdict(USVModel()).items()})
# 允许在线改动的是 PID 六个增益与 LOS 四个导引参数；模型、频率、容差等保持启动时值。
HOT_PARAMETERS = frozenset(
    C + "pid_controller." + k for k in
    ("k_p_speed", "k_i_speed", "k_d_speed", "k_p_course", "k_i_course", "k_d_course")
) | frozenset(T + "los." + k for k in
              ("lookahead_distance", "switch_dist", "kp_yaw", "yaw_rate_max"))


# 合并显式参数时若同名值不一致立即报错，避免后载入配置悄悄覆盖前者。
def merge_explicit(target, source):
    for key, value in source.items():
        if key in target and target[key] != value:
            raise ValueError(f"Conflicting configuration sources for {key}")
        target[key] = value
    return target


# 把嵌套映射转成点分隔参数键，同时校验键类型与重复参数。
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


# 统一历史拼写别名，LQR 整数列表转双精度数组，满足 ROS 参数声明类型。
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
    for key in (C + "lqr_controller.Q_matrix_diag", C + "lqr_controller.R_matrix_diag",
                "path_planner.turning_radius_multipliers"):
        if key in result:
            value = result[key]
            if not isinstance(value, list) or any(type(v) not in (int, float) for v in value):
                raise ValueError(f"{key} must be a numeric array")
            result[key] = [float(v) for v in value]
    return result


# 检查类型、有限性、枚举、正负范围、轨迹时间关系和 Q/R 维度，再验证船舶模型。
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
    for name in ("pid", "lqr", "adrc"):
        key = C + name + "_controller.goal_distance_tolerance"
        validate_goal_distance_tolerance(values[key], key)
    if values["planner_name"] not in ("astar", "dijkstra", "gbfs", "hybrid_astar"):
        raise ValueError("Only astar, dijkstra, gbfs and hybrid_astar are implemented")
    if values[C + "controller_name"] not in ("pid", "lqr", "adrc") or values[T + "planner_name"] != "los":
        raise ValueError("Only PID/LQR/ADRC controllers and LOS trajectory planner are implemented")
    if not 0 < values["obstacle_factor"] <= 1:
        raise ValueError("obstacle_factor must be in (0,1]")
    if values["path_planner.angle_bins"] < 8:
        raise ValueError("path_planner.angle_bins must be an integer >= 8")
    if values["path_planner.non_straight_penalty"] < 1:
        raise ValueError("path_planner.non_straight_penalty must be >= 1")
    if values["path_planner.heuristic_weight"] < 1:
        raise ValueError("path_planner.heuristic_weight must be >= 1")
    multipliers = values["path_planner.turning_radius_multipliers"]
    if (not isinstance(multipliers, list) or not multipliers
            or any(type(v) not in (int, float) or not math.isfinite(v) or v < 1 for v in multipliers)):
        raise ValueError("path_planner.turning_radius_multipliers must be a nonempty list of finite numbers >= 1")
    if not 0 < values["path_planner.reuse_max_heading_error"] <= math.pi:
        raise ValueError("path_planner.reuse_max_heading_error must be in (0, pi]")
    nonnegative = {k for k in HOT_PARAMETERS if ".k_" in k}
    nonnegative |= {"path_planner." + k for k in
                    ("change_penalty", "cost_penalty", "footprint_padding")}
    positive = {k for k in DEFAULTS if k.startswith("navigation.") and isinstance(DEFAULTS[k], float)}
    positive |= {"path_planner." + k for k in
                 ("minimum_turning_radius", "motion_step", "collision_step", "analytic_expansion_max_length",
                  "reuse_max_deviation", "reuse_search_distance")}
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
    adrc = C + "adrc_controller."
    if values[adrc + "sample_time"] <= 0 or values[adrc + "max_sample_gap"] <= 0:
        raise ValueError("ADRC sample_time and max_sample_gap must be positive")
    if values[C + "controller_name"] == "adrc" and not math.isclose(
            values[adrc + "sample_time"], 1.0 / values[C + "control_frequency"],
            rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError("ADRC sample_time must equal the reciprocal of control_frequency")
    if not 0 < values[adrc + "command_limit"] <= 1:
        raise ValueError("ADRC command_limit must be in (0,1]")
    for suffix in ("k_p_speed", "k_i_speed", "k_p_heading", "k_d_heading",
                   "k_i_heading", "acceleration_gain"):
        if values[adrc + suffix] < 0:
            raise ValueError(f"ADRC {suffix} must be nonnegative")
    if not 0 < values[adrc + "error_reset_deg"] <= 180:
        raise ValueError("ADRC error_reset_deg must be in (0,180]")
    for prefix in ("force", "speed_integral", "moment", "heading_integral"):
        lower, upper = values[adrc + prefix + "_min"], values[adrc + prefix + "_max"]
        if lower > 0 or upper < 0 or lower >= upper:
            raise ValueError(f"ADRC {prefix} range must be ordered and contain zero")
    window = values[adrc + "filter_window_length"]
    history = values[adrc + "filter_history_length"]
    if window < 2 or history < window:
        raise ValueError("ADRC filter history must be at least the window length, and window must be >= 2")
    station = C + "station_adrc_controller."
    if not math.isclose(values[station + "sample_time"], 1.0 / values[C + "control_frequency"],
                        rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError("station ADRC sample_time must equal the reciprocal of control_frequency")
    if not 0 < values[station + "command_limit"] <= 1 or values[station + "command_slew_rate"] <= 0:
        raise ValueError("Invalid station ADRC command envelope")
    for key in ("position_tolerance", "heading_tolerance_deg", "recover_speed",
                "recover_speed_tolerance", "alignment_time", "max_sample_gap"):
        if values[station + key] <= 0:
            raise ValueError(f"station ADRC {key} must be positive")
    if values[station + "heading_tolerance_deg"] > 180:
        raise ValueError("station ADRC heading_tolerance_deg must be in (0,180]")
    if values[station + "position_speed_gain"] < 0 or values[station + "alignment_yaw_rate"] < 0 \
            or values[station + "free_heading_yaw_damping"] < 0:
        raise ValueError("station ADRC outer-loop gains must be nonnegative")
    for suffix in ("k_p_speed", "k_i_speed", "k_p_heading", "k_d_heading",
                   "k_i_heading", "acceleration_gain"):
        if values[station + suffix] < 0:
            raise ValueError(f"station ADRC {suffix} must be nonnegative")
    for prefix in ("force", "speed_integral", "moment", "heading_integral"):
        lower, upper = values[station + prefix + "_min"], values[station + prefix + "_max"]
        if lower > 0 or upper < 0 or lower >= upper:
            raise ValueError(f"station ADRC {prefix} range must be ordered and contain zero")
    if not 0 < values[station + "error_reset_deg"] <= 180:
        raise ValueError("station ADRC error_reset_deg must be in (0,180]")
    window = values[station + "filter_window_length"]
    history = values[station + "filter_history_length"]
    if window < 2 or history < window or int(window) != window or int(history) != history:
        raise ValueError("station ADRC filter history must be at least the window length, and window must be >= 2")
    USVModel.from_mapping(section(values, "dynamics."))
    return values


# 标准化输入后覆盖默认值并完整验证，最终按默认类型保持 ROS double 参数一致。
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


# 按前缀选参数并剥离前缀，得到单个纯算法可直接消费的配置。
def section(values, prefix):
    return {k[len(prefix):]: v for k, v in values.items() if k.startswith(prefix)}


# 使用禁止重复键的安全 YAML 加载器；ROS 导出快照不接受为项目原始配置。
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


# 按动力学、控制、轨迹、路径配置的约定层级合并，再形成完整有效参数快照。
def load_project_parameters(dynamics_path, controller_path, trajectory_path, planner_path):
    values = flatten(load_yaml(dynamics_path))
    for path in (controller_path, trajectory_path):
        merge_explicit(values, flatten(load_yaml(path), "usv_simulation"))
    merge_explicit(values, flatten(load_yaml(planner_path)))
    return resolve_parameters(values)


# 只接受既有热参数键，先验证整个候选配置再返回，其他改动要求重启。
def validate_update(current, changes):
    forbidden = set(changes) - HOT_PARAMETERS
    if forbidden:
        raise ValueError(f"Restart required for: {sorted(forbidden)}")
    candidate = dict(current, **changes)
    validate_parameters(candidate)
    return candidate
