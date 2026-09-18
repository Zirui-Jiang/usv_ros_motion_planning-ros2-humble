# 验证项目 YAML 加载、旧参数别名、非法值拒绝和原子更新，避免部分参数生效。
from pathlib import Path
import math
import pytest
from sim_env.configuration import C, T, DEFAULTS, HOT_PARAMETERS, load_project_parameters, normalize_parameters, resolve_parameters, validate_update

CONFIG = Path(__file__).resolve().parents[2] / "core/shared/usv_navigation/config"


# 验证既有项目配置可加载，旧别名与规范参数保持一致。
def test_original_configuration_loads_and_aliases_work():
    base = CONFIG
    p = load_project_parameters(base/"robots/otter/dynamics_otter.yaml", base/"robots/otter/controller_params_otter.yaml", base/"trajectory_planner/trajectory_planner_params.yaml", base/"path_planner/common_planner_params.yaml")
    assert p[C+"control_frequency"] == 10.0
    assert p[C+"pid_controller.k_p_speed"] == 2.0
    assert p["obstacle_factor"] == .05
    assert all(type(v) is float for v in p[C+"lqr_controller.Q_matrix_diag"])


def test_station_adrc_defaults_are_resolved_and_sample_time_tracks_control_frequency():
    p = resolve_parameters({})
    prefix = C + "station_adrc_controller."
    assert p[prefix + "position_tolerance"] == .5
    assert p[prefix + "command_limit"] == .4
    assert p[prefix + "sample_time"] == .1
    assert prefix + "k_p_speed" not in HOT_PARAMETERS
    with pytest.raises(ValueError, match="station ADRC sample_time"):
        resolve_parameters({C + "control_frequency": 20., prefix + "sample_time": .1})


@pytest.mark.parametrize("key", ["position_tolerance", "force_min", "moment_max", "filter_window_length"])
def test_station_adrc_invalid_ranges_are_rejected(key):
    prefix = C + "station_adrc_controller."
    invalid = {
        "position_tolerance": 0., "force_min": 1., "moment_max": -1., "filter_window_length": 1,
    }
    with pytest.raises(ValueError):
        resolve_parameters({prefix + key: invalid[key]})


# 已移除的传感器参数必须明确拒绝，不能表面接受但实际不生效。
@pytest.mark.parametrize("removed", [
    {"navigation.require_radar": True},
    {"navigation.radar_timeout": 0.5},
])
def test_removed_sensor_parameters_are_rejected_instead_of_silently_accepted(removed):
    with pytest.raises(ValueError, match="Unsupported navigation parameters"):
        resolve_parameters(removed)


# 同一含义的别名给出冲突值时拒绝启动或更新。
def test_alias_conflicts_rejected():
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({C+"control_frequncy": 10., C+"controller_frequncy": 20.})
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({"obstacle_factor": .5, "path_planner.obstacle_factor": .9})


# 一项参数非法时整批更新失败，原有效参数不发生部分变化。
def test_invalid_update_is_transactional():
    p = resolve_parameters({})
    with pytest.raises(ValueError):
        validate_update(p, {C+"pid_controller.k_p_speed": 4., T+"los.lookahead_distance": -1.})
    assert p[C+"pid_controller.k_p_speed"] == 2.
    updated = validate_update(p, {C+"pid_controller.k_p_speed": 4.})
    assert updated[C+"pid_controller.k_p_speed"] == 4.
    with pytest.raises(ValueError, match="Restart"):
        validate_update(p, {C+"control_frequency": 20.})


# 检查非法启动配置在进入运行流程前被拒绝。
@pytest.mark.parametrize("changes", [{C+"pid_controller.k_p_speed": float("nan")}, {C+"controller_name": "mpc"}, {C+"lqr_controller.R_matrix_diag": [0.,1.]}, {C+"lqr_controller.Q_matrix_diag": [1.]}, {"navigation.max_expansions": 1.5}])
def test_invalid_startup(changes):
    with pytest.raises(ValueError):
        resolve_parameters(changes)


# 当前时刻参考可用于 LQR，但该配置不允许按热参数随意切换。
def test_current_time_lqr_reference_is_valid_but_not_hot_mutable():
    key = C + 'lqr_controller.lookahead_time'
    values = resolve_parameters({key: 0.})
    assert values[key] == 0.
    with pytest.raises(ValueError):
        resolve_parameters({key: -.1})
    with pytest.raises(ValueError, match='Restart'):
        validate_update(values, {key: .5})


# 到点半径不能小于 LOS 可分辨的精度，否则启动校验必须失败。
@pytest.mark.parametrize("name", ["pid", "lqr"])
@pytest.mark.parametrize("tolerance", [0., 1e-10, 1e-9])
def test_startup_goal_distance_tolerance_rejects_unresolvable_los_radius(name, tolerance):
    key = C + name + "_controller.goal_distance_tolerance"
    with pytest.raises(ValueError, match=key + r".*greater than.*1e-9"):
        resolve_parameters({key: tolerance})


# 最小可接受到点半径应原值保留，不能被静默放大。
@pytest.mark.parametrize("name", ["pid", "lqr"])
def test_startup_preserves_smallest_admitted_goal_radius(name):
    key = C + name + "_controller.goal_distance_tolerance"
    tolerance = math.nextafter(1e-9, math.inf)
    assert resolve_parameters({key: tolerance})[key] == tolerance


# 直接从项目 YAML 读取时也必须执行同一到点半径精度检查。
@pytest.mark.parametrize("name", ["pid", "lqr"])
def test_project_yaml_rejects_goal_radius_below_los_precision(tmp_path, name):
    controller = tmp_path / "tiny_goal_controller.yaml"
    controller.write_text("controller:\n  " + name + "_controller:\n"
                          "    goal_distance_tolerance: 1.0e-10\n", encoding="utf-8")
    base = CONFIG
    key = C + name + "_controller.goal_distance_tolerance"
    with pytest.raises(ValueError, match=key + r".*greater than.*1e-9"):
        load_project_parameters(base/"robots/otter/dynamics_otter.yaml", controller,
                                base/"trajectory_planner/trajectory_planner_params.yaml",
                                base/"path_planner/common_planner_params.yaml")
