# 任务配置回归：校验旧参数兼容、事务式热更新、LOS 精度边界及地理原点来源。
from pathlib import Path
import math
import pytest
from usv_mission.configuration import C, T, DEFAULTS, load_project_parameters, normalize_parameters, resolve_parameters, validate_update


# 验证：原工程 YAML 可加载，旧别名兼容且 LQR 矩阵转换为浮点数组。
def test_original_configuration_loads_and_aliases_work():
    base = Path(__file__).resolve().parents[3] / "shared/usv_navigation/config"
    p = load_project_parameters(base/"robots/otter/dynamics_otter.yaml", base/"robots/otter/controller_params_otter.yaml", base/"trajectory_planner/trajectory_planner_params.yaml", base/"path_planner/common_planner_params.yaml")
    assert p[C+"control_frequency"] == 10.0
    assert p[C+"pid_controller.k_p_speed"] == 2.0
    assert p["obstacle_factor"] == .25
    assert all(type(v) is float for v in p[C+"lqr_controller.Q_matrix_diag"])


# 验证：规范参数名与别名给出冲突值时明确拒绝。
def test_alias_conflicts_rejected():
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({C+"control_frequncy": 10., C+"controller_frequncy": 20.})
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({"obstacle_factor": .5, "path_planner.obstacle_factor": .9})


# 验证：无效更新不改写当前参数，合法更新返回新的候选配置。
def test_invalid_update_is_transactional():
    p = resolve_parameters({})
    with pytest.raises(ValueError):
        validate_update(p, {C+"pid_controller.k_p_speed": 4., T+"los.lookahead_distance": -1.})
    assert p[C+"pid_controller.k_p_speed"] == 2.
    updated = validate_update(p, {C+"pid_controller.k_p_speed": 4.})
    assert updated[C+"pid_controller.k_p_speed"] == 4.
    with pytest.raises(ValueError, match="Restart"):
        validate_update(p, {C+"control_frequency": 20.})


# 验证：不合法的启动参数组合被统一校验拒绝。
@pytest.mark.parametrize("changes", [{C+"pid_controller.k_p_speed": float("nan")}, {C+"controller_name": "mpc"}, {C+"lqr_controller.R_matrix_diag": [0.,1.]}, {C+"lqr_controller.Q_matrix_diag": [1.]}, {"navigation.max_expansions": 1.5}])
def test_invalid_startup(changes):
    with pytest.raises(ValueError):
        resolve_parameters(changes)


# 验证：LQR 可使用当前时刻参考，但 lookahead_time 不能在线修改。
def test_current_time_lqr_reference_is_valid_but_not_hot_mutable():
    key = C + 'lqr_controller.lookahead_time'
    values = resolve_parameters({key: 0.})
    assert values[key] == 0.
    with pytest.raises(ValueError):
        resolve_parameters({key: -.1})
    with pytest.raises(ValueError, match='Restart'):
        validate_update(values, {key: .5})


# 验证：控制器目标半径过小、无法支持 LOS 有效精度时拒绝启动。
@pytest.mark.parametrize("name", ["pid", "lqr"])
@pytest.mark.parametrize("tolerance", [0., 1e-10, 1e-9])
def test_startup_goal_distance_tolerance_rejects_unresolvable_los_radius(name, tolerance):
    key = C + name + "_controller.goal_distance_tolerance"
    with pytest.raises(ValueError, match=key + r".*greater than.*1e-9"):
        resolve_parameters({key: tolerance})


# 验证：最小允许的目标半径原值得到保留。
@pytest.mark.parametrize("name", ["pid", "lqr"])
def test_startup_preserves_smallest_admitted_goal_radius(name):
    key = C + name + "_controller.goal_distance_tolerance"
    tolerance = math.nextafter(1e-9, math.inf)
    assert resolve_parameters({key: tolerance})[key] == tolerance


# 验证：工程 YAML 中低于 LOS 精度的目标半径同样被拒绝。
@pytest.mark.parametrize("name", ["pid", "lqr"])
def test_project_yaml_rejects_goal_radius_below_los_precision(tmp_path, name):
    controller = tmp_path / "tiny_goal_controller.yaml"
    controller.write_text("controller:\n  " + name + "_controller:\n"
                          "    goal_distance_tolerance: 1.0e-10\n", encoding="utf-8")
    base = Path(__file__).resolve().parents[3] / "shared/usv_navigation/config"
    key = C + name + "_controller.goal_distance_tolerance"
    with pytest.raises(ValueError, match=key + r".*greater than.*1e-9"):
        load_project_parameters(base/"robots/otter/dynamics_otter.yaml", controller,
                                base/"trajectory_planner/trajectory_planner_params.yaml",
                                base/"path_planner/common_planner_params.yaml")


# 验证：地理坐标允许有符号数值，但拒绝范围外经纬度。
def test_mission_coordinates_accept_signed_values_and_reject_invalid_ranges():
    values = resolve_parameters({"mission.geo.enabled": True, "mission.geo.latitude": -33.7242,
                                 "mission.geo.longitude": -150.68, "mission.geo.altitude": -10})
    assert values["mission.geo.latitude"] == -33.7242
    assert values["mission.geo.altitude"] == -10.0
    for changes in ({"mission.geo.latitude": 91}, {"mission.geo.longitude": -181},
                    {"mission.heading_tolerance_deg": 0}, {"mission.safety_margin": -0.1}):
        with pytest.raises(ValueError):
            resolve_parameters(changes)


# 验证：任务调参 YAML 可省略，并且不能覆盖从世界读取的地理原点。
def test_mission_yaml_is_optional_and_cannot_override_world_reference(tmp_path):
    base = Path(__file__).resolve().parents[3] / "shared/usv_navigation/config"
    paths = [base/"robots/otter/dynamics_otter.yaml", base/"robots/otter/controller_params_otter.yaml",
             base/"trajectory_planner/trajectory_planner_params.yaml", base/"path_planner/common_planner_params.yaml"]
    values = load_project_parameters(*paths, Path(__file__).resolve().parents[1]/"config/mission/mission_params.yaml")
    assert values["mission.heading_tolerance_deg"] == 5.0
    assert values["mission.geo.enabled"] is False
    path = tmp_path / "mission.yaml"
    path.write_text("mission:\n  geo:\n    latitude: 0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="selected world"):
        load_project_parameters(*paths, path)
    path.write_text("mode: standby\n", encoding="utf-8")
    with pytest.raises(ValueError, match="mission mapping"):
        load_project_parameters(*paths, path)
