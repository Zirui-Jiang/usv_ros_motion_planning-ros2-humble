from pathlib import Path
import pytest
from sim_env.configuration import C, T, DEFAULTS, load_project_parameters, normalize_parameters, resolve_parameters, validate_update


def test_original_configuration_loads_and_aliases_work():
    base = Path(__file__).resolve().parents[1] / "config"
    p = load_project_parameters(base/"robots/otter/dynamics_otter.yaml", base/"robots/otter/controller_params_otter.yaml", base/"trajectory_planner/trajectory_planner_params.yaml", base/"path_planner/common_planner_params.yaml")
    assert p[C+"control_frequency"] == 10.0
    assert p[C+"pid_controller.k_p_speed"] == 2.0
    assert p["obstacle_factor"] == .5
    assert all(type(v) is float for v in p[C+"lqr_controller.Q_matrix_diag"])


def test_alias_conflicts_rejected():
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({C+"control_frequncy": 10., C+"controller_frequncy": 20.})
    with pytest.raises(ValueError, match="Conflicting"):
        normalize_parameters({"obstacle_factor": .5, "path_planner.obstacle_factor": .9})


def test_invalid_update_is_transactional():
    p = resolve_parameters({})
    with pytest.raises(ValueError):
        validate_update(p, {C+"pid_controller.k_p_speed": 4., T+"los.lookahead_distance": -1.})
    assert p[C+"pid_controller.k_p_speed"] == 2.
    updated = validate_update(p, {C+"pid_controller.k_p_speed": 4.})
    assert updated[C+"pid_controller.k_p_speed"] == 4.
    with pytest.raises(ValueError, match="Restart"):
        validate_update(p, {C+"control_frequency": 20.})


@pytest.mark.parametrize("changes", [{C+"pid_controller.k_p_speed": float("nan")}, {C+"controller_name": "mpc"}, {C+"lqr_controller.R_matrix_diag": [0.,1.]}, {C+"lqr_controller.Q_matrix_diag": [1.]}, {"navigation.max_expansions": 1.5}])
def test_invalid_startup(changes):
    with pytest.raises(ValueError):
        resolve_parameters(changes)


def test_current_time_lqr_reference_is_valid_but_not_hot_mutable():
    key = C + 'lqr_controller.lookahead_time'
    values = resolve_parameters({key: 0.})
    assert values[key] == 0.
    with pytest.raises(ValueError):
        resolve_parameters({key: -.1})
    with pytest.raises(ValueError, match='Restart'):
        validate_update(values, {key: .5})
