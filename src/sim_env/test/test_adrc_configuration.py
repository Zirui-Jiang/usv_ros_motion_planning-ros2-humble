from pathlib import Path

import pytest
import yaml

from sim_env.configuration import C, HOT_PARAMETERS, load_project_parameters, resolve_parameters, validate_update


CONFIG = Path(__file__).resolve().parents[2] / "core/shared/usv_navigation/config"
ADRC = C + "adrc_controller."


def test_adrc_defaults_resolve_and_remain_startup_only():
    values = resolve_parameters({C + "controller_name": "adrc"})

    assert values[ADRC + "sample_time"] == 0.1
    assert values[ADRC + "max_sample_gap"] == 0.5
    assert values[ADRC + "k_p_speed"] == 8.0
    assert values[ADRC + "heading_integral_max"] == 0.25
    assert values[ADRC + "force_min"] == -8.0
    assert values[ADRC + "force_max"] == 8.0
    assert values[ADRC + "moment_min"] == -2.0
    assert values[ADRC + "moment_max"] == 2.0
    assert not any(key.startswith(ADRC) for key in HOT_PARAMETERS)
    with pytest.raises(ValueError, match="Restart"):
        validate_update(values, {ADRC + "k_p_speed": 9.0})


def test_dedicated_adrc_yaml_preserves_existing_controllers_and_selects_adrc():
    base = CONFIG
    original = yaml.safe_load((base / "robots/otter/controller_params_otter.yaml").read_text(encoding="utf-8"))
    dedicated_path = base / "robots/otter/adrc_controller_params_otter.yaml"
    dedicated = yaml.safe_load(dedicated_path.read_text(encoding="utf-8"))

    assert dedicated["controller"]["controller_name"] == "adrc"
    assert dedicated["controller"]["pid_controller"] == original["controller"]["pid_controller"]
    assert dedicated["controller"]["lqr_controller"] == original["controller"]["lqr_controller"]
    values = load_project_parameters(
        base / "robots/otter/dynamics_otter.yaml",
        dedicated_path,
        base / "trajectory_planner/trajectory_planner_params.yaml",
        base / "path_planner/common_planner_params.yaml",
    )
    assert values[C + "controller_name"] == "adrc"
    assert values[ADRC + "filter_history_length"] == 50
    assert values[ADRC + "force_min"] == -40.0
    assert values[ADRC + "force_max"] == 40.0
    assert values[ADRC + "speed_integral_min"] == -30.0
    assert values[ADRC + "speed_integral_max"] == 30.0
    assert values[ADRC + "moment_min"] == -10.0
    assert values[ADRC + "moment_max"] == 10.0
    assert values[ADRC + "heading_integral_min"] == -2.0
    assert values[ADRC + "heading_integral_max"] == 2.0


@pytest.mark.parametrize("suffix,value", [
    ("sample_time", 0.0),
    ("max_sample_gap", 0.0),
    ("command_limit", 0.0),
    ("command_limit", 1.01),
    ("k_p_heading", -0.01),
    ("error_reset_deg", 0.0),
    ("error_reset_deg", 180.01),
    ("filter_window_length", 1),
])
def test_adrc_scalar_bounds_are_rejected(suffix, value):
    with pytest.raises(ValueError):
        resolve_parameters({ADRC + suffix: value})


@pytest.mark.parametrize("changes", [
    {ADRC + "force_min": 1.0},
    {ADRC + "force_max": -1.0},
    {ADRC + "force_min": 2.0, ADRC + "force_max": 1.0},
    {ADRC + "speed_integral_min": 1.0},
    {ADRC + "heading_integral_max": -0.1},
    {ADRC + "filter_window_length": 11, ADRC + "filter_history_length": 10},
])
def test_adrc_ordered_ranges_and_filter_history_are_rejected(changes):
    with pytest.raises(ValueError):
        resolve_parameters(changes)


def test_selected_adrc_sample_time_matches_control_frequency():
    resolve_parameters({C + "controller_name": "adrc", ADRC + "sample_time": 0.1 + 1e-14})
    with pytest.raises(ValueError, match="sample_time.*control_frequency"):
        resolve_parameters({C + "controller_name": "adrc", ADRC + "sample_time": 0.2})


def test_unselected_adrc_sample_time_does_not_constrain_existing_controllers():
    values = resolve_parameters({C + "controller_name": "pid", ADRC + "sample_time": 0.2})
    assert values[ADRC + "sample_time"] == 0.2
