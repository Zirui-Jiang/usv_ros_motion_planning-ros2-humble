# 验证推力分配参数随项目 YAML 加载、只能重启生效，且转速上限必须覆盖船模单桨最大推力。
"""Thrust allocator settings travel with the shared controller configuration."""
from pathlib import Path

import pytest

from usv_navigation.configuration import C, HOT_PARAMETERS, load_project_parameters, resolve_parameters, validate_update

CONFIG = Path(__file__).resolve().parents[1] / "config"
PREFIX = C + "thrust_allocator."


def project_parameters():
    return load_project_parameters(
        CONFIG / "robots/otter/dynamics_otter.yaml", CONFIG / "robots/otter/controller_params_otter.yaml",
        CONFIG / "trajectory_planner/trajectory_planner_params.yaml",
        CONFIG / "path_planner/common_planner_params.yaml")


# 项目 YAML 的 ±537 rpm 覆盖 180 N；默认值沿用 0924 交付代码。
def test_project_yaml_and_defaults_resolve():
    values = project_parameters()
    assert values["dynamics.max_force"] == 180.0
    assert values[PREFIX + "left_rpm_max"] == 537.0 and values[PREFIX + "right_rpm_min"] == -537.0
    defaults = resolve_parameters({})
    assert defaults[PREFIX + "thrust_coefficient"] == .5 and defaults[PREFIX + "left_rpm_max"] == 5000.0


# 转速上限对应的推力低于 max_force 时，整份配置在启动时被拒绝。
def test_rpm_limit_below_max_force_is_rejected():
    with pytest.raises(ValueError, match="rpm limits reach only"):
        resolve_parameters({"dynamics.max_force": 180.0, PREFIX + "right_rpm_max": 500.0})


# 推力分配参数不是热参数，在线修改要求重启。
def test_thrust_allocator_parameters_are_not_hot():
    assert not any(key.startswith(PREFIX) for key in HOT_PARAMETERS)
    values = project_parameters()
    with pytest.raises(ValueError, match="Restart required"):
        validate_update(values, {PREFIX + "left_rpm_max": 600.0})
