# 验证船型配置：Otter 与 WAM-V 的项目 YAML 都能解析，推进形式、横向控制器与船体尺寸随船型一致。
"""Per-vessel project YAML: thruster layout, lateral loop and footprint stay consistent."""
from pathlib import Path

import pytest

from common.models import USVModel
from controller.lateral_thrust import LateralThrust
from controller.thruster_output import ThrusterOutput
from usv_navigation.configuration import (
    C, HOT_PARAMETERS, load_project_parameters, resolve_parameters, section, validate_update)

CONFIG = Path(__file__).resolve().parents[1] / "config"
LATERAL = C + "lateral_controller."
WAMV = {"dynamics.name": "wamv", "dynamics.type": "Omni", "dynamics.max_lateral_force": 250.0}


def project_parameters(robot):
    return load_project_parameters(
        CONFIG / f"robots/{robot}/dynamics_{robot}.yaml", CONFIG / f"robots/{robot}/controller_params_{robot}.yaml",
        CONFIG / "trajectory_planner/trajectory_planner_params.yaml",
        CONFIG / "path_planner/common_planner_params.yaml")


# WAM-V 项目 YAML：两台全回转桨的 Azimuth 船型、横向环启用、船体尺寸随船模给出。
def test_wamv_project_yaml_resolves_with_azimuth_thrusters():
    values = project_parameters("wamv")
    model = USVModel.from_mapping(section(values, "dynamics."))
    assert (model.name, model.type) == ("wamv", "Azimuth")
    # 横向力归一化基准 200 N；回转轴在转动中心后 2.374 m，用于补偿横向力带出的转艏力矩。
    assert (model.max_force, model.max_lateral_force, model.thruster_x) == (250.0, 200.0, -2.374)
    # 附加质量显式为零，不沿用 Otter 默认值。
    assert (model.x_dot_u, model.y_dot_v, model.n_dot_r) == (0.0, 0.0, 0.0)
    assert (values["navigation.footprint_length"], values["navigation.footprint_width"]) == (5.7, 2.5)
    lateral = LateralThrust(model, section(values, LATERAL))
    assert lateral.enabled and lateral.max_force == 200.0
    # 两台桨的转速上限覆盖 250 N；横向力由推力输出口按全回转方式分给两台桨。
    output = ThrusterOutput(model, section(values, C + "thrust_allocator."))
    assert output.azimuth and output.has_lateral


# Otter 项目 YAML 行为不变：差动船型、没有侧推器、横向环关闭。
def test_otter_project_yaml_keeps_differential_defaults():
    values = project_parameters("otter")
    assert (values["dynamics.name"], values["dynamics.type"]) == ("otter", "Differential")
    assert values["dynamics.max_lateral_force"] == 0.0
    assert values[LATERAL + "enabled"] is False
    assert (values["navigation.footprint_length"], values["navigation.footprint_width"]) == (2.2, 1.2)
    defaults = resolve_parameters({})
    assert defaults[LATERAL + "enabled"] is False and defaults[LATERAL + "k_p_lateral"] == 4.0


@pytest.mark.parametrize("overrides, message", [
    # 没有侧推器的船型不能启用横向环。
    ({LATERAL + "enabled": True}, "requires a vessel that can produce lateral force"),
    # Azimuth 和 Omni 必须给出横向力基准，Differential 不得给出。
    ({"dynamics.name": "wamv", "dynamics.type": "Azimuth"}, "max_lateral_force"),
    ({"dynamics.name": "wamv", "dynamics.type": "Omni"}, "max_lateral_force"),
    ({"dynamics.max_lateral_force": 10.0}, "max_lateral_force"),
    ({**WAMV, "dynamics.max_lateral_force": -1.0}, "max_lateral_force"),
    # Otter 的仿真模型没有侧推器。
    ({"dynamics.type": "Omni", "dynamics.max_lateral_force": 10.0}, "Supported vessel models"),
    ({"dynamics.name": "catamaran"}, "Supported vessel models"),
    # 横向力限幅不得超过侧推器。
    ({**WAMV, LATERAL + "enabled": True, LATERAL + "force_max": 300.0}, "stay within"),
    ({**WAMV, LATERAL + "enabled": "true"}, "Invalid type"),
])
def test_inconsistent_vessel_and_lateral_settings_are_rejected(overrides, message):
    with pytest.raises(ValueError, match=message):
        resolve_parameters(overrides)


# WAM-V 也可以按差动船型使用（不启用侧推器）。
def test_wamv_may_run_as_differential_without_lateral_force():
    values = resolve_parameters({"dynamics.name": "wamv"})
    assert values["dynamics.type"] == "Differential" and values[LATERAL + "enabled"] is False


# 横向控制器参数不是热参数，在线修改要求重启。
def test_lateral_controller_parameters_are_not_hot():
    assert not any(key.startswith(LATERAL) for key in HOT_PARAMETERS)
    values = project_parameters("wamv")
    with pytest.raises(ValueError, match="Restart required"):
        validate_update(values, {LATERAL + "k_p_lateral": 80.0})
    with pytest.raises(ValueError, match="Restart required"):
        validate_update(values, {LATERAL + "enabled": False})
