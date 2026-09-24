"""0924 推力分配接入契约：交付文件未改、零命令、桨力与转速互逆、方向及启动校验。"""
import hashlib
import math
import random
from pathlib import Path

import pytest

from common.dynamics import allocate_force
from common.models import ControlCommand, USVModel
from controller.thruster_output import ThrusterOutput, ThrusterSetpoint

ALLOCATOR = Path(__file__).resolve().parents[1] / "controller" / "adrc_core" / "thrust_allocator.py"
OTTER = USVModel(max_force=180.0, D=0.395)
LIMITS = {f"{side}_rpm_{bound}": sign * 537.0
          for side in ("left", "right") for bound, sign in (("min", -1.0), ("max", 1.0))}


# 独立写出 0924 推力模型 F = KT·ρ·D⁴·n|n|，避免用被测分配器验证自身。
def force(rpm):
    return 0.5 * 1000.0 * 0.259 ** 4 * math.copysign((rpm / 60.0) ** 2, rpm)


# 交付文件必须原样保留；按 LF 计算，避免 Windows 检出换行改变哈希。
def test_delivered_allocator_is_unmodified():
    digest = hashlib.sha256(ALLOCATOR.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    assert digest == "ceee65f5719194a686365ff0b29ccb04350c3e9b1e51feb1a445d36153bd8a29"


def test_zero_command_is_zero_rpm_with_forward_angles():
    assert ThrusterOutput(OTTER, LIMITS).compute(ControlCommand()) == ThrusterSetpoint(0, 0.0, 0.0, 0.0, 0.0)


# 分配器只做单位换算：还原力/力矩再分配后，每桨推力仍等于命令 × max_force。
def test_each_propeller_force_equals_command_times_max_force():
    output = ThrusterOutput(OTTER, LIMITS)
    generator = random.Random(924)
    for _ in range(500):
        command = allocate_force(generator.uniform(-500, 500), generator.uniform(-80, 80), OTTER, command_limit=1.)
        result = output.compute(command)
        assert result.mode == 0 and result.left_angle == 0.0 and result.right_angle == 0.0
        assert force(result.left_rpm) == pytest.approx(command.left * 180.0, abs=1e-9)
        assert force(result.right_rpm) == pytest.approx(command.right * 180.0, abs=1e-9)


# 右桨命令大于左桨（左转）时右桨转速更高；倒车用负转速，桨角不翻转。
def test_turn_and_reverse_signs():
    output = ThrusterOutput(OTTER, LIMITS)
    turn = output.compute(ControlCommand(left=.1, right=.3))
    assert turn.right_rpm > turn.left_rpm > 0
    reverse = output.compute(ControlCommand(left=-.2, right=-.2))
    assert reverse.left_rpm < 0 and reverse.right_rpm < 0
    assert reverse.left_angle == reverse.right_angle == 0.0


# 满命令对应 ±180 N、±536.66 rpm，在 ±537 上限内，独立截断不会改变转艏力矩。
def test_full_command_reaches_max_force_without_rpm_clipping():
    result = ThrusterOutput(OTTER, LIMITS).compute(ControlCommand(left=1., right=-1.))
    assert result.left_rpm == pytest.approx(536.66, abs=.01)
    assert result.right_rpm == pytest.approx(-536.66, abs=.01)
    assert force(result.left_rpm) == pytest.approx(180.)


def test_default_configuration_covers_default_model():
    ThrusterOutput(USVModel())


@pytest.mark.parametrize("config, message", [
    ({**LIMITS, "left_rpm_max": 536.}, "left rpm limits reach only"),
    ({**LIMITS, "right_rpm_min": -500.}, "right rpm limits reach only"),
    ({**LIMITS, "left_rpm_min": 10.}, "forward and reverse"),
    ({"propeller_spacing": .79}, "Unknown thrust allocator config"),
    ({"thrust_coefficient": 0.}, "thrust_coefficient must be positive"),
])
def test_startup_rejects_inconsistent_configuration(config, message):
    with pytest.raises(ValueError, match=message):
        ThrusterOutput(OTTER, config)


# 无效命令和侧推都不换算：全回转横移尚未启用，节点收到异常时改发零转速。
@pytest.mark.parametrize("command", [
    ControlCommand(left=math.nan), ControlCommand(right=1.5), ControlCommand(lateral=.1)])
def test_invalid_or_lateral_commands_are_rejected(command):
    with pytest.raises(ValueError):
        ThrusterOutput(OTTER, LIMITS).compute(command)
