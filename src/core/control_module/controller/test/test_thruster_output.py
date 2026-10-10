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


# 带独立侧推器的船模：侧推分量不进入桨角/转速指令，左右桨照常按 mode 0 换算，桨角仍为零。
def test_separate_lateral_thruster_is_not_part_of_the_vector_setpoint():
    wamv = USVModel(name="wamv", type="Omni", max_force=250.0, max_lateral_force=250.0, D=1.027)
    limits = {key: math.copysign(633.0, value) for key, value in LIMITS.items()}
    output = ThrusterOutput(wamv, limits)
    plain = output.compute(ControlCommand(left=.2, right=.4))
    with_lateral = output.compute(ControlCommand(left=.2, right=.4, lateral=-.6))
    assert with_lateral == plain
    assert with_lateral.mode == 0 and with_lateral.left_angle == with_lateral.right_angle == 0.0
    assert force(with_lateral.right_rpm) == pytest.approx(.4 * 250.0, abs=1e-9)
    # 超出 [-1, 1] 的侧推仍是无效命令。
    with pytest.raises(ValueError):
        output.compute(ControlCommand(lateral=1.5))


# 两台艉部全回转桨的船模：回转轴在转动中心后 2.374 m、距中心线 1.027 m，侧推命令 1 对应 200 N 横向力。
AZIMUTH = USVModel(name="wamv", type="Azimuth", max_force=250.0, max_lateral_force=200.0, D=1.027, thruster_x=-2.374)
AZIMUTH_LIMITS = {key: math.copysign(633.0, value) for key, value in LIMITS.items()}


# 由桨角和转速独立还原每台桨的推力矢量 (纵向, 横向)，横向以左舷为正。
def thrust_vectors(setpoint):
    return tuple((force(rpm) * math.cos(angle), force(rpm) * math.sin(angle))
                 for angle, rpm in ((setpoint.left_angle, setpoint.left_rpm), (setpoint.right_angle, setpoint.right_rpm)))


# 两台桨对船体的合力：纵向力、横向力和转艏力矩（逆时针为正）；左桨在中心线左侧 D 处，回转轴纵向位置为 thruster_x。
def wrench(setpoint, model=AZIMUTH):
    left, right = thrust_vectors(setpoint)
    lateral = left[1] + right[1]
    return left[0] + right[0], lateral, model.thruster_x * lateral + model.D * (right[0] - left[0])


# 侧推命令为零时全回转桨与固定桨完全相同：mode 0、桨角为零、转速等于命令 × max_force。
def test_azimuth_without_lateral_command_equals_fixed_propellers():
    output = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS)
    result = output.compute(ControlCommand(left=.3, right=-.2))
    assert result.mode == 0 and result.left_angle == result.right_angle == 0.0
    assert force(result.left_rpm) == pytest.approx(.3 * 250.0, abs=1e-9)
    assert force(result.right_rpm) == pytest.approx(-.2 * 250.0, abs=1e-9)
    assert output.normalized(result) == pytest.approx((.3, -.2))


# 侧推命令非零时标为 mode 1：合成的纵向力、横向力和转艏力矩等于请求值，艉部力臂带出的力矩已被补偿。
# 取纵向力不小于横向力的命令，此时桨角不会触及 45° 上限。
def test_azimuth_allocation_reproduces_requested_wrench():
    generator = random.Random(924)
    for _ in range(500):
        output = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS)
        command = ControlCommand(left=generator.uniform(.15, .4), right=generator.uniform(.15, .4),
                                 lateral=generator.choice((-1, 1)) * generator.uniform(.01, .35))
        result = output.compute(command)
        requested = ((command.left + command.right) * 250.0, command.lateral * 200.0,
                     (command.right - command.left) * 250.0 * 1.027)
        assert result.mode == 1
        assert wrench(result) == pytest.approx(requested, abs=1e-6)
        assert max(abs(value) for value in output.normalized(result)) <= 1.0 + 1e-9
        assert max(abs(result.left_angle), abs(result.right_angle)) <= math.pi / 4 + 1e-9


# 倒推和转艏方向任意时同样成立：纵向力与转艏力矩总是等于请求值，横向力不超过请求值且方向不反。
def test_azimuth_allocation_always_keeps_surge_and_yaw():
    generator = random.Random(1010)
    reduced = 0
    for _ in range(1000):
        command = ControlCommand(left=generator.uniform(-.4, .4), right=generator.uniform(-.4, .4),
                                 lateral=generator.choice((-1, 1)) * generator.uniform(.01, .35))
        result = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS).compute(command)
        surge, lateral, yaw = wrench(result)
        assert surge == pytest.approx((command.left + command.right) * 250.0, abs=1e-6)
        assert yaw == pytest.approx((command.right - command.left) * 250.0 * 1.027, abs=1e-6)
        requested = command.lateral * 200.0
        assert lateral * requested >= 0 and abs(lateral) <= abs(requested) + 1e-6
        assert max(abs(result.left_angle), abs(result.right_angle)) <= math.pi / 4 + 1e-9
        reduced += abs(lateral) < abs(requested) - 1e-6
    # 两种情况都被这组随机命令覆盖到。
    assert 0 < reduced < 1000


# 桨角触及上限时只减小横向力。不前进，且请求的转艏力矩恰好等于横向力在船艉自然带出的力矩时，
# 两桨没有纵向推力可用，全额横向力需要 90° 桨角；按 45° 上限只能给出约七成。
def test_deflection_limit_reduces_lateral_force_only():
    lateral_command = .25
    moment = -2.374 * lateral_command * 200.0
    differential = moment / (250.0 * 1.027)
    result = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS).compute(
        ControlCommand(left=-.5 * differential, right=.5 * differential, lateral=lateral_command))
    surge, lateral, yaw = wrench(result)
    assert (surge, yaw) == pytest.approx((0.0, moment), abs=1e-6)
    # 可行比例 s 满足 s = (|thruster_x| / D)(1 − s)。
    ratio = 2.374 / 1.027
    assert lateral == pytest.approx(50.0 * ratio / (1 + ratio), abs=1e-4)
    assert abs(result.left_angle) == pytest.approx(math.pi / 4, abs=1e-6)
    assert abs(result.right_angle) == pytest.approx(math.pi / 4, abs=1e-6)
    # 上限可配置；超出 (0, π/2) 的取值在启动时被拒绝。
    wide = ThrusterOutput(AZIMUTH, {**AZIMUTH_LIMITS, "max_deflection": 1.2}).compute(
        ControlCommand(left=-.5 * differential, right=.5 * differential, lateral=lateral_command))
    assert abs(wrench(wide)[1]) > lateral
    for invalid in (0.0, math.pi / 2, -1.0, math.nan):
        with pytest.raises(ValueError, match="max_deflection"):
            ThrusterOutput(AZIMUTH, {**AZIMUTH_LIMITS, "max_deflection": invalid})


# 只要横向力、不前进不转艏：横向力在船艉会使船艏右转，右桨前推、左桨倒推给出左转力矩与之抵消，两桨各出一半横向力。
def test_pure_sway_uses_opposed_longitudinal_thrust_and_no_net_yaw():
    result = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS).compute(ControlCommand(lateral=.5))
    assert wrench(result) == pytest.approx((0.0, 100.0, 0.0), abs=1e-6)
    left, right = thrust_vectors(result)
    assert right[0] > 0 > left[0] and right[0] == pytest.approx(-left[0])
    assert left[1] == pytest.approx(50.0, abs=1e-6) and right[1] == pytest.approx(50.0, abs=1e-6)
    mirrored = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS).compute(ControlCommand(lateral=-.5))
    assert wrench(mirrored) == pytest.approx((0.0, -100.0, 0.0), abs=1e-6)


# 倒推的那台桨用负转速、桨角留在船艏一侧，而不是把桨转过半圈。
def test_reverse_thrust_keeps_angle_near_bow_with_negative_rpm():
    result = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS).compute(ControlCommand(lateral=.5))
    assert result.left_rpm < 0 < result.right_rpm
    assert abs(result.left_angle) < math.pi / 2 and abs(result.right_angle) < math.pi / 2


# 横向力逐步加大，使左桨的纵向推力由正变负：桨角始终在上限以内，左桨只在自身推力接近零时换到另一侧，
# 不会带着推力扫过大角度；右桨一直正推，桨角平滑变化。
def test_thruster_changes_deflection_side_only_at_zero_thrust():
    output = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS)
    previous, switched = None, 0
    for step in range(1, 201):
        result = output.compute(ControlCommand(left=.12, right=.12, lateral=.002 * step))
        assert wrench(result) == pytest.approx((60.0, .4 * step, 0.0), abs=1e-6)
        assert max(abs(result.left_angle), abs(result.right_angle)) <= math.pi / 4 + 1e-9
        if previous is not None:
            assert abs(result.right_angle - previous.right_angle) < .05
            if abs(result.left_angle - previous.left_angle) > .05:
                switched += 1
                assert max(abs(output.normalized(result)[0]), abs(output.normalized(previous)[0])) < .01
        previous = result
    assert switched == 1
    assert previous.left_rpm < 0 < previous.right_rpm
    # 回到零侧推即回到固定桨表示：mode 0、桨角归零。
    fixed = output.compute(ControlCommand(left=.12, right=.12))
    assert fixed.mode == 0 and fixed.left_angle == fixed.right_angle == 0.0
    assert force(fixed.left_rpm) == pytest.approx(30.0, abs=1e-9)


# 任一台桨的合力超过 max_force 时三项需求同比缩小：合力方向不变，单桨不超限。
def test_saturation_scales_the_whole_wrench_uniformly():
    output = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS)
    result = output.compute(ControlCommand(left=.9, right=.9, lateral=1.0))
    surge, lateral, yaw = wrench(result)
    scale = surge / 450.0
    assert 0 < scale < 1
    assert (lateral, yaw) == pytest.approx((200.0 * scale, 0.0), abs=1e-6)
    assert max(abs(value) for value in output.normalized(result)) == pytest.approx(1.0)


# 零命令回到 mode 0 并全部归零；归一化推力与零指令一致。
def test_zero_command_after_lateral_thrust_resets_to_fixed_zero():
    output = ThrusterOutput(AZIMUTH, AZIMUTH_LIMITS)
    output.compute(ControlCommand(lateral=.3))
    assert output.compute(ControlCommand()) == ThrusterSetpoint(0, 0.0, 0.0, 0.0, 0.0)
    assert output.normalized(ThrusterSetpoint()) == (0.0, 0.0)
