# 验证定点定艏控制的欠驱动约束、阶段滞回、推力接管连续性和确定性闭环收敛。
"""Station keeping contracts and deterministic planar closed-loop acceptance."""
import math
from dataclasses import replace

import numpy as np
import pytest

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus, USVModel, VesselState


def components():
    # Keep discovery failure an explicit failed assertion during the red stage.
    import importlib.util
    assert importlib.util.find_spec("controller.station_keeping_controller") is not None
    assert importlib.util.find_spec("common.dynamics") is not None
    from controller.station_keeping_controller import StationKeepingController
    from common.dynamics import step_dynamics
    return StationKeepingController, step_dynamics


# 正后方目标分别正驶/倒驶，纯横向目标用转向而非侧推。
def test_signed_reverse_and_lateral_station_targets_do_not_request_side_thrust():
    controller_type, _ = components()
    for target, expected_sign in [((2., 0., 0.), 1), ((-2., 0., 0.), -1)]:
        controller = controller_type(USVModel())
        cmd = controller.compute(VesselState(), target, .1)
        assert expected_sign * (cmd.left + cmd.right) > 0
        assert cmd.left == pytest.approx(cmd.right)
        assert cmd.lateral == 0
    controller = controller_type(USVModel())
    cmd = controller.compute(VesselState(), (0., 2., 0.), .1)
    assert cmd.right > cmd.left
    assert abs(cmd.left + cmd.right) < 1e-8


# 验证命令幅值、每拍变化率限制及非法输入立即归零。
def test_limits_slew_and_bad_inputs_stop_immediately():
    controller_type, _ = components()
    controller = controller_type(USVModel())
    last = (0., 0.)
    for target in [(30., 0., math.pi)] * 20 + [(-30., -20., 0.)] * 20:
        cmd = controller.compute(VesselState(), target, .1)
        assert cmd.is_valid() and cmd.lateral == 0
        for value, old in zip((cmd.left, cmd.right), last):
            assert abs(value) <= .4
            assert abs(value - old) <= .060000001
        last = (cmd.left, cmd.right)
    for state, goal, dt in [(VesselState(u=math.nan), (0, 0, 0), .1),
                             (VesselState(), (0, math.inf, 0), .1),
                             (VesselState(), (0, 0, 0), 0)]:
        cmd = controller.compute(state, goal, dt)
        assert cmd.status == AlgorithmStatus.INVALID_INPUT
        assert cmd.left == cmd.right == cmd.lateral == 0


# HOLD 持续有效，偏移超出按精度缩放的滞回门限后恢复。
def test_hold_remains_active_and_hysteresis_scales_with_requested_accuracy():
    controller_type, _ = components()
    controller = controller_type(USVModel(), {"position_tolerance": .2})
    for _ in range(20):
        cmd = controller.compute(VesselState(), (0, 0, 0), .1)
    assert controller.phase == "HOLD"
    assert cmd.status == AlgorithmStatus.SUCCESS
    controller.compute(VesselState(y=.11), (0, 0, 0), .1)
    assert controller.phase == "RECOVER"


# 安全替代后变化率从实际发布命令开始计算。
def test_slew_uses_last_published_override_instead_of_rejected_candidate():
    from common.models import ControlCommand
    controller_type, _ = components()
    controller = controller_type(USVModel())
    for _ in range(20):
        controller.compute(VesselState(), (10, 0, 0), .1)
    controller.observe_command(ControlCommand())
    cmd = controller.compute(VesselState(), (10, 0, 0), .1)
    assert cmd.left == pytest.approx(.06) and cmd.right == pytest.approx(.06)
    controller.observe_command(ControlCommand(-.15, -.15))
    cmd = controller.compute(VesselState(), (10, 0, 0), .1)
    assert cmd.left == pytest.approx(-.09) and cmd.right == pytest.approx(-.09)
    for invalid in [ControlCommand(left=math.nan), ControlCommand(lateral=.1), ControlCommand(left=1.1)]:
        with pytest.raises(ValueError):
            controller.observe_command(invalid)
    controller.reset()
    assert controller.phase == "RECOVER"


# 导航转定点接管时保持已发布推力的连续性。
def test_navigation_to_holding_keeps_actual_command_slew_continuous():
    from common.models import ControlCommand
    controller_type, _ = components()
    controller = controller_type(USVModel())
    controller.observe_command(ControlCommand(.1, .1))
    command = controller.compute(VesselState(u=.5), (2, 0, 0), .1)
    assert command.left == pytest.approx(.04) and command.right == pytest.approx(.04)
    assert abs(command.left - .1) <= .060000001


# 接管值高于定点上限时按变化率渐降，同时始终满足硬件范围。
@pytest.mark.parametrize("sign", [-1, 1])
def test_handover_above_station_limit_ramps_from_valid_hardware_command(sign):
    from common.models import ControlCommand
    controller_type, _ = components()
    controller = controller_type(USVModel())
    previous = sign * .8
    controller.observe_command(ControlCommand(previous, previous))
    for tick in range(15):
        command = controller.compute(VesselState(), (0, 0, 0), .1)
        assert command.is_valid() and command.lateral == 0
        assert abs(command.left - previous) <= .060000001
        assert command.left == pytest.approx(command.right)
        assert abs(command.left) <= abs(previous)
        if tick == 0:
            assert command.left == pytest.approx(sign * .74)
        previous = command.left
    assert abs(command.left) <= controller.config["command_limit"]
    assert command.left == pytest.approx(0.)
    with pytest.raises(ValueError):
        controller.observe_command(ControlCommand(sign * 1.01, 0.))


# 多个初始状态收敛后，在固定六十秒窗口同时检查位置和航向合格率。
@pytest.mark.parametrize("initial", [
    VesselState(x=x, y=y) for x, y in [(2, 2), (2, -2), (-2, 2), (-2, -2)]
] + [VesselState(y=.4), VesselState(u=.3), VesselState(u=-.3),
     VesselState(psi=math.pi / 2), VesselState(psi=math.radians(179)),
     VesselState(x=-2, y=2, psi=-math.pi / 2, u=-.3)])
def test_closed_loop_converges_then_meets_fixed_60_second_joint_window(initial):
    controller_type, step = components()
    model = USVModel()
    controller = controller_type(model)
    state, consecutive, converged_at = initial, 0, None
    for tick in range(1800):
        state = step(state, controller.compute(state, (0, 0, 0), .1), model, .1)
        converged = (math.hypot(state.x, state.y) <= .25
                     and abs(wrap_angle(state.psi)) <= math.radians(3)
                     and math.hypot(state.u, state.v) <= .04)
        consecutive = consecutive + 1 if converged else 0
        if consecutive >= 50:
            converged_at = tick * .1
            break
    assert converged_at is not None, (initial, state, controller.phase)
    samples = []
    for _ in range(600):
        state = step(state, controller.compute(state, (0, 0, 0), .1), model, .1)
        samples.append(math.hypot(state.x, state.y) <= .5
                       and abs(wrap_angle(state.psi)) <= math.radians(5))
    assert np.mean(samples) >= .95


# 施加横向与转动脉冲后不重置控制器也应回到目标。
def test_lateral_impulse_recovers_position_and_fixed_heading_without_reset():
    controller_type, step = components()
    model, state = USVModel(), VesselState()
    controller = controller_type(model)
    for _ in range(100):
        state = step(state, controller.compute(state, (0, 0, 0), .1), model, .1)
    state = replace(state, v=.4, r=.1)
    saw_recover = False
    samples = []
    for tick in range(2400):
        state = step(state, controller.compute(state, (0, 0, 0), .1), model, .1)
        saw_recover |= controller.phase == "RECOVER"
        if tick >= 1800:
            samples.append(math.hypot(state.x, state.y) <= .5
                           and abs(wrap_angle(state.psi)) <= math.radians(5))
    assert saw_recover
    assert np.mean(samples) >= .95


# 拒绝无效精度、非有限角度、过大命令限幅和未知配置。
@pytest.mark.parametrize("config", [{"position_tolerance": 0}, {"heading_tolerance_deg": math.nan},
                                    {"command_limit": 1.1}, {"invented_gain": 1}])
def test_invalid_station_configuration_is_rejected(config):
    controller_type, _ = components()
    with pytest.raises(ValueError):
        controller_type(USVModel(), config)


def test_absent_heading_does_not_lock_current_or_zero_heading():
    controller_type, _ = components()
    controller = controller_type(USVModel())
    for heading in [1.2, -2.1, .7]:
        command = controller.compute(VesselState(psi=heading), (0., 0., None), .1)
        assert command.status == AlgorithmStatus.SUCCESS
        assert command.left == command.right == 0.
        assert command.message == 'HOLD'


@pytest.mark.parametrize('initial', [VesselState(x=2., y=2., psi=1.2),
                                     VesselState(y=-1., psi=-2., u=-.2)])
def test_position_only_closed_loop_recovers_without_fixed_heading(initial):
    controller_type, step = components()
    model = USVModel()
    controller = controller_type(model)
    state = initial
    for _ in range(1800):
        state = step(state, controller.compute(state, (0., 0., None), .1), model, .1)
    assert math.hypot(state.x, state.y) <= .5
    assert math.hypot(state.u, state.v) <= .04
