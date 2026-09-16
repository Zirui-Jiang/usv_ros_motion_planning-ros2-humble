"""0914 接入契约：输入单位、实际样本时间、生命周期与推力分配。"""
import math

import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from controller import ADRCController, create_controller
from trajectory_planner import Trajectory


def reference(speed=.5, yaw=.1, goal=None):
    return Trajectory([[0, 0, 0, 0, yaw, 0, speed, 0],
                       [10, 5, 5, 0, yaw, 0, speed, 0]],
                      frame_id="otter/map", goal=goal)


def test_factory_maps_original_first_step_to_physical_thrusters():
    controller = create_controller("adrc", USVModel())
    assert isinstance(controller, ADRCController)
    result = controller.compute(VesselState(u=.2, v=3.), reference(), .1)
    # 原 PI: 8*.3 + .8*.3*.1；原艏向环第一拍没有误差差分。
    force = 2.424
    moment = .1503
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.left == pytest.approx((force - moment / .395) / 100)
    assert result.right == pytest.approx((force + moment / .395) / 100)
    assert result.lateral == 0


@pytest.mark.parametrize("yaw", [-.1, .1])
def test_yaw_sign_and_signed_longitudinal_velocity(yaw):
    controller = ADRCController(USVModel())
    result = controller.compute(VesselState(u=-.2), reference(yaw=yaw), .1)
    assert (result.left + result.right) * 50 == pytest.approx(5.656)
    assert (result.right - result.left) * yaw > 0


def test_core_step_uses_sample_timestamp_not_control_callback_dt():
    controller = ADRCController(USVModel())
    controller.compute(VesselState(u=.2, stamp=1.), reference(), .9)
    result = controller.compute(VesselState(u=.2, stamp=1.2), reference(), .01)
    # 首拍 .1 s，再隔 .2 s 的实测状态；回调的 .01 s 不参与积分。
    assert (result.left + result.right) * 50 == pytest.approx(2.472)
    assert (result.right - result.left) * 50 * .395 == pytest.approx(.1509)


def test_reset_restores_original_first_step_and_clears_filter():
    controller = ADRCController(USVModel())
    first = controller.compute(VesselState(), reference(), .1)
    controller.compute(VesselState(psi=.05, stamp=.1), reference(), .1)
    controller.reset()
    repeated = controller.compute(VesselState(), reference(), .1)
    assert repeated == first


@pytest.mark.parametrize("state, trajectory, dt, status", [
    (VesselState(), Trajectory(), .1, AlgorithmStatus.NO_PATH),
    (VesselState(u=float("nan")), reference(), .1, AlgorithmStatus.INVALID_INPUT),
    (VesselState(r=float("inf")), reference(), .1, AlgorithmStatus.INVALID_INPUT),
    (VesselState(frame_id="other/map"), reference(), .1, AlgorithmStatus.INVALID_INPUT),
    (VesselState(), reference(speed=-1), .1, AlgorithmStatus.INVALID_INPUT),
    (VesselState(), reference(), 0, AlgorithmStatus.INVALID_INPUT),
    (VesselState(), reference(), float("nan"), AlgorithmStatus.INVALID_INPUT),
])
def test_invalid_input_stops_and_clears_history(state, trajectory, dt, status):
    controller = ADRCController(USVModel())
    controller.compute(VesselState(), reference(), .1)
    result = controller.compute(state, trajectory, dt)
    assert result.status == status
    assert (result.left, result.right, result.lateral) == (0, 0, 0)
    assert controller.speed_loop.integral_speed == 0
    assert controller.heading_loop.previous_error is None


def test_goal_settling_completion_and_resuming_outside_disk():
    controller = ADRCController(USVModel())
    path = reference(goal=(5., 0.))
    moving = controller.compute(VesselState(x=4.9, u=.4), path, .1)
    assert moving.status == AlgorithmStatus.SETTLING
    stopped = controller.compute(VesselState(x=4.9, u=.1), path, .1)
    assert stopped.status == AlgorithmStatus.GOAL_REACHED
    assert controller.compute(VesselState(x=2.), path, .1).status == AlgorithmStatus.SUCCESS


def test_actual_yaw_rate_is_not_an_input_to_the_original_scalar_core():
    a = ADRCController(USVModel()).compute(VesselState(r=0), reference(), .1)
    b = ADRCController(USVModel()).compute(VesselState(r=2), reference(), .1)
    assert a == b


def test_allocation_honors_command_limit_and_preserves_turn_direction():
    controller = ADRCController(USVModel(), {"command_limit": .05})
    result = controller.compute(VesselState(), reference(speed=10, yaw=1), .1)
    assert result.is_valid()
    assert max(abs(result.left), abs(result.right)) == pytest.approx(.05)
    assert result.right > result.left


@pytest.mark.parametrize("config", [
    {"sample_time": 0}, {"command_limit": 1.1}, {"command_limit": 0},
    {"speed_integral_min": 9}, {"heading_integral_max": -.5},
    {"filter_window_length": 2.5}, {"filter_history_length": 3},
    {"k_p_heading": -1}, {"lookahead_time": -1}, {"force_min": 1},
    {"goal_distance_tolerance": 0}, {"error_reset_deg": 181},
    {"unknown": 1}, {"sample_time": math.inf},
])
def test_adapter_rejects_invalid_startup_configuration(config):
    with pytest.raises(ValueError):
        ADRCController(USVModel(), config)


@pytest.mark.parametrize("model", [USVModel(D=0), USVModel(max_force=0)])
def test_adapter_rejects_invalid_actuator_model(model):
    with pytest.raises(ValueError):
        ADRCController(model)


def test_duplicate_state_holds_command_without_integrating_or_filtering():
    controller = ADRCController(USVModel())
    state = VesselState(stamp=10.)
    first = controller.compute(state, reference(), .1)
    history = list(controller.heading_loop.heading_filter.position_history)
    integral = controller.speed_loop.integral_speed
    for _ in range(5):
        assert controller.compute(state, reference(yaw=.2), .1) == first
    assert controller.heading_loop.heading_filter.position_history == history
    assert controller.speed_loop.integral_speed == integral


def test_backwards_sample_stops_and_next_sample_starts_with_nominal_step():
    controller = ADRCController(USVModel())
    controller.compute(VesselState(stamp=10.), reference(), .1)
    result = controller.compute(VesselState(stamp=9.9), reference(), .1)
    assert result.status == AlgorithmStatus.INVALID_INPUT
    assert result.left == result.right == 0
    assert controller.speed_loop.integral_speed == 0
    assert controller.compute(VesselState(stamp=10.1), reference(), .1).status == AlgorithmStatus.SUCCESS


def test_long_gap_and_explicit_hold_reset_old_integral_and_filter():
    controller = ADRCController(USVModel())
    first = controller.compute(VesselState(stamp=10.), reference(), .1)
    controller.compute(VesselState(psi=.05, stamp=10.1), reference(), .1)
    assert controller.compute(VesselState(stamp=12.), reference(), .1) == first
    controller.on_control_hold()
    assert controller.speed_loop.integral_speed == 0
    assert controller.heading_loop.heading_filter.position_history == []
    assert controller.compute(VesselState(stamp=12.1), reference(), .1) == first


def test_new_rolling_trajectory_preserves_history_for_next_new_sample():
    controller = ADRCController(USVModel())
    controller.compute(VesselState(stamp=10.), reference(), .1)
    controller.compute(VesselState(stamp=10.1), reference(yaw=.12), .1)
    assert len(controller.heading_loop.heading_filter.position_history) == 2
    assert controller.speed_loop.integral_speed > .04


@pytest.mark.parametrize("measured_speed", [-.5, .5])
@pytest.mark.parametrize("yaw", [-.1, .1])
def test_allocation_saturation_freezes_both_integrals_in_windup_direction(measured_speed, yaw):
    controller = ADRCController(USVModel(), {"command_limit": .001})
    command = controller.compute(VesselState(u=measured_speed), reference(speed=0., yaw=yaw), .1)
    assert command.is_valid()
    assert controller.speed_loop.integral_speed == 0.
    assert controller.heading_loop.integral_heading == 0.
    assert max(abs(command.left), abs(command.right)) == pytest.approx(.001)


def test_allocation_freeze_never_undoes_large_heading_error_reset():
    controller = ADRCController(USVModel(), {"command_limit": .001})
    controller.heading_loop.integral_heading = .2
    controller.compute(VesselState(), reference(speed=0., yaw=1.), .1)
    assert controller.heading_loop.integral_heading == 0.


def test_core_failure_clears_partial_updates(monkeypatch):
    controller = ADRCController(USVModel())
    def fail(*args, **kwargs):
        raise ValueError("test arithmetic failure")
    monkeypatch.setattr(controller.heading_loop, "compute", fail)
    command = controller.compute(VesselState(), reference(), .1)
    assert command.status == AlgorithmStatus.INVALID_INPUT
    assert command.left == command.right == 0.
    assert controller.speed_loop.integral_speed == 0.
