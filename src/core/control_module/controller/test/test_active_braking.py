"""Signed active braking never becomes a position hold or a reverse cruise."""
import math
from dataclasses import replace

import pytest

from common.dynamics import step_dynamics
from common.models import AlgorithmStatus, ControlCommand, USVModel, VesselState


@pytest.mark.parametrize('initial_speed', [1.5, -.8])
def test_braking_opposes_motion_and_stops_without_driving_back(initial_speed):
    from controller.active_braking_controller import ActiveBrakingController
    model = USVModel()
    controller = ActiveBrakingController(model)
    state = VesselState(u=initial_speed)
    first = controller.compute(state, .1)
    assert (first.left + first.right) * initial_speed < 0
    assert first.left == pytest.approx(first.right)
    for _ in range(300):
        command = controller.compute(state, .1)
        assert command.is_valid() and command.lateral == 0
        assert max(abs(command.left), abs(command.right)) <= .4
        assert state.u * initial_speed >= -1e-6
        if command.status == AlgorithmStatus.GOAL_REACHED:
            break
        state = step_dynamics(state, command, model, .1)
    assert command.status == AlgorithmStatus.GOAL_REACHED
    assert math.hypot(state.u, state.v) <= .05 and abs(state.r) <= .02
    assert command.left == command.right == 0
    # A displaced but stationary boat is not driven back to its stop position.
    command = controller.compute(replace(state, x=100., stamp=state.stamp + .1), .1)
    assert command.left == command.right == 0


def test_braking_requires_continuous_fresh_low_speed_and_low_yaw_observations():
    from controller.active_braking_controller import ActiveBrakingController
    controller = ActiveBrakingController(USVModel())
    quiet = VesselState(stamp=1.)
    for _ in range(30):
        assert controller.compute(quiet, .1).status == AlgorithmStatus.SUCCESS
    for tick in range(1, 8):
        assert controller.compute(replace(quiet, stamp=1 + .1 * tick), .1).status == AlgorithmStatus.SUCCESS
    assert controller.compute(VesselState(stamp=1.8, r=.03), .1).status == AlgorithmStatus.SUCCESS
    for tick in range(19, 29):
        assert controller.compute(VesselState(stamp=tick * .1), .1).status == AlgorithmStatus.SUCCESS
    assert controller.compute(VesselState(stamp=2.9), .1).status == AlgorithmStatus.GOAL_REACHED
    controller.reset()
    assert controller.compute(VesselState(stamp=3.), .1).status == AlgorithmStatus.SUCCESS


@pytest.mark.parametrize('state,dt', [(VesselState(u=math.nan), .1), (VesselState(), 0.)])
def test_braking_invalid_input_resets_confirmation_and_returns_zero(state, dt):
    from controller.active_braking_controller import ActiveBrakingController
    command = ActiveBrakingController(USVModel()).compute(state, dt)
    assert command.status == AlgorithmStatus.INVALID_INPUT
    assert command.left == command.right == command.lateral == 0


@pytest.mark.parametrize('yaw_rate', [-.2, .2])
def test_braking_damps_signed_yaw_and_does_not_claim_lateral_motion_stopped(yaw_rate):
    from controller.active_braking_controller import ActiveBrakingController
    controller = ActiveBrakingController(USVModel())
    command = controller.compute(VesselState(r=yaw_rate), .1)
    assert (command.right - command.left) * yaw_rate < 0
    for tick in range(30):
        command = controller.compute(VesselState(v=.1, stamp=tick * .1), .1)
        assert command.status == AlgorithmStatus.SUCCESS
        assert command.lateral == 0  # No nonexistent lateral actuator is invented.


def test_modelled_active_stop_uses_less_distance_than_zero_thrust_coasting():
    from controller.active_braking_controller import ActiveBrakingController
    model = USVModel()
    controller = ActiveBrakingController(model)
    active = coast = VesselState(u=1.5)
    active_time = coast_time = None
    for tick in range(300):
        if active_time is None:
            active = step_dynamics(active, controller.compute(active, .1), model, .1)
            if active.u <= .05:
                active_time = (tick + 1) * .1
        if coast_time is None:
            coast = step_dynamics(coast, ControlCommand(), model, .1)
            if coast.u <= .05:
                coast_time = (tick + 1) * .1
        if active_time is not None and coast_time is not None:
            break
    assert active_time is not None and coast_time is not None
    assert active_time < coast_time
    assert active.x < coast.x


def test_braking_confirmation_restarts_after_large_observation_gap():
    from controller.active_braking_controller import ActiveBrakingController
    controller = ActiveBrakingController(USVModel())
    assert controller.compute(VesselState(stamp=1.), .1).status == AlgorithmStatus.SUCCESS
    assert controller.compute(VesselState(stamp=3.), .1).status == AlgorithmStatus.SUCCESS
    for tick in range(1, 10):
        assert controller.compute(VesselState(stamp=3. + tick * .1), .1).status == AlgorithmStatus.SUCCESS
    assert controller.compute(VesselState(stamp=4.), .1).status == AlgorithmStatus.GOAL_REACHED
