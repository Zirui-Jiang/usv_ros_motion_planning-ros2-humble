"""Signed 3-DOF prediction and whole hull swept safety tests."""
import math

import numpy as np
import pytest

from common.models import AlgorithmStatus, ControlCommand, GridSnapshot, USVModel, VesselState


def dynamics():
    import importlib.util
    assert importlib.util.find_spec("common.dynamics") is not None
    from common import dynamics as module
    return module


def grid_at_obstacle(x=None, y=None):
    data = np.zeros((400, 400), dtype=np.uint8)
    if x is not None:
        data[math.floor((y + 10) / .05), math.floor((x + 10) / .05)] = 254
    return GridSnapshot(data, .05, -10, -10)


def test_dynamics_rotates_body_velocity_and_preserves_reverse_motion():
    module = dynamics()
    state = module.step_dynamics(VesselState(psi=math.pi / 2, u=-.3), ControlCommand(), USVModel(), .1)
    assert state.y < 0 and abs(state.x) < 1e-8
    assert -.3 < state.u < 0
    assert state.stamp == pytest.approx(.1)
    spin = module.step_dynamics(VesselState(), ControlCommand(-.2, .2), USVModel(), .1)
    assert spin.r > 0 and spin.psi > 0


def test_braking_opposes_signed_motion_and_prediction_reaches_rest():
    module = dynamics()
    for velocity in [-.3, .3]:
        state = VesselState(u=velocity, r=.2)
        brake = module.braking_command(state, USVModel())
        assert (brake.left + brake.right) * velocity < 0
        assert brake.right - brake.left < 0
        path, stopped = module.predict_control_path(state, brake, USVModel(), .1)
        assert stopped and len(path) > 2
        assert path.shape[1] == 3 and np.isfinite(path).all()
        assert module.clear_control(state, brake, grid_at_obstacle(), USVModel(), .1)


def test_reverse_stern_collision_and_current_overlap_fail_closed():
    module = dynamics()
    state = VesselState(u=-.3)
    assert not module.clear_control(state, ControlCommand(-.4, -.4), grid_at_obstacle(-1.4, 0), USVModel(), .1)
    assert not module.clear_control(VesselState(), ControlCommand(), grid_at_obstacle(0, 0), USVModel(), .1)


def test_rotation_sweep_checks_corner_between_clear_endpoints():
    module = dynamics()
    # A yawing rectangle sweeps a corner beyond its initial y extent.
    state = VesselState(r=.5)
    empty = grid_at_obstacle()
    path, stopped = module.predict_control_path(state, ControlCommand(-.4, .4), USVModel(), .5)
    assert stopped and abs(path[-1, 2]) > .2
    obstacle = grid_at_obstacle(.9, .85)
    assert module.clear_control(VesselState(), ControlCommand(), obstacle, USVModel(), .1)
    assert not module.clear_control(state, ControlCommand(-.4, .4), obstacle, USVModel(), .5)
    assert module.clear_control(state, ControlCommand(-.4, .4), empty, USVModel(), .5)


def test_invalid_candidate_unmodeled_side_thrust_and_nonstop_fail_closed():
    module = dynamics()
    model = USVModel()
    for command in [ControlCommand(left=math.nan), ControlCommand(lateral=.1),
                    ControlCommand(status=AlgorithmStatus.INVALID_INPUT)]:
        assert not module.clear_control(VesselState(), command, grid_at_obstacle(), model, .1)
    assert not module.clear_control(VesselState(u=math.nan), ControlCommand(), grid_at_obstacle(), model, .1)
    _, stopped = module.predict_control_path(VesselState(v=10), ControlCommand(), model, .1)
    assert not stopped


def test_prediction_matches_analytic_free_decay_not_instantaneous_stop():
    module = dynamics()
    model = USVModel()
    state = module.step_dynamics(VesselState(u=.3), ControlCommand(), model, 1.)
    decay_rate = 20. / (model.mass + model.x_dot_u)
    assert state.u == pytest.approx(.3 * math.exp(-decay_rate), abs=1e-9)
    assert state.x == pytest.approx(.3 / decay_rate * (1 - math.exp(-decay_rate)), abs=1e-9)
    assert state.x > .2


def test_unknown_cell_and_map_frame_mismatch_are_unsafe():
    module = dynamics()
    data = np.zeros((100, 100), dtype=np.uint8)
    data[50, 50] = 255
    unknown = GridSnapshot(data, .1, -5, -5)
    assert not module.clear_control(VesselState(), ControlCommand(), unknown, USVModel(), .1)
    other_frame = GridSnapshot(np.zeros((100, 100), dtype=np.uint8), .1, -5, -5, frame_id="other")
    assert not module.clear_control(VesselState(), ControlCommand(), other_frame, USVModel(), .1)


def test_recorded_first_control_snapshot_accepts_bounded_stronger_braking():
    module = dynamics()
    # Exact 57.3 s odom and nearby lethal cells from safety-smoke-003,
    # costmap_input_883edcae (57.2 s). The map is translated/cropped only;
    # the 0.5 m cell alignment, ship state, hull size and margin are unchanged.
    data = np.zeros((60, 60), dtype=np.uint8)
    for x, y in [(23.75, -18.25), (24.25, -18.25), (23.75, -17.75),
                 (24.25, -17.75), (23.75, -17.25)]:
        data[math.floor((y + 35.) / .5), math.floor((x - 10.) / .5)] = 254
    grid = GridSnapshot(data, .5, 10., -35., stamp=57.2)
    state = VesselState(25.994430458604526, -17.279572175183908, -2.8235968187774616,
                        .4812277111283225, .03603041138338536, -.09637738961840121, stamp=57.3)
    commands = module.braking_candidates(state, USVModel())
    assert not module.clear_control(state, commands[0], grid, USVModel(), .1)
    assert module.clear_control(state, commands[1], grid, USVModel(), .1)
    assert module.clear_control(state, commands[2], grid, USVModel(), .1)
    assert commands[1].left == pytest.approx(-.31313596582089254)
    assert commands[1].right == pytest.approx(-.26433728753309443)
    for command in commands:
        assert max(abs(command.left), abs(command.right)) <= .4
        assert command.lateral == 0.


@pytest.mark.parametrize("velocity", [-.5, .5])
def test_emergency_braking_candidates_oppose_signed_motion_with_no_side_thrust(velocity):
    module = dynamics()
    state = VesselState(u=velocity, v=.2, r=-.1)
    commands = module.braking_candidates(state, USVModel())
    assert commands[0] == module.braking_command(state, USVModel())
    magnitudes = []
    for command in commands:
        assert command.is_valid() and command.lateral == 0.
        assert (command.left + command.right) * velocity < 0
        assert (command.right - command.left) * state.r < 0
        assert max(abs(command.left), abs(command.right)) <= .4
        magnitudes.append(abs(command.left + command.right))
    assert magnitudes == sorted(magnitudes)


def test_all_braking_candidates_remain_rejected_for_an_occupied_current_hull():
    module = dynamics()
    state = VesselState(u=.5)
    commands = module.braking_candidates(state, USVModel())
    assert not any(module.clear_control(state, command, grid_at_obstacle(0, 0), USVModel(), .1)
                   for command in commands)
    invalid = module.braking_candidates(VesselState(u=math.nan), USVModel())
    assert all(command.status == AlgorithmStatus.INVALID_INPUT for command in invalid)
