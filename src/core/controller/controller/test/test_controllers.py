import math

import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from controller import PIDController, LQRController, create_controller
from trajectory_planner import Trajectory, TrajectoryCell, LosTrajectoryPlanner


def constant_reference(speed=0.8, yaw=0.1, goal=None, stamp=0.0):
    return Trajectory([[0, 0, 0, 0, yaw, 0, speed, 0],
                       [10, 8, 8, 0, yaw, 0, speed, 0]], stamp=stamp, frame_id="otter/map", goal=goal)


def test_pid_matches_legacy_allocation_and_integral_update():
    controller = PIDController(USVModel(), {"k_p_speed": 0.4, "k_i_speed": 0.2, "k_p_course": 0.5})
    result = controller.compute(VesselState(u=0.2), constant_reference(), 0.1)
    expected_surge = 0.4 * 0.6 + 0.2 * 0.6 * 0.1
    expected_yaw = 0.5 * 0.1
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.left == pytest.approx((expected_surge - expected_yaw / 0.395) / 2)
    assert result.right == pytest.approx((expected_surge + expected_yaw / 0.395) / 2)
    assert controller.integral_speed == pytest.approx(0.012)


def test_pid_gain_update_preserves_history_and_failed_update_is_atomic():
    controller = PIDController(USVModel(), {"k_i_speed": 0.2})
    controller.compute(VesselState(), constant_reference(), 0.1)
    old_integral = controller.integral_speed
    controller.update_config({"k_p_speed": 0.3})
    assert controller.integral_speed == old_integral
    with pytest.raises(ValueError):
        controller.update_config({"k_p_speed": float("nan")})
    assert controller.config["k_p_speed"] == 0.3
    controller.reset()
    assert controller.integral_speed == 0


@pytest.mark.parametrize("kind", [PIDController, LQRController])
def test_invalid_input_returns_zero_command(kind):
    controller = kind(USVModel())
    for state, trajectory, dt in [
        (VesselState(), Trajectory(), 0.1),
        (VesselState(u=float("nan")), constant_reference(), 0.1),
        (VesselState(), constant_reference(), 0.0),
        (VesselState(frame_id="different/map"), constant_reference(), 0.1),
        (VesselState(), constant_reference(speed=-1), 0.1),
    ]:
        result = controller.compute(state, trajectory, dt)
        assert result.status in (AlgorithmStatus.INVALID_INPUT, AlgorithmStatus.NO_PATH)
        assert (result.left, result.right, result.lateral) == (0, 0, 0)


@pytest.mark.parametrize("kind", [PIDController, LQRController])
def test_goal_checks_global_goal_instead_of_local_horizon_endpoint(kind):
    controller = kind(USVModel())
    reached = controller.compute(VesselState(), constant_reference(goal=(0.1, 0.2)), 0.1)
    assert reached.status == AlgorithmStatus.GOAL_REACHED
    assert reached.left == reached.right == 0
    # A coincident horizon endpoint does not imply completion of an unknown goal.
    reference = constant_reference(goal=None)
    assert controller.compute(VesselState(x=8), reference, 0.1).status != AlgorithmStatus.GOAL_REACHED


@pytest.mark.parametrize("kind", [PIDController, LQRController])
def test_goal_is_not_latched_while_vessel_can_coast_outside_tolerance(kind):
    controller = kind(USVModel(), {"goal_distance_tolerance": 1., "goal_speed_tolerance": .25})
    trajectory = constant_reference(goal=(8., 0.))
    # Reproduce the baseline failure's high-speed entry into the goal disk.
    moving = controller.compute(VesselState(x=7.8, u=1.49), trajectory, .1)
    assert moving.status == AlgorithmStatus.SETTLING
    assert moving.left == moving.right == 0.
    assert "settling" in moving.message
    # Completion is allowed only while both position and speed meet the gates.
    settled = controller.compute(VesselState(x=8.1, u=.1, v=.1), trajectory, .1)
    assert settled.status == AlgorithmStatus.GOAL_REACHED
    assert controller.compute(VesselState(x=10.7, u=.05), trajectory, .1).status != AlgorithmStatus.GOAL_REACHED


def test_pid_outputs_are_bounded_and_body_speed_is_heading_invariant():
    controller = PIDController(USVModel(), {"k_p_speed": 100})
    a = controller.compute(VesselState(u=0.2, v=0.1), constant_reference(), 0.1)
    b = controller.compute(VesselState(psi=math.pi / 2, u=0.2, v=0.1), constant_reference(yaw=math.pi / 2 + 0.1), 0.1)
    assert a.left == b.left == 1
    assert a.right == b.right == 1


def test_reference_ages_with_state_timestamp():
    controller = PIDController(USVModel(), {"lookahead_time": 0, "k_p_speed": 1})
    trajectory = Trajectory([[0, 0, 0, 0, 0, 0, 0, 0], [2, 1, 1, 0, 0, 0, 1, 0]], stamp=10)
    result = controller.compute(VesselState(stamp=11), trajectory, 0.1)
    assert result.left == result.right == pytest.approx(0.25)


def test_lqr_retains_original_discrete_model_coefficients():
    controller = LQRController(USVModel())
    A, B = controller.discrete_model(TrajectoryCell(theta=0, velocity=1))
    expected_A = np.eye(6)
    expected_A[0, 3] = expected_A[1, 2] = expected_A[1, 4] = expected_A[2, 5] = 0.1
    expected_A[3, 3] = expected_A[4, 4] = 1 - 2 / 34.84
    expected_A[3, 5] = 2.484 / 34.84
    expected_A[4, 5] = -2.484 / 34.84
    expected_A[5, 5] = 1 - 2 / 11.03
    np.testing.assert_allclose(A, expected_A)
    expected_B = np.zeros((6, 2))
    expected_B[3, :] = 5 / 34.84
    expected_B[5, :] = [-1.975 / 11.03, 1.975 / 11.03]
    np.testing.assert_allclose(B, expected_B)


def test_lqr_converges_to_stabilizing_gain_and_zero_tracking_error():
    controller = LQRController(USVModel(), {"solve_eps": 1e-8, "max_iterations": 10000, "lookahead_time": 0})
    reference = TrajectoryCell(theta=0.1, velocity=0.8)
    K = controller.solve_gain(reference)
    A, B = controller.discrete_model(reference)
    assert np.max(np.abs(np.linalg.eigvals(A + B @ K))) < 1
    assert controller.last_residual < 1e-8
    result = controller.compute(VesselState(psi=0.1, u=0.8), constant_reference(), 0.1)
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.left == pytest.approx(0)
    assert result.right == pytest.approx(0)


def test_lqr_nonconvergence_and_deadline_never_publish_partial_gain():
    for cfg in [{"max_iterations": 1}, {"solve_timeout": 1e-15}]:
        result = LQRController(USVModel(), cfg).compute(VesselState(), constant_reference(), 0.1)
        assert result.status == AlgorithmStatus.TIMEOUT
        assert result.left == result.right == 0
        assert result.message


def test_lqr_uses_signed_body_velocities_without_rotating_them_again(monkeypatch):
    controller = LQRController(USVModel(), {"lookahead_time": 0})
    # Isolate state conversion from the solver: direct gains expose u and v.
    gain = np.zeros((2, 6))
    gain[0, 3] = gain[1, 4] = 1
    monkeypatch.setattr(controller, "solve_gain", lambda reference: gain)
    result = controller.compute(VesselState(psi=math.pi / 2, u=-0.2, v=0.3),
                                constant_reference(speed=0, yaw=math.pi / 2), 0.1)
    assert result.left == pytest.approx(-0.2)
    assert result.right == pytest.approx(0.3)


@pytest.mark.parametrize("config", [{"Q_matrix_diag": [1, 1]}, {"R_matrix_diag": [0, 1]},
                                     {"Q_matrix_diag": [1, 1, 1, 1, 1, -1]},
                                     {"max_iterations": 1.5}, {"discrete_dt": 0}])
def test_lqr_rejects_invalid_dimensions_and_solver_configuration(config):
    with pytest.raises(ValueError):
        LQRController(USVModel(), config)


def test_factories_reject_unsupported_algorithms_and_models():
    with pytest.raises(ValueError):
        create_controller("mpc", USVModel())
    with pytest.raises(ValueError):
        create_controller("pid", USVModel(name="other"))


@pytest.mark.parametrize("kind", [PIDController, LQRController])
@pytest.mark.parametrize("age", [0.0, 1.0])
def test_preview_of_predicted_arrival_does_not_stop_before_actual_goal(kind, age):
    model = USVModel()
    initial = VesselState(x=9.3, y=10.25, stamp=10.0)
    plan = LosTrajectoryPlanner(model, {"goal_tolerance": 1.0}).plan(
        [[9.3, 10.25, 0], [10.49, 10.49, 0]], initial).trajectory
    controller = kind(model, {"goal_distance_tolerance": 1.0, "lookahead_time": 3.0,
                              "k_p_speed": 2.0, "k_p_course": 2.0})
    state = VesselState(x=initial.x, y=initial.y, stamp=10.0 + age)
    command = controller.compute(state, plan, 0.1)
    assert command.status == AlgorithmStatus.SUCCESS
    assert max(abs(command.left), abs(command.right)) > 0
    arrived = controller.compute(VesselState(x=10.49, y=10.49, stamp=10.0 + age), plan, 0.1)
    assert arrived.status == AlgorithmStatus.GOAL_REACHED
    assert arrived.left == arrived.right == 0


def test_preview_stop_away_from_goal_is_not_overridden():
    controller = PIDController(USVModel(), {"k_p_speed": 2, "lookahead_time": 3})
    intermediate_stop = Trajectory([[0, 0, 0, 0, 0, 0, 1, 0],
                                    [0.5, 0.5, 0.5, 0, 0, 0, 0, 0]], goal=(10, 0))
    command = controller.compute(VesselState(), intermediate_stop, 0.1)
    assert command.status == AlgorithmStatus.SUCCESS
    assert command.left == command.right == 0
