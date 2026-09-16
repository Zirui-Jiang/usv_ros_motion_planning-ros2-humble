# 验证 PID/LQR 的历史数值契约、预瞄终点语义、非法输入零输出、在线更新与求解边界。
import math

import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from controller import PIDController, LQRController, create_controller
from trajectory_planner import Trajectory, TrajectoryCell, LosTrajectoryPlanner


def constant_reference(speed=0.8, yaw=0.1, goal=None, stamp=0.0):
    return Trajectory([[0, 0, 0, 0, yaw, 0, speed, 0],
                       [10, 8, 8, 0, yaw, 0, speed, 0]], stamp=stamp, frame_id="otter/map", goal=goal)


# 核对 PID 差动分配公式及积分项的逐拍累积。
def test_pid_matches_legacy_allocation_and_integral_update():
    controller = PIDController(USVModel(), {"k_p_speed": 0.4, "k_i_speed": 0.2, "k_p_course": 0.5})
    result = controller.compute(VesselState(u=0.2), constant_reference(), 0.1)
    expected_surge = 0.4 * 0.6 + 0.2 * 0.6 * 0.1
    expected_yaw = 0.5 * 0.1
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.left == pytest.approx((expected_surge - expected_yaw / 0.395) / 2)
    assert result.right == pytest.approx((expected_surge + expected_yaw / 0.395) / 2)
    assert controller.integral_speed == pytest.approx(0.012)


# 验证成功调参保留积分，失败更新不改已有配置。
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


# 空轨迹、非有限状态、非正时间步、异系和倒车参考均返回零命令。
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


# 真实全局目标决定到达状态，局部时域终点不代替任务目标。
@pytest.mark.parametrize("kind", [PIDController, LQRController])
def test_goal_checks_global_goal_instead_of_local_horizon_endpoint(kind):
    controller = kind(USVModel())
    reached = controller.compute(VesselState(), constant_reference(goal=(0.1, 0.2)), 0.1)
    assert reached.status == AlgorithmStatus.GOAL_REACHED
    assert reached.left == reached.right == 0
    # A coincident horizon endpoint does not imply completion of an unknown goal.
    reference = constant_reference(goal=None)
    assert controller.compute(VesselState(x=8), reference, 0.1).status != AlgorithmStatus.GOAL_REACHED


# 高速入目标盘只进入 SETTLING，漂出后仍允许继续控制。
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


# 验证命令限幅及船体系速度模不随世界航向改变。
def test_pid_outputs_are_bounded_and_body_speed_is_heading_invariant():
    controller = PIDController(USVModel(), {"k_p_speed": 100})
    a = controller.compute(VesselState(u=0.2, v=0.1), constant_reference(), 0.1)
    b = controller.compute(VesselState(psi=math.pi / 2, u=0.2, v=0.1), constant_reference(yaw=math.pi / 2 + 0.1), 0.1)
    assert a.left == b.left == 1
    assert a.right == b.right == 1


# 验证预瞄点随状态时间相对轨迹源时间推进。
def test_reference_ages_with_state_timestamp():
    controller = PIDController(USVModel(), {"lookahead_time": 0, "k_p_speed": 1})
    trajectory = Trajectory([[0, 0, 0, 0, 0, 0, 0, 0], [2, 1, 1, 0, 0, 0, 1, 0]], stamp=10)
    result = controller.compute(VesselState(stamp=11), trajectory, 0.1)
    assert result.left == result.right == pytest.approx(0.25)


# 逐项核对保留的 LQR 离散 A/B 系数。
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


# 验证收敛增益使离散闭环特征值模小于一，零跟踪误差产生零输入。
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


# 迭代不足或求解超时必须返回零命令，不使用部分增益。
def test_lqr_nonconvergence_and_deadline_never_publish_partial_gain():
    for cfg in [{"max_iterations": 1}, {"solve_timeout": 1e-15}]:
        result = LQRController(USVModel(), cfg).compute(VesselState(), constant_reference(), 0.1)
        assert result.status == AlgorithmStatus.TIMEOUT
        assert result.left == result.right == 0
        assert result.message


# 直接增益暴露 u/v，检查船体系速度不被重复旋转且保留负号。
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


# 拒绝 Q/R 维度、正定性和求解参数错误。
@pytest.mark.parametrize("config", [{"Q_matrix_diag": [1, 1]}, {"R_matrix_diag": [0, 1]},
                                     {"Q_matrix_diag": [1, 1, 1, 1, 1, -1]},
                                     {"max_iterations": 1.5}, {"discrete_dt": 0}])
def test_lqr_rejects_invalid_dimensions_and_solver_configuration(config):
    with pytest.raises(ValueError):
        LQRController(USVModel(), config)


# 拒绝未实现算法和不支持船型。
def test_factories_reject_unsupported_algorithms_and_models():
    with pytest.raises(ValueError):
        create_controller("mpc", USVModel())
    with pytest.raises(ValueError):
        create_controller("pid", USVModel(name="other"))


# 预测参考到达不能使真实船舶提前停车。
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


# 途中停车参考不被目标接近逻辑擅自恢复速度。
def test_preview_stop_away_from_goal_is_not_overridden():
    controller = PIDController(USVModel(), {"k_p_speed": 2, "lookahead_time": 3})
    intermediate_stop = Trajectory([[0, 0, 0, 0, 0, 0, 1, 0],
                                    [0.5, 0.5, 0.5, 0, 0, 0, 0, 0]], goal=(10, 0))
    command = controller.compute(VesselState(), intermediate_stop, 0.1)
    assert command.status == AlgorithmStatus.SUCCESS
    assert command.left == command.right == 0


# 目标半径小于或等于 LOS 去重尺度时必须拒绝。
@pytest.mark.parametrize("kind", [PIDController, LQRController])
@pytest.mark.parametrize("tolerance", [0., 1e-10, 1e-9])
def test_goal_distance_tolerance_rejects_unresolvable_los_radius(kind, tolerance):
    with pytest.raises(ValueError, match=r"goal_distance_tolerance.*greater than.*1e-9"):
        kind(USVModel(), {"goal_distance_tolerance": tolerance})


# 拒绝不合法目标半径更新时保留 PID 参数及历史。
def test_rejected_goal_tolerance_update_preserves_pid_configuration_and_history():
    controller = PIDController(USVModel(), {"k_i_speed": .2})
    controller.compute(VesselState(), constant_reference(), .1)
    previous_config = dict(controller.config)
    previous_integral = controller.integral_speed
    with pytest.raises(ValueError, match=r"goal_distance_tolerance.*greater than.*1e-9"):
        controller.update_config({"goal_distance_tolerance": 1e-10, "k_p_speed": .3})
    assert controller.config == previous_config
    assert controller.integral_speed == previous_integral


# 验证最小允许半径仍覆盖被 LOS 去重合并的目标位置差。
@pytest.mark.parametrize("kind", [PIDController, LQRController])
def test_smallest_admitted_goal_radius_covers_los_duplicate_span(kind):
    tolerance = np.nextafter(1e-9, np.inf)
    model = USVModel()
    state = VesselState(x=10.5, y=10.5, stamp=10.)
    goal = (10.5000000005, 10.5)
    trajectory = LosTrajectoryPlanner(model, {"goal_tolerance": tolerance}).plan(
        [[10.5, 10.5, 0.], [*goal, 0.]], state).trajectory
    trajectory.goal = goal
    controller = kind(model, {"goal_distance_tolerance": tolerance})
    assert controller.config["goal_distance_tolerance"] == tolerance
    command = controller.compute(state, trajectory, .1)
    assert command.status == AlgorithmStatus.GOAL_REACHED
    assert command.left == command.right == 0.
