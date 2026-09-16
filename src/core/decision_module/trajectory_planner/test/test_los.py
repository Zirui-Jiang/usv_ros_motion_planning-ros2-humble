# 验证 LOS 导引方向、精确输出时间、非负速度、近目标减速与密集路点切换顺序。
import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner import LosTrajectoryPlanner, create_trajectory_planner


# 远离目标时横向误差仍产生正确导引转向。
def test_los_defines_heading_error_far_from_waypoint():
    planner = LosTrajectoryPlanner(USVModel(), {"trajectory_time": 1.0})
    result = planner.plan([[0, 0, 0], [100, 0, 0]], VesselState(y=4))
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[1].theta < 0
    assert np.isfinite(result.trajectory.points).all()
    assert result.trajectory.goal == (100, 0)


# 各种输出间隔均与实际积分时间匹配，不超过速度允许行程。
@pytest.mark.parametrize("dt", [0.05, 0.07, 0.3, 0.5])
def test_sampling_matches_actual_rollout_time_even_below_inner_step(dt):
    result = LosTrajectoryPlanner(USVModel(), {"trajectory_time_interval": dt, "trajectory_time": 1.0}).plan(
        [[0, 0, 0], [100, 0, 0]], VesselState(u=1))
    points = result.trajectory.points
    expected = np.r_[0.0, np.minimum(np.arange(1, int(np.ceil(1.0 / dt)) + 1) * dt, 1.0)]
    np.testing.assert_allclose(points[:, 0], expected)
    # Straight-line motion cannot advance more than cruising_speed * elapsed.
    assert np.all(points[:, 2] <= points[:, 0] + 1e-10)
    assert points[1, 2] > 0.9 * dt


# 弯曲长路线不产生负参考速度或非有限字段。
def test_curved_long_route_never_creates_negative_reference_speed():
    path = [[0, 0, 0], [20, 0, 0], [20, 20, 0], [0, 3, 0]]
    result = LosTrajectoryPlanner(USVModel()).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert np.all(result.trajectory.points[:, 6] >= 0)
    assert np.isfinite(result.trajectory.points).all()
    assert result.trajectory.points[-1, 1] > 3


# 已在目标且路点重复时生成有限静止轨迹。
def test_goal_coincident_and_duplicate_waypoints_are_finite_stationary_plan():
    result = LosTrajectoryPlanner(USVModel()).plan([[2, 3, 0], [2, 3, 0]], VesselState(x=2, y=3))
    assert result.status == AlgorithmStatus.GOAL_REACHED
    assert len(result.trajectory) == 2
    assert np.all(result.trajectory.points[:, 6:] == 0)


# 空、畸形和非有限输入返回明确失败状态。
def test_empty_malformed_and_nonfinite_inputs_fail_without_throwing():
    planner = LosTrajectoryPlanner(USVModel())
    assert planner.plan([], VesselState()).status == AlgorithmStatus.NO_PATH
    assert planner.plan([[0, 1]], VesselState()).status == AlgorithmStatus.INVALID_INPUT
    assert planner.plan([[0, 0, 0]], VesselState(x=float("nan"))).status == AlgorithmStatus.INVALID_INPUT


# 未知算法和无效输出时间配置提前拒绝。
def test_bad_algorithm_and_configuration_fail_early():
    with pytest.raises(ValueError, match="unsupported"):
        create_trajectory_planner("nonexistent", USVModel())
    with pytest.raises(ValueError):
        LosTrajectoryPlanner(USVModel(), {"trajectory_time_interval": 0})


# 目标外静止船仍得到正初始接近速度。
def test_stationary_near_goal_still_has_positive_initial_reference_speed():
    state = VesselState(x=9.3, y=10.25)
    result = LosTrajectoryPlanner(USVModel(), {"goal_tolerance": 1.0}).plan(
        [[9.3, 10.25, 0], [10.49, 10.49, 0]], state)
    assert result.status == AlgorithmStatus.SUCCESS
    distance = np.linalg.norm([10.49 - state.x, 10.49 - state.y])
    assert result.trajectory[0].velocity == pytest.approx(distance / 3.0)
    assert result.trajectory[-1].velocity == 0


# 临近目标反复规划应继续减速，不恢复巡航速度。
def test_near_goal_replanning_reduces_initial_speed_without_reset_to_cruise():
    planner = LosTrajectoryPlanner(USVModel(), {"slowdown_distance": 3.0})
    speeds = []
    for x in [0., 7., 8., 8.8]:
        result = planner.plan([[x, 0., 0.], [10., 0., 0.]], VesselState(x=x))
        assert result.status == AlgorithmStatus.SUCCESS
        speeds.append(result.trajectory[0].velocity)
        # The first integration step uses the same profile as the first row.
        assert result.trajectory[1].x - x <= .5 * speeds[-1] + 1e-10
    np.testing.assert_allclose(speeds, [1., 1., 2. / 3., .4])


# 减速距离下限不改变远场原有速度比例。
def test_slowdown_floor_preserves_far_field_legacy_ratio():
    path, state = [[0., 0., 0.], [100., 0., 0.]], VesselState()
    a = LosTrajectoryPlanner(USVModel(), {"slowdown_distance": 3.}).plan(path, state)
    b = LosTrajectoryPlanner(USVModel(), {"slowdown_distance": 1e-9}).plan(path, state)
    np.testing.assert_array_equal(a.trajectory.points, b.trajectory.points)


# 密集路点每输出周期最多推进一次，先使用当前段导引。
def test_nearby_waypoints_advance_once_per_output_after_current_segment_guidance():
    # All points are within the 5 m switching radius, but the original LOS
    # must not jump immediately from the two horizontal segments to the bend.
    path = [[0., 0., 0.], [.1, 0., 0.], [.2, 0., 0.], [.2, 1., 0.], [3., 1., 0.]]
    result = LosTrajectoryPlanner(USVModel(), {
        'trajectory_time': 1., 'trajectory_time_interval': .5,
        'lookahead_distance': 1., 'switch_dist': 5.,
    }).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[1].theta == pytest.approx(0., abs=1e-12)
    assert result.trajectory[1].y == pytest.approx(0., abs=1e-12)
    # At the next output boundary the current horizontal heading is computed
    # before advancing, leaving exactly four 0.1 s bounded yaw steps.
    assert result.trajectory[2].theta == pytest.approx(.12, abs=1e-12)
