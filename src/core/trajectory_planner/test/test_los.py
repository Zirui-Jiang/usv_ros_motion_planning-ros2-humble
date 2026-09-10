import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner import LosTrajectoryPlanner, create_trajectory_planner


def test_los_defines_heading_error_far_from_waypoint():
    planner = LosTrajectoryPlanner(USVModel(), {"trajectory_time": 1.0})
    result = planner.plan([[0, 0, 0], [100, 0, 0]], VesselState(y=4))
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[1].theta < 0
    assert np.isfinite(result.trajectory.points).all()
    assert result.trajectory.goal == (100, 0)


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


def test_curved_long_route_never_creates_negative_reference_speed():
    path = [[0, 0, 0], [20, 0, 0], [20, 20, 0], [0, 3, 0]]
    result = LosTrajectoryPlanner(USVModel()).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert np.all(result.trajectory.points[:, 6] >= 0)
    assert np.isfinite(result.trajectory.points).all()
    assert result.trajectory.points[-1, 1] > 3


def test_goal_coincident_and_duplicate_waypoints_are_finite_stationary_plan():
    result = LosTrajectoryPlanner(USVModel()).plan([[2, 3, 0], [2, 3, 0]], VesselState(x=2, y=3))
    assert result.status == AlgorithmStatus.GOAL_REACHED
    assert len(result.trajectory) == 2
    assert np.all(result.trajectory.points[:, 6:] == 0)


def test_empty_malformed_and_nonfinite_inputs_fail_without_throwing():
    planner = LosTrajectoryPlanner(USVModel())
    assert planner.plan([], VesselState()).status == AlgorithmStatus.NO_PATH
    assert planner.plan([[0, 1]], VesselState()).status == AlgorithmStatus.INVALID_INPUT
    assert planner.plan([[0, 0, 0]], VesselState(x=float("nan"))).status == AlgorithmStatus.INVALID_INPUT


def test_bad_algorithm_and_configuration_fail_early():
    with pytest.raises(ValueError, match="unsupported"):
        create_trajectory_planner("nonexistent", USVModel())
    with pytest.raises(ValueError):
        LosTrajectoryPlanner(USVModel(), {"trajectory_time_interval": 0})


def test_stationary_near_goal_still_has_positive_initial_reference_speed():
    state = VesselState(x=9.3, y=10.25)
    result = LosTrajectoryPlanner(USVModel(), {"goal_tolerance": 1.0}).plan(
        [[9.3, 10.25, 0], [10.49, 10.49, 0]], state)
    assert result.status == AlgorithmStatus.SUCCESS
    assert len(result.trajectory) == 2
    assert result.trajectory[0].velocity == pytest.approx(1.0)
    assert result.trajectory[-1].velocity == 0
