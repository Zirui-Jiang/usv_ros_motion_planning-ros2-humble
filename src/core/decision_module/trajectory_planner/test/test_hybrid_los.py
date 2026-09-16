"""Dense Hybrid samples must not make LOS waypoint progression lag behind."""
import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner.los_trajectory_planner import LosTrajectoryPlanner


def test_dense_straight_route_advances_like_the_same_sparse_route():
    planner = LosTrajectoryPlanner(USVModel(), {'follow_dense_path': True,
                                               'trajectory_time': 8.})
    x = np.linspace(0., 30., 301)
    dense = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    sparse = dense[[0, -1]]
    full = planner.plan(dense, VesselState())
    short = planner.plan(sparse, VesselState())
    assert full.status == short.status == AlgorithmStatus.SUCCESS
    np.testing.assert_allclose(full.trajectory.points, short.trajectory.points, atol=1e-10)


def test_dense_bend_switches_heading_after_passing_the_corner():
    x = np.linspace(0., 4., 41)
    y = np.linspace(.1, 10., 100)
    path = np.vstack((np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
                      np.column_stack((np.full_like(y, 4.), y, np.full_like(y, np.pi/2)))))
    planner = LosTrajectoryPlanner(USVModel(), {'follow_dense_path': True,
                                               'trajectory_time': 10.})
    result = planner.plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[-1].y > 1.
    assert result.trajectory[-1].theta > .7


def test_dense_mode_does_not_skip_a_point_only_because_it_is_nearby():
    # Both endpoints lie inside switch_dist, but the corner is still ahead.
    path = np.array([[0., 0., 0.], [2., 0., 0.], [2., 2., np.pi/2]])
    result = LosTrajectoryPlanner(USVModel(), {'follow_dense_path': True,
                                 'trajectory_time': .5}).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[-1].y == 0.
    assert result.trajectory[-1].theta == 0.


def test_dense_route_near_its_goal_still_follows_the_remaining_loop():
    # 起终点相近但中间仍有一圈待走，不能仅按终点欧氏距离宣告到达。
    angle = np.linspace(0., 2*np.pi-.05, 240)
    path = np.column_stack((4*np.sin(angle), 4-4*np.cos(angle), angle))
    result = LosTrajectoryPlanner(USVModel(), {'follow_dense_path': True,
                                  'trajectory_time': 8.}).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[-1].s > 3.
    assert result.trajectory[0].velocity > 0.


def test_dense_route_entering_goal_disk_midway_does_not_stop():
    # 先经过最终目标盘附近，再转弯绕行；仅剩余航路长度允许最终停车。
    path = np.array([[0., 0., 0.], [2., 0., 0.], [8., 0., 0.],
                     [8., 8., np.pi/2], [2., 8., np.pi], [2., 0., -np.pi/2]])
    result = LosTrajectoryPlanner(USVModel(), {'follow_dense_path': True,
                                  'trajectory_time': 8.}).plan(path, VesselState())
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.trajectory[-1].s > 3.


@pytest.mark.parametrize('value', [1, 'true', None])
def test_dense_mode_requires_an_explicit_boolean(value):
    with pytest.raises(ValueError, match='follow_dense_path'):
        LosTrajectoryPlanner(USVModel(), {'follow_dense_path': value})
