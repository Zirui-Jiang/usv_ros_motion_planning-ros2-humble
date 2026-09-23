# 验证八字段轨迹插值、角度跨界、重复弧长、只读快照与输入约束。
import math

import numpy as np
import pytest

from trajectory_planner.trajectory import Trajectory, restart_trajectory_clock


# 时间和弧长查询用正确下包围点插值全部八列。
def test_queries_use_lower_bracket_and_interpolate_all_eight_fields():
    points = [[0, 0, 0, 0, 0, 0, 0, 0],
              [1, 2, 10, 20, 0.2, 0.4, 0.6, 0.8],
              [3, 6, 100, 200, 0.6, 0.9, 1.2, 1.5]]
    trajectory = Trajectory(points)
    np.testing.assert_allclose(trajectory.reference_at_time(0.25), np.asarray(points[1]) * 0.25)
    np.testing.assert_allclose(trajectory.reference_at_distance(0.5), np.asarray(points[1]) * 0.25)
    assert trajectory.reference_at_time(99) == trajectory[-1]
    assert trajectory.reference_at_distance(99) == trajectory[-1]
    assert trajectory.reference_at_time(1) == trajectory[1]


# 跨 ±π 的航向沿短弧插值。
def test_heading_interpolation_uses_short_arc_across_pi():
    trajectory = Trajectory([[0, 0, 0, 0, math.radians(179), 0, 1, 0],
                             [1, 1, 1, 0, math.radians(-179), 0, 1, 0]])
    assert abs(abs(trajectory.reference_at_time(0.5).theta) - math.pi) < 1e-12


# 停车造成重复弧长时查询仍有定义。
def test_repeated_distance_during_stationary_motion_is_defined():
    trajectory = Trajectory([[0, 0, 0, 0, 0, 0, 0, 0],
                             [1, 0, 0, 0, 0, 0, 0, 0],
                             [2, 1, 1, 0, 0, 0, 1, 1]])
    assert trajectory.reference_at_distance(0.5).time == 1.5


# 源数组和回调不能修改已建立轨迹快照。
def test_snapshot_is_copied_and_cannot_be_mutated_by_callback():
    points = np.zeros((1, 8))
    trajectory = Trajectory(points, stamp=10, frame_id="otter/map", goal=(2, 3))
    points[0, 2] = 100
    assert trajectory[0].x == 0
    with pytest.raises(ValueError):
        trajectory.points[0, 2] = 10


# 拒绝负数及非有限查询值。
@pytest.mark.parametrize("value", [-1, float("nan"), float("inf")])
def test_invalid_reference_is_rejected(value):
    with pytest.raises(ValueError):
        Trajectory(np.zeros((1, 8))).reference_at_time(value)


# 拒绝错误形状、非有限字段、非递增时间和空轨迹查询。
def test_invalid_shapes_nonfinite_and_unordered_time_rejected():
    for values in [np.zeros((2, 3)), np.full((1, 8), np.nan), np.zeros((2, 8))]:
        with pytest.raises(ValueError):
            Trajectory(values)
    with pytest.raises(ValueError, match="empty"):
        Trajectory().reference_at_time(0)


# 全局搜索结束后改时钟，相对时间不变，旧的搜索起点时间不再保留。
def test_restart_clock_keeps_samples_and_uses_the_new_stamp():
    original = Trajectory([[0, 0, 1, 2, 0, 0, 1, 0], [1, 1, 3, 2, 0, 0, 1, 0]],
                          stamp=10, frame_id="otter/map", goal=(3, 2))
    restarted = restart_trajectory_clock(original, 18.5)
    assert restarted.stamp == 18.5
    assert restarted.goal == (3, 2)
    np.testing.assert_allclose(restarted.points, original.points)
    assert original.stamp == 10
