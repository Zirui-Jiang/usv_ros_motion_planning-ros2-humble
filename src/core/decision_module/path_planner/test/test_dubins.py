# 独立验证 Dubins 曲线的终点、尺度、镜像与常曲率采样，防止靠覆盖末点掩盖几何错误。
"""Geometric contracts for forward-only Dubins connections."""

import importlib
import math

import numpy as np
import pytest


def test_dubins_geometry_module_is_available():
    assert importlib.util.find_spec("path_planner.graph_planner.dubins") is not None


@pytest.fixture
def dubins():
    # 首轮测试以明确断言报告缺失功能，避免导入错误阻止其他测试收集。
    name = "path_planner.graph_planner.dubins"
    assert importlib.util.find_spec(name) is not None, "Dubins geometry module is missing"
    return importlib.import_module(name)


def assert_pose(actual, expected, atol=1e-8):
    np.testing.assert_allclose(actual[:2], expected[:2], atol=atol, rtol=0)
    angle_error = math.atan2(math.sin(actual[2] - expected[2]), math.cos(actual[2] - expected[2]))
    assert abs(angle_error) <= atol


# 只使用复数平面积分检查解析解；不调用被测 advance_pose / sample_dubins。
def integrate_segments(start, path):
    point = complex(start[0], start[1])
    heading = start[2]
    for mode, distance in zip(path.modes, path.lengths):
        sign = {"L": 1, "R": -1, "S": 0}[mode]
        if sign:
            delta = sign * distance / path.radius
            step = complex(math.sin(delta), 1 - math.cos(delta)) * (path.radius / sign)
            point += complex(math.cos(heading), math.sin(heading)) * step
            heading += delta
        else:
            point += distance * complex(math.cos(heading), math.sin(heading))
    return np.array([point.real, point.imag, heading])


def test_straight_connection_is_shortest_and_keeps_endpoints(dubins):
    paths = dubins.dubins_paths((1, 2, 0), (13, 2, 0), 2.0)
    assert paths[0].length == pytest.approx(12.0)
    assert [p.length for p in paths] == sorted(p.length for p in paths)
    samples = dubins.sample_dubins((1, 2, 0), paths[0], 0.7)
    assert_pose(samples[0], (1, 2, 0))
    assert_pose(samples[-1], (13, 2, 0))
    np.testing.assert_allclose(samples[:, 1], 2)
    assert np.linalg.norm(np.diff(samples[:, :2], axis=0), axis=1).max() <= 0.7 + 1e-12


def test_all_six_types_reach_the_same_query_pose(dubins):
    start, goal = (0, 0, 0.3), (1.3, 1.2, -0.6)
    paths = dubins.dubins_paths(start, goal, 1.0)
    seen = set()
    # 随机例中也验证全部类型；这个近距离例必须至少有两种三圆弧连接。
    for path in paths:
        seen.add("".join(path.modes))
        assert_pose(integrate_segments(start, path), goal)
        assert_pose(dubins.sample_dubins(start, path, 0.11)[-1], goal)
    assert {"LRL", "RLR"} <= seen


def test_random_connections_integrate_to_goal_for_every_returned_family(dubins):
    rng = np.random.default_rng(9374)
    seen = set()
    for _ in range(90):
        start = np.r_[rng.uniform(-10, 10, 2), rng.uniform(-math.pi, math.pi)]
        goal = np.r_[rng.uniform(-10, 10, 2), rng.uniform(-math.pi, math.pi)]
        radius = float(rng.uniform(0.3, 8))
        paths = dubins.dubins_paths(start, goal, radius)
        assert paths
        for path in paths:
            seen.add("".join(path.modes))
            assert all(length >= 0 for length in path.lengths)
            assert path.length + 1e-8 >= np.linalg.norm(goal[:2] - start[:2])
            assert_pose(integrate_segments(start, path), goal)
            assert_pose(dubins.sample_dubins(start, path, 0.4)[-1], goal)
    assert seen == {"LSL", "RSR", "LSR", "RSL", "LRL", "RLR"}


def test_reflection_swaps_left_and_right_without_changing_lengths(dubins):
    start, goal = (1.1, -2.4, 0.3), (3.6, 0.7, 1.2)
    original = dubins.dubins_paths(start, goal, 2.5)
    mirrored = dubins.dubins_paths((start[0], -start[1], -start[2]),
                                  (goal[0], -goal[1], -goal[2]), 2.5)
    reflected = {"".join(p.modes): p for p in mirrored}
    for path in original:
        key = "".join(path.modes).translate(str.maketrans("LR", "RL"))
        np.testing.assert_allclose(path.lengths, reflected[key].lengths, atol=1e-10)


def test_rotation_translation_and_scale_preserve_geometry(dubins):
    start, goal, radius = (1, 2, -0.7), (4, -1, 0.5), 1.7
    angle, scale, offset = 0.8, 3.2, np.array([123.5, -97.0])
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    def transformed(pose):
        return np.r_[scale * (rotation @ np.array(pose[:2])) + offset, pose[2] + angle]
    original = dubins.dubins_paths(start, goal, radius)
    changed = {p.modes: p for p in dubins.dubins_paths(transformed(start), transformed(goal), scale * radius)}
    for path in original:
        np.testing.assert_allclose(changed[path.modes].lengths, scale * np.array(path.lengths), atol=1e-10)
        samples = dubins.sample_dubins(transformed(start), changed[path.modes], 0.6)
        assert_pose(samples[-1], transformed(goal))


def test_identical_pose_has_zero_length_but_opposite_heading_requires_motion(dubins):
    start = (1.2, -3.4, 0.7)
    same = dubins.dubins_paths(start, (1.2, -3.4, 0.7 + 2 * math.pi), 2.0)[0]
    assert same.length == pytest.approx(0, abs=1e-12)
    assert dubins.sample_dubins(start, same, 0.2).shape == (1, 3)
    reverse = (start[0], start[1], start[2] + math.pi)
    path = dubins.dubins_paths(start, reverse, 2.0)[0]
    assert path.length > 2 * math.pi
    samples = dubins.sample_dubins(start, path, 0.2)
    assert np.linalg.norm(samples[:, :2] - start[:2], axis=1).max() > 1.0
    assert_pose(samples[-1], reverse)


@pytest.mark.parametrize("sign", [-1, 1])
def test_quarter_circle_has_requested_radius_and_exact_heading(dubins, sign):
    assert_pose(dubins.advance_pose((0, 0, 0), sign / 3, 1.5 * math.pi),
                (3, sign * 3, sign * math.pi / 2))
    path = dubins.DubinsPath(("L" if sign == 1 else "R",), (1.5 * math.pi,), 3)
    samples = dubins.sample_dubins((0, 0, 0), path, 0.21)
    radius = np.linalg.norm(samples[:, :2] - (0, sign * 3), axis=1)
    np.testing.assert_allclose(radius, 3, atol=1e-12)
    assert_pose(samples[-1], (3, sign * 3, sign * math.pi / 2))


def test_sampling_includes_switch_points_and_has_no_duplicate_zero_segments(dubins):
    path = dubins.DubinsPath(("L", "S", "R", "S"), (math.pi, 1, 0, 0), 2)
    samples = dubins.sample_dubins((0, 0, 0), path, 0.6)
    assert any(np.allclose(row, (2, 2, math.pi / 2)) for row in samples)
    assert_pose(samples[-1], (2, 3, math.pi / 2))
    assert np.all(np.linalg.norm(np.diff(samples[:, :2], axis=0), axis=1) > 0)


@pytest.mark.parametrize("delta", [-1e-10, 0, 1e-10])
def test_degenerate_tangent_and_three_arc_boundaries_reach_target(dubins, delta):
    for goal in ((4 + delta, 0, 0), (2 + delta, 2, math.pi), (1, 1 + delta, math.pi / 2)):
        paths = dubins.dubins_paths((0, 0, 0), goal, 1)
        assert paths
        for path in paths:
            assert_pose(integrate_segments((0, 0, 0), path), goal, atol=3e-8)


@pytest.mark.parametrize('sign', [-1, 1])
def test_nearly_coincident_turn_centres_keep_nonzero_middle_arc(dubins, sign):
    start, goal = (0., 0., 0.), (9.9999998, sign*10., sign*math.pi/2)
    for path in dubins.dubins_paths(start, goal, 10.):
        assert_pose(integrate_segments(start, path), goal, atol=1e-8)


@pytest.mark.parametrize("radius", [0, -1, math.nan, math.inf])
def test_invalid_radius_is_rejected(dubins, radius):
    with pytest.raises(ValueError):
        dubins.dubins_paths((0, 0, 0), (1, 1, 0), radius)


@pytest.mark.parametrize("pose", [(0, 1), (0, 1, 2, 3), (0, math.inf, 0), (0, 0, math.nan)])
def test_invalid_pose_is_rejected(dubins, pose):
    with pytest.raises(ValueError):
        dubins.dubins_paths(pose, (1, 1, 0), 1)
    with pytest.raises(ValueError):
        dubins.advance_pose(pose, 0, 1)


@pytest.mark.parametrize("spacing", [0, -1, math.inf, math.nan, 1e-30])
def test_invalid_or_excessive_sampling_is_rejected(dubins, spacing):
    path = dubins.DubinsPath(("S",), (3,), 1)
    with pytest.raises(ValueError):
        dubins.sample_dubins((0, 0, 0), path, spacing)


@pytest.mark.parametrize("modes,lengths", [(('L',), (-1,)), (('X',), (1,)), (('L',), (math.nan,)), (('S',), ())])
def test_invalid_segment_contract_is_rejected(dubins, modes, lengths):
    with pytest.raises(ValueError):
        dubins.DubinsPath(modes, lengths, 1)


@pytest.mark.parametrize("curvature,distance", [(math.nan, 1), (math.inf, 1), (0, -1), (0, math.inf)])
def test_invalid_motion_is_rejected(dubins, curvature, distance):
    with pytest.raises(ValueError):
        dubins.advance_pose((0, 0, 0), curvature, distance)
