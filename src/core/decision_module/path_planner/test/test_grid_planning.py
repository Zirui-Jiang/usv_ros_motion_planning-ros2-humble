# 验证三类栅格搜索、边界、路径处理、软膨胀起点逃逸及真实快照回放的数值契约。
"""Numerical and edge-case contracts for migration from the C++ planners."""

import hashlib
import json
import math
from pathlib import Path
import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from path_planner import PathPlannerCore, create_planner, prune, resample
from path_planner.path_planner import grid_to_index, index_to_grid
from path_planner.path_planner_core import path_is_clear_with_inflation_escape


def grid(data=None, resolution=1.0, origin_x=0.0, origin_y=0.0):
    return GridSnapshot(np.zeros((8, 8), dtype=np.uint8) if data is None else np.asarray(data, dtype=np.uint8),
                        resolution=resolution, origin_x=origin_x, origin_y=origin_y)


# 空地图路径长度与三次对角加两次轴向步的解析结果一致。
@pytest.mark.parametrize("name", ["astar", "dijkstra", "gbfs"])
def test_open_grid_matches_analytic_optimal_path(name):
    result = PathPlannerCore({"planner_name": name}).make_plan((0.5, 0.5, 0.3), (5.5, 3.5, -0.6), grid())
    assert result.status == AlgorithmStatus.SUCCESS
    np.testing.assert_allclose(result.path[[0, -1]], [[0.5, 0.5, 0.3], [5.5, 3.5, -0.6]])
    # Three diagonal and two axis-aligned steps are necessary and sufficient.
    length = np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1).sum()
    assert length == pytest.approx(3 * math.sqrt(2) + 2)
    assert result.expanded > 0 and result.elapsed >= 0


# 贯通障碍墙必须无路。
@pytest.mark.parametrize("name", ["astar", "dijkstra", "gbfs"])
def test_blocking_wall_has_no_path(name):
    data = np.zeros((8, 8), dtype=np.uint8)
    data[:, 4] = 254
    result = PathPlannerCore({"planner_name": name}).make_plan((1.5, 1.5, 0), (6.5, 1.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH
    assert result.path.shape == (0, 3)


# 二维邻接不能把线性索引跨行误当相邻。
def test_two_dimensional_bounds_cannot_alias_across_rows():
    # The only free cells have adjacent *linear* indices (2 and 3), but are
    # separated by blocked neighbours in the actual 3-column map.
    data = np.full((3, 3), 254, dtype=np.uint8)
    data[0, 2] = data[1, 0] = 0
    result = PathPlannerCore().make_plan((2.5, 0.5, 0), (0.5, 1.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH


# 起点或终点占据时明确失败。
@pytest.mark.parametrize("which", ["start", "goal"])
def test_blocked_endpoint_fails_explicitly(which):
    data = np.zeros((4, 4), dtype=np.uint8)
    data[(0, 0) if which == "start" else (3, 3)] = 254
    result = PathPlannerCore().make_plan((0.5, 0.5, 0), (3.5, 3.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH
    assert "blocked" in result.message


# 非法或越界端点不能被静默投影回地图。
@pytest.mark.parametrize("point", [(-0.01, 0.5, 0), (8, 0.5, 0), (0.5, float("nan"), 0), (1, 2)])
def test_invalid_endpoint_is_not_projected_or_clamped(point):
    result = PathPlannerCore().make_plan(point, (3.5, 3.5, 0), grid())
    assert result.status == AlgorithmStatus.INVALID_INPUT


# 验证未知策略与原有 254×factor 障碍阈值。
def test_unknown_policy_and_original_inflation_threshold():
    data = np.array([[0, 255, 0]], dtype=np.uint8)
    source, target = (0.5, 0.5, 0), (2.5, 0.5, 0)
    assert PathPlannerCore().make_plan(source, target, grid(data)).status == AlgorithmStatus.NO_PATH
    assert PathPlannerCore({"unknown_is_obstacle": False}).make_plan(source, target, grid(data)).success
    data[0, 1] = 127
    assert PathPlannerCore().make_plan(source, target, grid(data)).status == AlgorithmStatus.NO_PATH
    assert PathPlannerCore({"obstacle_factor": 1.0}).make_plan(source, target, grid(data)).success


# 对角切角是否允许由显式选项控制。
def test_diagonal_corner_policy_is_explicit():
    scene = grid([[0, 254], [254, 0]])
    source, target = (0.5, 0.5, 0), (1.5, 1.5, 0)
    assert PathPlannerCore().make_plan(source, target, scene).success
    assert PathPlannerCore({"allow_corner_cutting": False}).make_plan(source, target, scene).status == AlgorithmStatus.NO_PATH


# 地图描边使用独立掩码，不改只读快照。
def test_outline_uses_owned_mask_and_leaves_snapshot_unchanged():
    scene = grid()
    original = scene.data.copy()
    result = PathPlannerCore({"outline_map": True}).make_plan((0.5, 0.5, 0), (2.5, 2.5, 0), scene)
    assert result.status == AlgorithmStatus.NO_PATH
    np.testing.assert_array_equal(scene.data, original)


# 非零原点和小数分辨率正确转换坐标。
def test_nonzero_origin_and_fractional_resolution_are_respected():
    scene = grid(resolution=0.25, origin_x=-5, origin_y=10)
    source, target = (-4.875, 10.125, 1), (-3.875, 11.125, 2)
    result = PathPlannerCore().make_plan(source, target, scene)
    assert result.success
    np.testing.assert_allclose(result.path[[0, -1]], [source, target])
    assert scene.world_to_map(-5.001, 10) is None
    assert scene.world_to_map(-3, 10) is None


# 同格起终点返回单元中心并采用目标航向。
def test_same_grid_cell_has_one_pose_with_goal_heading():
    result = PathPlannerCore().make_plan((0.6, 0.6, 0), (0.8, 0.7, 1.2), grid())
    assert result.success
    np.testing.assert_allclose(result.path, [[0.5, 0.5, 1.2]])


# 扩展预算用尽只返回超时，不泄露部分路径。
def test_expansion_budget_returns_timeout_with_no_partial_path():
    result = PathPlannerCore({"max_expansions": 1}).make_plan((0.5, 0.5, 0), (7.5, 7.5, 0), grid())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 1 and len(result.path) == 0


# 替换计时器以确定性检验规划时间预算。
def test_clock_budget_is_enforced_deterministically(monkeypatch):
    from path_planner.graph_planner import astar_planner
    ticks = iter([10.0, 10.0, 11.1, 11.2, 11.3])
    monkeypatch.setattr(astar_planner.time, "perf_counter", lambda: next(ticks))
    result = PathPlannerCore({"timeout": 1.0}).make_plan((0.5, 0.5, 0), (7.5, 7.5, 0), grid())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.expanded == 0


# 配置错误在构造时立即抛出。
@pytest.mark.parametrize("config", [{"planner_name": "rrt"}, {"timeout": 0}, {"obstacle_factor": 0},
                                     {"obstacle_factor": float("nan")}, {"max_expansions": 1.5},
                                     {"max_expansions": 0}, {"enable_resample": True, "resample_ratio": 0},
                                     {"cost_penalty": -1}, {"default_tolerance": -0.1}])
def test_configuration_errors_do_not_create_half_initialized_planners(config):
    with pytest.raises(ValueError):
        PathPlannerCore(config)


# 二维索引可往返且各轴边界独立验证。
def test_grid_index_round_trip_and_axis_bounds():
    for y in range(3):
        for x in range(5):
            assert index_to_grid(grid_to_index(x, y, 5, 3), 5, 3) == (x, y)
    for x, y in [(-1, 1), (5, 0), (1, -1), (1, 3)]:
        with pytest.raises(ValueError):
            grid_to_index(x, y, 5, 3)
    with pytest.raises(ValueError):
        index_to_grid(15, 5, 3)


# 剪枝沿用最后最近路点规则并返回独立副本。
def test_path_pruning_matches_original_last_nearest_waypoint_rule():
    path = np.array([[0, 0, 0], [1, 0, 0.1], [2, 0, 0.2], [3, 0, 0.3]])
    result = prune(path, VesselState(x=1.5, y=0))
    np.testing.assert_allclose(result, path[2:])
    result[0, 0] = 99
    assert path[2, 0] == 2
    np.testing.assert_allclose(prune(path, (2, 0), prune_distance=1.0), path[1:])
    assert prune([], (0, 0)).shape == (0, 3)


# 重采样保留端点完整位姿和直线内部切线。
def test_resampling_preserves_endpoint_pose_and_tangent_on_straight_line():
    path = [[0, 0, 0.1], [1, 0, 0], [2.2, 0, -0.2]]
    result = resample(path, 0.5)
    np.testing.assert_allclose(result[:, 0], [0, 0.5, 1, 1.5, 2, 2.2])
    np.testing.assert_allclose(result[1:-1, 2], 0)
    np.testing.assert_allclose(result[[0, -1]], np.asarray(path)[[0, -1]])


# 验证圆弧切线估计及重复位置数值稳定性。
def test_resampling_circle_tangent_and_duplicate_points():
    points = np.array([[1, 0, 0.2], [1, 0, 0.2], [0, 1, 0], [-1, 0, 0.5]])
    result = resample(points, math.sqrt(2))
    assert result.shape == (3, 3)
    assert abs(abs(result[1, 2]) - math.pi) < 1e-10
    np.testing.assert_allclose(result[[0, -1]], points[[0, -1]])
    assert np.isfinite(resample([[1, 1, 0], [1, 1, 1]], 0.5)).all()


# 拒绝零、负数与非有限采样间距。
@pytest.mark.parametrize("spacing", [0, -1, float("nan"), float("inf")])
def test_invalid_resampling_spacing_fails(spacing):
    with pytest.raises(ValueError):
        resample([[0, 0], [1, 1]], spacing)


# 门面确实应用启用的重采样间距。
def test_core_resample_configuration_is_applied():
    result = PathPlannerCore({"enable_resample": True, "resample_spacing": 0.25}).make_plan(
        (0.5, 0.5, 0.1), (2.5, 0.5, 0.2), grid())
    assert result.success and len(result.path) == 9
    np.testing.assert_allclose(result.path[[0, -1], 2], [0.1, 0.2])


# 粗重采样穿障时保留原安全栅格路径。
def test_coarse_resampling_cannot_cut_across_an_obstacle():
    from common.math_helper import path_is_clear
    data = np.zeros((7, 7), dtype=np.uint8)
    data[:5, 3] = 254
    scene = grid(data)
    source, target = (0.5, 0.5, 0), (6.5, 0.5, 0)
    assert not path_is_clear(scene, np.asarray([source, target]))
    result = PathPlannerCore({"enable_resample": True, "resample_spacing": 100.0}).make_plan(source, target, scene)
    assert result.success
    assert len(result.path) > 2
    assert path_is_clear(scene, result.path, threshold=127)
    assert "retained" in result.message


# 软膨胀起点可沿严格下降代价退出，且兼容重采样。
@pytest.mark.parametrize("name", ["astar", "dijkstra", "gbfs"])
@pytest.mark.parametrize("resampling", [False, True])
def test_soft_start_restores_original_strictly_descending_escape(name, resampling):
    scene = grid([[120, 100, 75, 50, 0]])
    config = {"planner_name": name, "obstacle_factor": .25,
              "enable_resample": resampling, "resample_spacing": .2}
    result = PathPlannerCore(config).make_plan((.5, .5, .2), (4.5, .5, -.3), scene)
    assert result.success
    assert path_is_clear_with_inflation_escape(scene, result.path, 63.5)
    from common.math_helper import path_is_clear
    assert not path_is_clear(scene, result.path, 63.5)  # Existing strict helper is unchanged.
    np.testing.assert_allclose(result.path[[0, -1]], [[.5, .5, .2], [4.5, .5, -.3]])


# 逃逸不能走代价平台、上坡或离开自由区后重新进入。
@pytest.mark.parametrize("costs", [[120, 120, 0], [120, 121, 0], [120, 75, 75, 0],
                                   [120, 50, 75, 0], [120, 75, 90, 0]])
def test_soft_escape_cannot_plateau_climb_or_reenter_after_reaching_free_space(costs):
    scene = grid([costs])
    path = np.array([[i+.5, .5, 0] for i in range(len(costs))])
    result = PathPlannerCore({"obstacle_factor": .25}).make_plan(path[0], path[-1], scene)
    assert result.status == AlgorithmStatus.NO_PATH
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5)


# 逃逸例外绝不允许致命或默认禁止的未知单元。
@pytest.mark.parametrize("costs", [[254, 120, 0], [255, 120, 0], [253, 254, 0], [253, 255, 0]])
def test_escape_never_admits_hard_or_default_unknown_cells(costs):
    scene = grid([costs])
    path = np.array([[i+.5, .5, 0] for i in range(len(costs))])
    assert PathPlannerCore({"obstacle_factor": .25}).make_plan(path[0], path[-1], scene).status == AlgorithmStatus.NO_PATH
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5)


# 即使起点可逃逸，高代价终点在零容差下仍拒绝。
@pytest.mark.parametrize("costs", [[120], [120, 100, 75], [0, 75]])
def test_soft_goal_is_still_blocked_even_when_start_can_escape(costs):
    scene = grid([costs])
    path = np.array([[i+.5, .5, 0] for i in range(len(costs))])
    result = PathPlannerCore({"obstacle_factor": .25}).make_plan(path[0], path[-1], scene)
    assert result.status == AlgorithmStatus.NO_PATH and "blocked" in result.message
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5)


# 软膨胀终点可在容差内松弛到最近自由格，致命终点不能松弛。
def test_soft_goal_snaps_to_nearest_free_cell_within_tolerance():
    data = np.zeros((7, 7), dtype=np.uint8)
    data[3, 5] = 80
    result = PathPlannerCore({"obstacle_factor": .25, "default_tolerance": 1.5}).make_plan(
        (1.5, 3.5, 0), (5.5, 3.5, 0.4), grid(data))
    assert result.success
    assert not np.allclose(result.path[-1, :2], [5.5, 3.5])
    assert np.linalg.norm(result.path[-1, :2] - [5.5, 3.5]) <= 1.5 + 1e-9
    assert result.path[-1, 2] == pytest.approx(0.4)


def test_lethal_goal_cannot_be_relaxed_by_tolerance():
    data = np.zeros((7, 7), dtype=np.uint8)
    data[3, 5] = 254
    result = PathPlannerCore({"default_tolerance": 5.0}).make_plan(
        (1.5, 3.5, 0), (5.5, 3.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH and "blocked" in result.message


# 边权计入膨胀代价后，宁可走更长的自由绕行也不穿过低膨胀捷径。
def test_cost_penalty_prefers_free_detour_over_inflated_shortcut():
    data = np.zeros((5, 5), dtype=np.uint8)
    data[1:4, 1:4] = 40
    start, goal = (0.5, 2.5, 0), (4.5, 2.5, 0)
    straight = PathPlannerCore({"obstacle_factor": .25}).make_plan(start, goal, grid(data))
    detour = PathPlannerCore({"obstacle_factor": .25, "cost_penalty": 30.0}).make_plan(
        start, goal, grid(data))
    assert straight.success and detour.success
    assert np.allclose(straight.path[:, 1], 2.5)
    assert (np.min(np.abs(detour.path[:, 1] - 0.5)) < 1e-9
            or np.min(np.abs(detour.path[:, 1] - 4.5)) < 1e-9)


# 显式可通行未知格不成为重新进入膨胀区的入口。
def test_unknown_explicitly_free_does_not_restart_inflation_escape():
    scene = grid([[120, 255, 75, 0]])
    path = np.array([[i+.5, .5, 0] for i in range(4)])
    result = PathPlannerCore({"obstacle_factor": .25, "unknown_is_obstacle": False}).make_plan(path[0], path[-1], scene)
    assert result.status == AlgorithmStatus.NO_PATH
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5, False)


# 软起点规则同样不能绕过跨行二维边界。
def test_soft_escape_cannot_alias_linear_indices_across_row_boundary():
    data = np.full((3, 3), 254, dtype=np.uint8)
    data[0, 2], data[1, 0] = 120, 75
    data[2, 0] = 0
    result = PathPlannerCore({"obstacle_factor": .25}).make_plan((2.5, .5, 0), (.5, 2.5, 0), grid(data))
    assert result.status == AlgorithmStatus.NO_PATH


# 地图描边上的软起点仍视为硬边界。
def test_outline_soft_start_remains_a_hard_boundary():
    scene = grid([[120, 0, 0], [0, 0, 0], [0, 0, 0]])
    path = np.array([[.5, .5, 0], [1.5, 1.5, 0]])
    result = PathPlannerCore({"obstacle_factor": .25, "outline_map": True}).make_plan(path[0], path[-1], scene)
    assert result.status == AlgorithmStatus.NO_PATH
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5, outline_map=True)


# 软逃逸保留对角切角开关。
def test_escape_preserves_diagonal_corner_cutting_option():
    scene = grid([[120, 254], [254, 0]])
    path = np.array([[.5, .5, 0], [1.5, 1.5, 0]])
    assert PathPlannerCore({"obstacle_factor": .25}).make_plan(path[0], path[-1], scene).success
    assert PathPlannerCore({"obstacle_factor": .25, "allow_corner_cutting": False}).make_plan(path[0], path[-1], scene).status == AlgorithmStatus.NO_PATH
    assert path_is_clear_with_inflation_escape(scene, path, 63.5)
    assert not path_is_clear_with_inflation_escape(scene, path, 63.5, allow_corner_cutting=False)


# 粗重采样不能跳过必须逐格下降的逃逸前缀。
def test_coarse_resampling_cannot_shortcut_the_descending_escape_prefix():
    data = np.full((3, 4), 253, dtype=np.uint8)
    data[0, :3], data[1, 2], data[2, 2:] = [120, 110, 100], 80, [0, 0]
    scene = grid(data)
    source, target = (.5, .5, 0), (3.5, 2.5, 0)
    assert not path_is_clear_with_inflation_escape(scene, [source, target], 63.5)
    result = PathPlannerCore({"obstacle_factor": .25, "allow_corner_cutting": False,
                              "enable_resample": True, "resample_spacing": 100}).make_plan(source, target, scene)
    assert result.success and len(result.path) > 2
    assert "retained" in result.message
    assert path_is_clear_with_inflation_escape(scene, result.path, 63.5, allow_corner_cutting=False)


# 逐段校验捕捉短暂穿过高代价格和离开地图。
def test_segment_validator_detects_short_cell_visits_and_map_exit():
    # A segment can clip a high cell for less than half a cell's travel.
    scene = grid([[0, 127], [0, 0]])
    assert not path_is_clear_with_inflation_escape(scene, [[.9, .9], [1.2, 1.1]], 127)
    assert not path_is_clear_with_inflation_escape(scene, [[.5, .5], [-.01, .5]], 127)


# 先校验保存快照哈希，再检验真实 factor=0.25 软起点场景恢复合法路线。
@pytest.mark.parametrize("suffix", ["c0a7f065", "78bb0382"])
@pytest.mark.parametrize("resampling", [False, True])
def test_real_factor025_soft_start_snapshots_recover_a_valid_grid_route(suffix, resampling):
    directory = Path(__file__).parent / "fixtures" / "factor025_soft_start"
    manifest = json.loads((directory / "manifest.json").read_text())
    for name in (f"state_input_{suffix}.json", f"costmap_input_{suffix}.npz"):
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == manifest["files"][name]["sha256"]
    state = json.loads((directory / f"state_input_{suffix}.json").read_text())
    with np.load(directory / f"costmap_input_{suffix}.npz") as captured:
        assert str(captured["cost_kind"]) == "raw"
        scene = GridSnapshot(captured["data"], float(captured["resolution"]),
                             float(captured["origin_x"]), float(captured["origin_y"]),
                             float(captured["stamp"]), str(captured["frame_id"]))
    source, target = scene.world_to_map(*state["pose"][:2]), scene.world_to_map(*state["goal"][:2])
    assert scene.data[source[1], source[0]] == 75
    assert scene.data[target[1], target[0]] == 0
    result = PathPlannerCore({"obstacle_factor": .25, "enable_resample": resampling,
                              "resample_spacing": .5}).make_plan(state["pose"], state["goal"], scene)
    assert result.success
    assert path_is_clear_with_inflation_escape(scene, result.path, 63.5)
    from common.math_helper import path_is_clear
    assert path_is_clear(scene, result.path, 254)  # No hard/unknown exemption.
    assert not path_is_clear(scene, result.path, 63.5)  # The old strict-start check rejected these inputs.
