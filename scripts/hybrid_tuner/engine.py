# Hybrid A* 离线调参引擎：手绘占据栅格按 Nav2 膨胀公式合成代价，再调用仓库规划器。
"""Numeric workbench for Hybrid A*. No ROS nodes, no process spawn.

The cost synthesis follows nav2_costmap_2d InflationLayer:
lethal 254, inscribed 253, then 252 * exp(-scale * (d - inscribed_radius))
inside inflation_radius. Unknown cells stay 255 and do not seed inflation.
"""
from __future__ import annotations

import heapq
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _relative in (
        "src/core/shared/common", "src/core/shared/usv_navigation",
        "src/core/control_module/controller", "src/core/decision_module/path_planner",
        "src/core/decision_module/path_processor",
        "src/core/decision_module/trajectory_planner",
        "src/core/decision_module/usv_decision"):
    _path = str(ROOT / _relative)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from common.models import GridSnapshot, VesselState
from path_planner import PathPlannerCore
from path_planner.graph_planner.dubins import dubins_paths
from usv_navigation.configuration import resolve_parameters
from usv_decision.runtime import PlanningRequest, plan_navigation

MAP = dict(width=128, height=80, resolution=0.5, origin_x=0.0, origin_y=0.0)
WORLD_WIDTH = MAP["width"] * MAP["resolution"]
WORLD_HEIGHT = MAP["height"] * MAP["resolution"]
FREE, LETHAL, UNKNOWN = 0, 1, 2
HEURISTIC_MODES = ("euclidean", "dubins", "max_dubins_dijkstra")


def empty_occupancy():
    return np.zeros((MAP["height"], MAP["width"]), np.uint8)


def occupancy_from_payload(payload):
    array = np.asarray(payload, dtype=np.uint8)
    if array.ndim == 1:
        if array.size != MAP["height"] * MAP["width"]:
            raise ValueError("occupancy length must equal height*width")
        array = array.reshape(MAP["height"], MAP["width"])
    if array.shape != (MAP["height"], MAP["width"]):
        raise ValueError(f"occupancy must be {MAP['height']}x{MAP['width']}")
    if np.any(array > UNKNOWN):
        raise ValueError("occupancy cells must be 0 (free), 1 (lethal) or 2 (unknown)")
    return np.array(array, dtype=np.uint8, copy=True)


# 按 Nav2 InflationLayer 从致命单元生成软代价；未知格保持 255，不作为膨胀源。
def inflate(occupancy, *, inscribed_radius=1.3, inflation_radius=3.0, cost_scaling_factor=2.0,
            resolution=MAP["resolution"]):
    if not all(math.isfinite(v) and v > 0 for v in (inscribed_radius, inflation_radius, cost_scaling_factor, resolution)):
        raise ValueError("inflation radii, scaling and resolution must be positive and finite")
    inflation_radius = max(float(inflation_radius), float(inscribed_radius))
    occ = np.asarray(occupancy, dtype=np.uint8)
    if occ.ndim != 2 or not occ.size:
        raise ValueError("occupancy must be a nonempty 2-D array")
    costs = np.zeros(occ.shape, dtype=np.uint8)
    lethal = occ == LETHAL
    unknown = occ == UNKNOWN
    costs[lethal] = 254
    costs[unknown] = 255
    if not np.any(lethal):
        return costs
    height, width = occ.shape
    max_cells = int(math.ceil(inflation_radius / resolution))
    dist2 = np.full(occ.shape, np.inf)
    rows, cols = np.nonzero(lethal)
    for row, col in zip(rows.tolist(), cols.tolist()):
        y0, y1 = max(0, row - max_cells), min(height, row + max_cells + 1)
        x0, x1 = max(0, col - max_cells), min(width, col + max_cells + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        np.minimum(dist2[y0:y1, x0:x1], (yy - row) ** 2 + (xx - col) ** 2, out=dist2[y0:y1, x0:x1])
    distance = np.sqrt(dist2) * resolution
    inscribed = (distance > 0) & (distance <= inscribed_radius) & ~lethal & ~unknown
    ring = (distance > inscribed_radius) & (distance <= inflation_radius) & ~lethal & ~unknown
    costs[inscribed] = 253
    if np.any(ring):
        factor = np.exp(-cost_scaling_factor * (distance[ring] - inscribed_radius))
        costs[ring] = (252.0 * factor).astype(np.uint8)
    return costs


def _fill_rect(occupancy, x0, y0, x1, y1, value=LETHAL):
    xa, xb = sorted((int(x0), int(x1)))
    ya, yb = sorted((int(y0), int(y1)))
    occupancy[ya:yb, xa:xb] = value


def presets():
    empty = empty_occupancy()
    island = empty_occupancy()
    _fill_rect(island, 58, 24, 78, 52)
    channel = empty_occupancy()
    _fill_rect(channel, 42, 0, 50, 36)
    _fill_rect(channel, 78, 44, 86, 80)
    pocket = empty_occupancy()
    _fill_rect(pocket, 48, 8, 120, 16)
    _fill_rect(pocket, 112, 8, 124, 68)
    _fill_rect(pocket, 48, 60, 124, 72)
    dock = empty_occupancy()
    _fill_rect(dock, 20, 64, 116, 80)
    _fill_rect(dock, 110, 24, 128, 80)
    return {
        "empty": dict(name="空图", occupancy=empty, start=(8.0, 12.0, 0.0), goal=(54.0, 28.0, 0.0),
                      note="无障碍对照，用来看终点艏向和转弯半径本身会把路拉成什么样。"),
        "island": dict(name="绕岛", occupancy=island, start=(8.0, 12.0, 0.0), goal=(54.0, 28.0, 0.0),
                       note="中间一座岛。对比 Hybrid 圆弧绕行和 A* 折线。"),
        "channel": dict(name="错位窄道", occupancy=channel, start=(8.0, 8.0, 0.0), goal=(56.0, 32.0, 0.0),
                        note="两道错开的墙，逼出连续转向。"),
        "pocket": dict(name="凹形港池", occupancy=pocket, start=(8.0, 18.0, 0.0), goal=(44.0, 18.0, math.pi),
                       note="东面封口、西面开口。目标在池内朝西，用来看 180° 到达艏向要付多少路长。"),
        "dock": dict(name="贴岸靠泊", occupancy=dock, start=(12.0, 10.0, math.pi / 2), goal=(48.0, 28.0, 0.0),
                     note="北面是岸。从朝北驶来、要求朝东贴岸到达，专门观察终点艏向税。"),
    }


def _pose(value, name):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError(f"{name} must be finite (x, y, yaw)")
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    if not (0 <= x <= WORLD_WIDTH and 0 <= y <= WORLD_HEIGHT):
        raise ValueError(f"{name} is outside the {WORLD_WIDTH:.0f}×{WORLD_HEIGHT:.0f} m paper")
    return (x, y, math.remainder(yaw, 2 * math.pi))


def default_params():
    return {
        "planner_name": "hybrid_astar",
        "obstacle_factor": 0.25,
        "timeout": 3.0,
        "max_expansions": 40000,
        "minimum_turning_radius": 4.0,
        "turning_radius_multipliers": [1.0],
        "angle_bins": 72,
        "heuristic_weight": 1.2,
        "motion_step": 1.0,
        "collision_step": 0.2,
        "analytic_expansion_max_length": 20.0,
        "non_straight_penalty": 1.05,
        "change_penalty": 0.2,
        "cost_penalty": 0.0,
        "footprint_padding": 0.0,
        "footprint_length": 2.2,
        "footprint_width": 1.2,
        "unknown_is_obstacle": True,
        "outline_map": True,
        "heuristic": "euclidean",
        "enable_shortcut": True,
        "enable_resample": False,
        "inscribed_radius": 1.3,
        "inflation_radius": 3.0,
        "cost_scaling_factor": 2.0,
        "with_runtime": False,
    }


def _coerce_params(raw):
    params = default_params()
    incoming = dict(raw or {})
    params.update(incoming)
    params["planner_name"] = str(params["planner_name"])
    if params["planner_name"] not in ("hybrid_astar", "astar", "dijkstra", "gbfs"):
        raise ValueError("planner_name must be hybrid_astar, astar, dijkstra or gbfs")
    params["heuristic"] = str(params.get("heuristic") or "euclidean")
    if params["heuristic"] not in HEURISTIC_MODES:
        raise ValueError(f"heuristic must be one of {HEURISTIC_MODES}")
    for key in ("timeout", "obstacle_factor", "minimum_turning_radius", "heuristic_weight",
                "motion_step", "collision_step", "analytic_expansion_max_length",
                "non_straight_penalty", "change_penalty", "cost_penalty", "footprint_padding",
                "footprint_length", "footprint_width", "inscribed_radius", "inflation_radius",
                "cost_scaling_factor"):
        params[key] = float(params[key])
    params["max_expansions"] = int(params["max_expansions"])
    params["angle_bins"] = int(params["angle_bins"])
    for key in ("unknown_is_obstacle", "outline_map", "enable_shortcut", "enable_resample", "with_runtime"):
        params[key] = bool(params[key])
    multipliers = params["turning_radius_multipliers"]
    if isinstance(multipliers, (int, float)) and not isinstance(multipliers, bool):
        multipliers = [multipliers]
    params["turning_radius_multipliers"] = [float(v) for v in multipliers]
    return params


def _planner_config(params):
    config = {
        "planner_name": params["planner_name"],
        "obstacle_factor": params["obstacle_factor"],
        "timeout": params["timeout"],
        "max_expansions": params["max_expansions"],
        "minimum_turning_radius": params["minimum_turning_radius"],
        "turning_radius_multipliers": params["turning_radius_multipliers"],
        "angle_bins": params["angle_bins"],
        "heuristic_weight": params["heuristic_weight"],
        "motion_step": params["motion_step"],
        "collision_step": params["collision_step"],
        "analytic_expansion_max_length": params["analytic_expansion_max_length"],
        "non_straight_penalty": params["non_straight_penalty"],
        "change_penalty": params["change_penalty"],
        "cost_penalty": params["cost_penalty"],
        "footprint_padding": params["footprint_padding"],
        "footprint_length": params["footprint_length"],
        "footprint_width": params["footprint_width"],
        "unknown_is_obstacle": params["unknown_is_obstacle"],
        "outline_map": params["outline_map"],
        "enable_shortcut": params["enable_shortcut"],
        "enable_resample": params["enable_resample"],
    }
    return config


def _grid(costs):
    return GridSnapshot(costs, MAP["resolution"], MAP["origin_x"], MAP["origin_y"])


# 八邻域 Dijkstra 距离场，单位米；与 Hybrid 同一软阈值，供双启发式取 max。
def dijkstra_field(grid, goal, threshold, unknown_is_obstacle):
    height, width = grid.height, grid.width
    dist = np.full((height, width), np.inf)
    cell = grid.world_to_map(float(goal[0]), float(goal[1]))
    if cell is None:
        return dist
    gx, gy = cell
    blocked = grid.data >= threshold
    if unknown_is_obstacle:
        blocked = blocked | (grid.data == 255)
    else:
        blocked = blocked.copy()
        blocked[grid.data == 255] = False
    if blocked[gy, gx]:
        return dist
    dist[gy, gx] = 0.0
    heap = [(0.0, gx, gy)]
    steps = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
    lengths = tuple(grid.resolution * math.hypot(dx, dy) for dx, dy in steps)
    while heap:
        cost, x, y = heapq.heappop(heap)
        if cost > dist[y, x]:
            continue
        for (dx, dy), step in zip(steps, lengths):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height) or blocked[ny, nx]:
                continue
            candidate = cost + step
            if candidate < dist[ny, nx]:
                dist[ny, nx] = candidate
                heapq.heappush(heap, (candidate, nx, ny))
    return dist


def _install_heuristic(planner, mode, grid, goal, params):
    if mode == "euclidean" or params["planner_name"] != "hybrid_astar":
        return False
    field = None
    if mode == "max_dubins_dijkstra":
        field = dijkstra_field(grid, goal, 254.0 * params["obstacle_factor"], params["unknown_is_obstacle"])
    fallback = type(planner)._heuristic

    def patched(self, pose, target):
        curves = dubins_paths(pose, target, self.minimum_turning_radius)
        dubins = min((curve.length for curve in curves), default=0.0)
        if mode == "dubins":
            return dubins
        cell = grid.world_to_map(float(pose[0]), float(pose[1]))
        grid_h = field[cell[1], cell[0]] if cell is not None else math.inf
        if not math.isfinite(grid_h):
            grid_h = fallback(self, pose, target)
        return max(dubins, grid_h)

    planner._heuristic = patched.__get__(planner, type(planner))
    return True


def _restore_heuristic(planner, installed):
    if installed:
        try:
            del planner._heuristic
        except AttributeError:
            pass


def path_length(path):
    if path is None or len(path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1).sum())


def turn_switches(path):
    points = np.asarray(path, dtype=float)
    if len(points) < 3:
        return 0
    delta = (np.diff(points[:, 2]) + math.pi) % (2 * math.pi) - math.pi
    signs = np.sign(np.where(np.abs(delta) < 1e-6, 0.0, delta))
    nonzero = signs[signs != 0]
    if len(nonzero) < 2:
        return 0
    return int(np.sum(nonzero[1:] != nonzero[:-1]))


def min_clearance(path, costs):
    points = np.asarray(path, dtype=float)
    if len(points) == 0:
        return None
    rows, cols = np.nonzero(costs == 254)
    if not len(cols):
        return None
    origin_x, origin_y, resolution = MAP["origin_x"], MAP["origin_y"], MAP["resolution"]
    lethal = np.stack((origin_x + (cols + 0.5) * resolution, origin_y + (rows + 0.5) * resolution), axis=1)
    delta = points[:, None, :2] - lethal[None, :, :]
    return float(np.linalg.norm(delta, axis=2).min())


def _summarize(result, costs):
    path = np.asarray(result.path, dtype=float)
    payload = {
        "status": result.status.value if hasattr(result.status, "value") else str(result.status),
        "message": result.message or "",
        "elapsed": float(result.elapsed),
        "expanded": int(result.expanded),
        "points": int(len(path)),
        "length": path_length(path),
        "turn_switches": turn_switches(path),
        "min_clearance": min_clearance(path, costs) if len(path) else None,
        "path": path.tolist() if len(path) else [],
    }
    return payload


def _runtime_snapshot(params):
    overrides = {
        "planner_name": params["planner_name"],
        "obstacle_factor": params["obstacle_factor"],
        "navigation.planning_timeout": params["timeout"],
        "navigation.max_expansions": params["max_expansions"],
        "navigation.footprint_length": params["footprint_length"],
        "navigation.footprint_width": params["footprint_width"],
        "path_planner.minimum_turning_radius": params["minimum_turning_radius"],
        "path_planner.turning_radius_multipliers": params["turning_radius_multipliers"],
        "path_planner.angle_bins": params["angle_bins"],
        "path_planner.heuristic_weight": params["heuristic_weight"],
        "path_planner.motion_step": params["motion_step"],
        "path_planner.collision_step": params["collision_step"],
        "path_planner.analytic_expansion_max_length": params["analytic_expansion_max_length"],
        "path_planner.non_straight_penalty": params["non_straight_penalty"],
        "path_planner.change_penalty": params["change_penalty"],
        "path_planner.cost_penalty": params["cost_penalty"],
        "path_planner.footprint_padding": params["footprint_padding"],
        "path_planner.outline_map": params["outline_map"],
        "path_planner.enable_shortcut": params["enable_shortcut"],
        "path_planner.enable_resample": params["enable_resample"] if params["planner_name"] != "hybrid_astar" else False,
    }
    return resolve_parameters(overrides)


def plan(occupancy, start, goal, params=None, costs=None):
    params = _coerce_params(params)
    occ = occupancy_from_payload(occupancy)
    if costs is None:
        costs = inflate(occ, inscribed_radius=params["inscribed_radius"],
                        inflation_radius=params["inflation_radius"],
                        cost_scaling_factor=params["cost_scaling_factor"])
    else:
        costs = np.asarray(costs, dtype=np.uint8)
        if costs.shape != occ.shape:
            raise ValueError("costs shape must match occupancy")
    start = _pose(start, "start")
    goal = _pose(goal, "goal")
    grid = _grid(costs)
    planner = PathPlannerCore(_planner_config(params))
    original = _install_heuristic(planner.planner, params["heuristic"], grid, goal, params)
    try:
        found = planner.make_plan(start, goal, grid)
    finally:
        _restore_heuristic(planner.planner, original)
    report = _summarize(found, costs)
    report.update(start=list(start), goal=list(goal), heuristic=params["heuristic"],
                  planner_name=params["planner_name"],
                  costs=costs.astype(int).reshape(-1).tolist())
    if params["with_runtime"]:
        snapshot = _runtime_snapshot(params)
        state = VesselState(x=start[0], y=start[1], psi=start[2], stamp=0.0, frame_id=grid.frame_id)
        request = PlanningRequest(1, goal, state, grid, np.empty((0, 3)), snapshot)
        response = plan_navigation(request)
        trajectory = np.asarray(response.trajectory.points, dtype=float) if len(response.trajectory.points) else np.empty((0, 8))
        report["runtime"] = {
            "status": response.status.value, "message": response.message or "",
            "elapsed": float(response.elapsed), "expanded": int(response.expanded),
            "route_points": int(len(response.route)), "trajectory_points": int(len(trajectory)),
            "trajectory": trajectory.tolist() if len(trajectory) else [],
        }
    return report


def sweep(occupancy, start, goal, params, name, values):
    if name not in default_params():
        raise ValueError(f"cannot sweep unknown parameter {name}")
    if not isinstance(values, (list, tuple)) or not 2 <= len(values) <= 16:
        raise ValueError("sweep needs 2 to 16 values")
    points = []
    for value in values:
        candidate = dict(params or {})
        candidate[name] = value
        result = plan(occupancy, start, goal, candidate)
        points.append({
            "value": value, "status": result["status"], "length": result["length"],
            "elapsed": result["elapsed"], "expanded": result["expanded"],
            "message": result["message"], "turn_switches": result["turn_switches"],
            "min_clearance": result["min_clearance"], "path": result["path"],
        })
    return {"param": name, "points": points}


def heading_experiment(occupancy, start, goal, params, samples=12):
    samples = int(samples)
    if not 4 <= samples <= 36:
        raise ValueError("heading samples must be between 4 and 36")
    base = dict(params or {})
    base["with_runtime"] = False
    fixed = plan(occupancy, start, goal, base)
    x, y, _ = _pose(goal, "goal")
    variants = []
    best = None
    for index in range(samples):
        yaw = -math.pi + 2 * math.pi * index / samples
        result = plan(occupancy, start, (x, y, yaw), base)
        variants.append({"yaw": yaw, **{k: result[k] for k in
                         ("status", "length", "elapsed", "expanded", "message", "path",
                          "turn_switches", "min_clearance")}})
        if result["status"] == "success" and (best is None or result["length"] < best["length"]):
            best = variants[-1]
    return {"samples": samples, "fixed": fixed, "best": best, "variants": variants}


def heuristic_experiment(occupancy, start, goal, params):
    base = dict(params or {})
    base["with_runtime"] = False
    reports = {}
    for mode in HEURISTIC_MODES:
        candidate = dict(base)
        candidate["heuristic"] = mode
        reports[mode] = plan(occupancy, start, goal, candidate)
    return reports


def cost_grid_experiment(occupancy, start, goal, params,
                         factors=(0.05, 0.25, 0.5, 1.0), penalties=(0.0, 0.5, 1.0, 2.0)):
    factors, penalties = list(factors), list(penalties)
    if not (1 <= len(factors) <= 6 and 1 <= len(penalties) <= 6):
        raise ValueError("cost grid is limited to 6×6")
    base = dict(params or {})
    base["with_runtime"] = False
    cells = []
    for factor in factors:
        row = []
        for penalty in penalties:
            candidate = dict(base)
            candidate["obstacle_factor"] = factor
            candidate["cost_penalty"] = penalty
            result = plan(occupancy, start, goal, candidate)
            row.append({
                "obstacle_factor": factor, "cost_penalty": penalty,
                "status": result["status"], "length": result["length"],
                "elapsed": result["elapsed"], "expanded": result["expanded"],
                "min_clearance": result["min_clearance"], "path": result["path"],
                "message": result["message"],
            })
        cells.append(row)
    return {"factors": factors, "penalties": penalties, "cells": cells}


def describe_world():
    catalog = presets()
    return {
        "map": dict(MAP),
        "world": {"width": WORLD_WIDTH, "height": WORLD_HEIGHT},
        "defaults": default_params(),
        "presets": {key: {"name": item["name"], "note": item["note"],
                          "start": list(item["start"]), "goal": list(item["goal"]),
                          "occupancy": item["occupancy"].reshape(-1).astype(int).tolist()}
                    for key, item in catalog.items()},
        "heuristics": list(HEURISTIC_MODES),
    }


