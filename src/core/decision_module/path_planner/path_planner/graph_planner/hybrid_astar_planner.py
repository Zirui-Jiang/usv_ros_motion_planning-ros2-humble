# 前进限定 Hybrid A*：连续位姿圆弧扩展，位置/艏向分箱判重，Dubins 曲线连接精确目标。
"""Bounded forward Hybrid A* in world metres, with rectangular hull checks.

This is a geometric global planner, not a hydrodynamic or COLREGs planner.
It retains continuous poses, checks every primitive and returns their samples.
The first feasible analytic completion terminates the search: no continuous
global-optimality or resolution-completeness guarantee is made.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time

import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, PlanResult
from .dubins import DubinsPath, advance_pose, dubins_paths, sample_dubins
from .hybrid_collision import HybridCollisionChecker


@dataclass(frozen=True)
class _SearchNode:
    pose: np.ndarray
    cost: float
    parent: _SearchNode | None = None
    curvature: float = 0.
    distance: float = 0.


class HybridAStarPathPlanner:
    """Same ``plan(start, goal, grid) -> PlanResult`` boundary as grid A*.

    Unlike grid A*, endpoints are not snapped to cell centres; a blocked soft
    start is rejected rather than allowed to escape through decreasing costs.
    ``minimum_turning_radius=4`` is a simulation starting value, not measured
    vessel capability. Hull dimensions refer to a rectangle centred on pose.
    """

    name = "hybrid_astar"
    preserves_kinematics = True

    def __init__(self, config: dict | None = None):
        self.config = dict(config or {})
        if self.config.get("planner_name", self.name) != self.name:
            raise ValueError("HybridAStarPathPlanner requires planner_name=hybrid_astar")
        self.factor = self._number("obstacle_factor", .5)
        if not 0 < self.factor <= 1:
            raise ValueError("obstacle_factor must be in (0,1]")
        self.timeout = self._number("timeout", 1., positive=True)
        self.max_expansions = self._integer("max_expansions", 200000, 1)
        self.angle_bins = self._integer("angle_bins", 72, 8)
        self.heuristic_weight = self._number("heuristic_weight", 1.2)
        if self.heuristic_weight < 1.:
            raise ValueError("heuristic_weight must be at least one")
        self.minimum_turning_radius = self._number("minimum_turning_radius", 4., positive=True)
        self.turning_radius_multipliers = self._multipliers("turning_radius_multipliers", (1.,))
        # 默认只有最小半径左右转加直行，与原先三种运动基元一致；乘数>1 增加更缓的圆弧。
        self.curvatures = (0.,) + tuple(
            sign / (self.minimum_turning_radius * multiplier)
            for multiplier in self.turning_radius_multipliers
            for sign in (1., -1.))
        self.motion_step = self._number("motion_step", 1., positive=True)
        self.collision_step = self._number("collision_step", .2, positive=True)
        self.analytic_expansion_max_length = self._number("analytic_expansion_max_length", 20., positive=True)
        self.non_straight_penalty = self._number("non_straight_penalty", 1.05)
        if self.non_straight_penalty < 1:
            raise ValueError("non_straight_penalty must be at least one")
        self.change_penalty = self._number("change_penalty", .2)
        self.cost_penalty = self._number("cost_penalty", 0.)
        self.footprint_padding = self._number("footprint_padding", 0.)
        self.footprint_length = self._number("footprint_length", 2.2, positive=True)
        self.footprint_width = self._number("footprint_width", 1.2, positive=True)
        if min(self.change_penalty, self.cost_penalty, self.footprint_padding) < 0:
            raise ValueError("change_penalty, cost_penalty and footprint_padding must be nonnegative")
        self.unknown_is_obstacle = bool(self.config.get("unknown_is_obstacle", True))
        self.outline_map = bool(self.config.get("outline_map", False))
        # 实体船体/运动段检查不能通过开启点机器人切角绕过；保留属性供既有门面读取。
        self.allow_corner_cutting = False

    def _number(self, key, default, *, positive=False):
        value = self.config.get(key, default)
        try:
            if isinstance(value, (bool, np.bool_)):
                raise ValueError
            value = float(value)
        except (ValueError, TypeError, OverflowError) as error:
            raise ValueError(f"{key} must be a finite number") from error
        if not math.isfinite(value) or (positive and value <= 0):
            raise ValueError(f"{key} must be finite" + (" and positive" if positive else ""))
        return value

    def _integer(self, key, default, minimum):
        value = self.config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < minimum:
            raise ValueError(f"{key} must be an integer >= {minimum}")
        return int(value)

    # 半径乘数必须 >=1；重复项合并后按从小到大生成更缓的左右转基元。
    def _multipliers(self, key, default):
        value = self.config.get(key, default)
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool):
            value = (float(value),)
        if not isinstance(value, (list, tuple)) or not value:
            raise ValueError(f"{key} must be a nonempty list of finite numbers >= 1")
        unique = []
        seen = set()
        for item in value:
            if isinstance(item, (bool, np.bool_)) or not isinstance(item, (int, float, np.integer, np.floating)):
                raise ValueError(f"{key} must be a nonempty list of finite numbers >= 1")
            number = float(item)
            if not math.isfinite(number) or number < 1.:
                raise ValueError(f"{key} must be a nonempty list of finite numbers >= 1")
            marker = round(number, 9)
            if marker not in seen:
                seen.add(marker)
                unique.append(number)
        return tuple(sorted(unique))

    def _key(self, pose, curvature, grid):
        cell = grid.world_to_map(float(pose[0]), float(pose[1]))
        if cell is None:
            return None
        angle = float(pose[2]) % (2*math.pi)
        heading = int(math.floor(angle*self.angle_bins/(2*math.pi) + .5)) % self.angle_bins
        # 上一次转向影响 change_penalty，必须参与判重才能区分后续代价不同的状态。
        turn = 0 if curvature == 0 else (1 if curvature > 0 else -1)
        return (*cell, heading, turn)

    def _heuristic(self, pose, goal):
        distance = math.hypot(goal[0]-pose[0], goal[1]-pose[1])
        angle = abs(math.remainder(float(goal[2]-pose[2]), 2*math.pi))
        return max(distance, self.minimum_turning_radius*angle)

    def _edge_cost(self, start, curvature, distance, previous_curvature, grid, spacing, deadline):
        if time.perf_counter() >= deadline:
            raise TimeoutError("cost evaluation exceeded planning time budget")
        cost = distance * (self.non_straight_penalty if curvature else 1.)
        if curvature != previous_curvature:
            cost += self.change_penalty
        if self.cost_penalty and distance > 0:
            ratio = distance/spacing
            if not math.isfinite(ratio) or ratio > 2_000_000:
                raise TimeoutError("cost evaluation exceeds two million samples")
            count = max(1, math.ceil(ratio))
            total = 0.
            for index in range(count):
                if time.perf_counter() >= deadline:
                    raise TimeoutError("cost evaluation exceeded planning time budget")
                pose = advance_pose(start, curvature, distance*(index+.5)/count)
                cell = grid.world_to_map(*pose[:2])
                raw = int(grid.data[cell[1], cell[0]]) if cell is not None else 254
                # 显式可通行未知单元的 255 是编码，不应当作高于致命障碍的代价。
                total += 0. if raw == 255 else raw/254.
            cost += self.cost_penalty*distance*total/count
        return cost

    def _try_connection(self, node, goal, grid, checker, spacing, deadline):
        if math.hypot(*(goal[:2]-node.pose[:2])) > self.analytic_expansion_max_length:
            return None
        candidates = []
        for connection in dubins_paths(node.pose, goal, self.minimum_turning_radius):
            if connection.length > self.analytic_expansion_max_length + 1e-9:
                continue
            pose, previous, cost = node.pose, node.curvature, 0.
            for mode, length in zip(connection.modes, connection.lengths):
                curvature = {"S": 0., "L": 1./connection.radius, "R": -1./connection.radius}[mode]
                if length > 1e-10:
                    cost += self._edge_cost(pose, curvature, length, previous, grid, spacing, deadline)
                    pose = advance_pose(pose, curvature, length)
                    previous = curvature
            candidates.append((cost, connection))
        # 按与搜索一致的代价排序解析候选；碰撞、时间预算也与普通基元共用。
        for _, connection in sorted(candidates, key=lambda candidate: candidate[0]):
            pose, clear = node.pose, True
            for mode, length in zip(connection.modes, connection.lengths):
                curvature = {"S": 0., "L": 1./connection.radius, "R": -1./connection.radius}[mode]
                if not checker.motion_is_clear(pose, curvature, length, deadline=deadline):
                    clear = False
                    break
                pose = advance_pose(pose, curvature, length)
            if clear:
                return connection
        return None

    def _reconstruct(self, node, connection, first, last, spacing, deadline):
        edges = []
        while node.parent is not None:
            edges.append(node)
            node = node.parent
        chunks = [first[None, :].copy()]
        count = 1
        for edge in reversed(edges):
            if time.perf_counter() >= deadline:
                raise TimeoutError("path reconstruction exceeded planning time budget")
            mode = "S" if edge.curvature == 0 else ("L" if edge.curvature > 0 else "R")
            # 回放半径必须对应搜索边的实际曲率；缓弧不能按最小半径重新采样。
            radius = self.minimum_turning_radius if edge.curvature == 0 else 1. / abs(edge.curvature)
            primitive = DubinsPath((mode,), (edge.distance,), radius)
            samples = sample_dubins(edge.parent.pose, primitive, spacing)[1:]
            count += len(samples)
            if count > 2_000_000:
                raise TimeoutError("path reconstruction exceeds two million points")
            chunks.append(samples)
        final_start = edges[0].pose if edges else first
        tail = sample_dubins(final_start, connection, spacing)[1:]
        if count + len(tail) > 2_000_000:
            raise TimeoutError("path reconstruction exceeds two million points")
        chunks.append(tail)
        path = np.vstack(chunks)
        # 仅在解析积分已到达目标后消除浮点舍入；不能以强行追加目标制造末端急转。
        if (np.linalg.norm(path[-1, :2]-last[:2]) > 1e-7
                or abs(math.remainder(float(path[-1, 2]-last[2]), 2*math.pi)) > 1e-7):
            raise ValueError("analytic connection did not reach the requested pose")
        path[0], path[-1] = first, last
        if time.perf_counter() >= deadline:
            raise TimeoutError("path reconstruction exceeded planning time budget")
        return path

    def plan(self, start, goal, grid: GridSnapshot) -> PlanResult:
        begun, expanded = time.perf_counter(), 0
        deadline = begun + self.timeout

        def result(status, message="", path=None):
            return PlanResult(status, np.empty((0, 3)) if path is None else path,
                              message, time.perf_counter()-begun, expanded)

        try:
            first, last = np.array(start, dtype=float), np.array(goal, dtype=float)
            if (first.shape != (3,) or last.shape != (3,)
                    or not np.isfinite(first).all() or not np.isfinite(last).all()
                    or not isinstance(grid, GridSnapshot)):
                return result(AlgorithmStatus.INVALID_INPUT, "start and goal must be finite (x, y, yaw); grid must be GridSnapshot")
            if grid.world_to_map(*first[:2]) is None or grid.world_to_map(*last[:2]) is None:
                return result(AlgorithmStatus.INVALID_INPUT, "start or goal is outside the grid")
            checker = HybridCollisionChecker(
                grid, factor=self.factor, unknown_is_obstacle=self.unknown_is_obstacle,
                outline_map=self.outline_map, footprint_length=self.footprint_length,
                footprint_width=self.footprint_width, footprint_padding=self.footprint_padding,
                collision_step=self.collision_step)
            if time.perf_counter() >= deadline:
                raise TimeoutError("map preparation exceeded planning time budget")
            if not checker.pose_is_clear(first) or not checker.pose_is_clear(last):
                return result(AlgorithmStatus.NO_PATH, "start or goal is blocked by centre cost, hull or map boundary")
            if (np.linalg.norm(first[:2]-last[:2]) <= 1e-10
                    and abs(math.remainder(float(first[2]-last[2]), 2*math.pi)) <= 1e-10):
                return result(AlgorithmStatus.SUCCESS, path=last[None, :].copy())
            # 过短基元会反复落回本格；至少走一个栅格对角线，碰撞步长独立细分。
            distance = max(self.motion_step, math.sqrt(2)*grid.resolution)
            spacing = min(self.collision_step, grid.resolution/4,
                          self.minimum_turning_radius*math.pi/8)
            curvatures = self.curvatures
            initial = _SearchNode(first, 0.)
            counter = itertools.count()
            # 适度加权优先向目标推进，避免长航段对大量近似等代价分支做昂贵的船体检查。
            # 权重=1恢复原搜索排序；>1用可能稍长的路线换取更少展开，不改变可行性检查。
            queue = [(self.heuristic_weight*self._heuristic(first, last), next(counter), initial)]
            best = {self._key(first, 0., grid): initial}
            while queue:
                if time.perf_counter() >= deadline:
                    raise TimeoutError("path planning time budget exceeded")
                if expanded >= self.max_expansions:
                    raise TimeoutError("path planning expansion budget exceeded")
                _, _, current = heapq.heappop(queue)
                if best.get(self._key(current.pose, current.curvature, grid)) is not current:
                    continue
                expanded += 1
                connection = self._try_connection(current, last, grid, checker, spacing, deadline)
                if connection is not None:
                    path = self._reconstruct(current, connection, first, last, spacing, deadline)
                    return result(AlgorithmStatus.SUCCESS, path=path)
                for curvature in curvatures:
                    pose = advance_pose(current.pose, curvature, distance)
                    key = self._key(pose, curvature, grid)
                    if key is None:
                        continue
                    if not checker.motion_is_clear(current.pose, curvature, distance, deadline=deadline):
                        continue
                    cost = current.cost + self._edge_cost(current.pose, curvature, distance,
                                                          current.curvature, grid, spacing, deadline)
                    previous = best.get(key)
                    if previous is not None and cost >= previous.cost-1e-10:
                        continue
                    child = _SearchNode(pose, cost, current, curvature, distance)
                    best[key] = child
                    heapq.heappush(queue, (cost+self.heuristic_weight*self._heuristic(pose, last), next(counter), child))
            return result(AlgorithmStatus.NO_PATH, "no traversable forward path at the configured search resolution")
        except TimeoutError as error:
            return result(AlgorithmStatus.TIMEOUT, str(error))
        except (ValueError, TypeError, AttributeError, ArithmeticError) as error:
            return result(AlgorithmStatus.INVALID_INPUT, str(error))
