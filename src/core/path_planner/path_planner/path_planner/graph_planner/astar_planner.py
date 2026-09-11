"""Bounded eight-neighbour A*, Dijkstra and greedy best-first search."""

from __future__ import annotations

import heapq
import itertools
import math
import time
import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, PlanResult


class AStarPathPlanner:
    """Pure grid search. Costs use Nav2's uint8 scale (lethal=254, unknown=255).

    ``obstacle_factor`` preserves the old threshold ``254 * factor``.
    A soft inflated start may escape through strictly descending costs, as
    in C++; after reaching free space no above-threshold cell is reachable.
    Lethal cells, blocked goals and outlined boundaries cannot be escaped.
    Unknown cells are blocked unless explicitly allowed. Diagonal corner
    cutting defaults to the original behaviour and can be disabled.
    """

    MOTIONS = ((0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
               (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
               (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))

    def __init__(self, config: dict | None = None):
        self.config = dict(config or {})
        self.name = self.config.get("planner_name", "astar")
        if self.name not in ("astar", "dijkstra", "gbfs"):
            raise ValueError(f"unknown path planner: {self.name}")
        self.factor = float(self.config.get("obstacle_factor", 0.5))
        self.timeout = float(self.config.get("timeout", 1.0))
        self.max_expansions = self.config.get("max_expansions", 200000)
        if not np.isfinite(self.factor) or not 0 < self.factor <= 1:
            raise ValueError("obstacle_factor must be in (0, 1]")
        if not np.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        if isinstance(self.max_expansions, bool) or not isinstance(self.max_expansions, (int, np.integer)) or self.max_expansions < 1:
            raise ValueError("max_expansions must be a positive integer")
        self.unknown_is_obstacle = bool(self.config.get("unknown_is_obstacle", True))
        self.allow_corner_cutting = bool(self.config.get("allow_corner_cutting", True))
        self.outline_map = bool(self.config.get("outline_map", False))

    def plan(self, start, goal, grid: GridSnapshot) -> PlanResult:
        begun = time.perf_counter()
        expanded = 0

        def result(status, message="", path=None):
            return PlanResult(status=status,
                              path=np.empty((0, 3)) if path is None else path,
                              message=message, elapsed=time.perf_counter() - begun,
                              expanded=expanded)

        try:
            first, last = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
            if first.shape != (3,) or last.shape != (3,) or not np.isfinite(first).all() or not np.isfinite(last).all():
                return result(AlgorithmStatus.INVALID_INPUT, "start and goal must be finite (x, y, yaw)")
            source = grid.world_to_map(float(first[0]), float(first[1]))
            target = grid.world_to_map(float(last[0]), float(last[1]))
            if source is None or target is None:
                return result(AlgorithmStatus.INVALID_INPUT, "start or goal is outside the grid")
            width, height = grid.width, grid.height
            costs = np.asarray(grid.data)
            if costs.shape != (height, width) or width < 1 or height < 1:
                return result(AlgorithmStatus.INVALID_INPUT, "invalid grid shape")
            hard_blocked = costs == 254
            if self.unknown_is_obstacle:
                hard_blocked = hard_blocked | (costs == 255)
            blocked = costs >= (254.0 * self.factor)
            if not self.unknown_is_obstacle:
                blocked[costs == 255] = False
            if self.outline_map:
                blocked[[0, -1], :] = True
                blocked[:, [0, -1]] = True
                hard_blocked[[0, -1], :] = True
                hard_blocked[:, [0, -1]] = True
        except (ValueError, TypeError, AttributeError, OverflowError) as error:
            return result(AlgorithmStatus.INVALID_INPUT, str(error))
        if hard_blocked[source[1], source[0]] or blocked[target[1], target[0]]:
            return result(AlgorithmStatus.NO_PATH, "start or goal is blocked")
        if source == target:
            x, y = grid.map_to_world(*source)
            return result(AlgorithmStatus.SUCCESS, path=np.array([[x, y, last[2]]]))
        counter = itertools.count()
        queue = [(0.0, 0.0, next(counter), source, 0.0)]
        best = {source: 0.0}
        parents = {}
        closed = set()
        while queue:
            if time.perf_counter() - begun >= self.timeout:
                return result(AlgorithmStatus.TIMEOUT, "path planning time budget exceeded")
            if expanded >= self.max_expansions:
                return result(AlgorithmStatus.TIMEOUT, "path planning expansion budget exceeded")
            _, _, _, current, current_g = heapq.heappop(queue)
            if current in closed or current_g > best.get(current, math.inf):
                continue
            closed.add(current)
            expanded += 1
            if current == target:
                nodes = [current]
                while nodes[-1] != source:
                    nodes.append(parents[nodes[-1]])
                nodes.reverse()
                path = np.zeros((len(nodes), 3))
                path[:, :2] = [grid.map_to_world(*node) for node in nodes]
                path[0, 2], path[-1, 2] = first[2], last[2]
                return result(AlgorithmStatus.SUCCESS, path=path)
            x, y = current
            for dx, dy, distance in self.MOTIONS:
                nx, ny = x + dx, y + dy
                # Check both coordinates before indexing: no cross-row aliases.
                if not (0 <= nx < width and 0 <= ny < height) or hard_blocked[ny, nx]:
                    continue
                # Preserve the C++ inflation escape condition. The current
                # cell must also be blocked: an explicitly permitted unknown
                # cell has raw cost 255 but is free under that policy, and
                # must not become a new source of soft-obstacle exemptions.
                if blocked[ny, nx] and (not blocked[y, x] or costs[ny, nx] >= costs[y, x]):
                    continue
                if dx and dy and not self.allow_corner_cutting and (blocked[y, nx] or blocked[ny, x]):
                    continue
                neighbour = (nx, ny)
                if neighbour in closed:
                    continue
                cost = current_g + distance
                if cost >= best.get(neighbour, math.inf):
                    continue
                best[neighbour], parents[neighbour] = cost, current
                heuristic = math.hypot(nx - target[0], ny - target[1]) if self.name != "dijkstra" else 0.0
                priority = heuristic if self.name == "gbfs" else cost + heuristic
                heapq.heappush(queue, (priority, heuristic, next(counter), neighbour, cost))
        return result(AlgorithmStatus.NO_PATH, "no traversable path")
