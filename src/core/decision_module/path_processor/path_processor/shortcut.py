"""Greedy line-of-sight shortcuts with caller-supplied collision policy."""

import math
from numbers import Real
import time

from .path_processor import PathProcessor, as_path


def shortcut_path(path, is_path_clear, *, deadline=None):
    """Return an owned path, keeping endpoints and the farthest visible nodes.

    ``is_path_clear`` validates a complete (N, 3) path on one fixed map snapshot.
    It must enforce the caller's obstacle/unknown/corner policies. The processor
    neither imports a planner nor changes the map. ``deadline`` is an absolute
    ``time.perf_counter()`` deadline, shared with search, not a fresh time budget.
    Expiration raises TimeoutError; invalid input or an unsafe original raises
    ValueError. The callback should check the same deadline during long checks.

    Use this on one grid-planned leg at a time, not across mandatory waypoints
    or on kinematically constrained Hybrid A* curves. This is not a hull check.
    """
    if not callable(is_path_clear):
        raise ValueError('is_path_clear must be callable')
    if deadline is not None and (isinstance(deadline, bool)
            or not isinstance(deadline, Real) or not math.isfinite(deadline)):
        raise ValueError('deadline must be a finite absolute time or None')

    def check_time():
        if deadline is not None and time.perf_counter() >= deadline:
            raise TimeoutError('path shortcut exceeded planning budget')

    def valid(candidate):
        check_time()
        accepted = is_path_clear(candidate)
        check_time()  # 碰撞检查本身的耗时也计入同一预算。
        return accepted

    check_time()
    points = as_path(path)
    check_time()
    if not len(points):
        return points
    if not valid(points):
        raise ValueError('original path failed collision validation')
    if len(points) < 3:
        return points

    kept, index = [0], 0
    while index < len(points)-1:
        check_time()
        following = len(points)-1
        while following > index+1 and not valid(points[[index, following]]):
            following -= 1
        # 无可用捷径时保留原相邻边。软起点逃逸的中间点可能仍在膨胀区，
        # 单独检查该边会因“终点被阻塞”失败，但整条原路径已通过下降逃逸校验。
        kept.append(following)
        index = following
    candidate = points[kept]
    # 最后按完整路径再验一次；不接受局部检查遗漏的跨段约束。
    return candidate if valid(candidate) else points


class ShortcutPathProcessor(PathProcessor):
    """PathProcessor adapter; each call receives its own absolute deadline."""

    def __init__(self, is_path_clear):
        super().__init__()
        if not callable(is_path_clear):
            raise ValueError('is_path_clear must be callable')
        self.is_path_clear = is_path_clear

    def process(self, path, *, deadline=None):
        return shortcut_path(path, self.is_path_clear, deadline=deadline)
