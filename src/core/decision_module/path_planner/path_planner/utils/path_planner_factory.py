# 从同一配置入口选择栅格搜索或前进限定 Hybrid A*；未知名称仍在构造时拒绝。
"""Explicit factory for grid search and forward Hybrid A*."""

from ..graph_planner.astar_planner import AStarPathPlanner


def create_planner(config: dict | None = None):
    if (config or {}).get("planner_name", "astar") == "hybrid_astar":
        from ..graph_planner.hybrid_astar_planner import HybridAStarPathPlanner
        return HybridAStarPathPlanner(config)
    return AStarPathPlanner(config)


class PathPlannerFactory:
    create_planner = staticmethod(create_planner)
