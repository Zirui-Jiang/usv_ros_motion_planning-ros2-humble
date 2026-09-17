"""Explicit factory for the three implemented grid-search modes."""

from ..graph_planner.astar_planner import AStarPathPlanner


def create_planner(config: dict | None = None) -> AStarPathPlanner:
    return AStarPathPlanner(config)


class PathPlannerFactory:
    create_planner = staticmethod(create_planner)
