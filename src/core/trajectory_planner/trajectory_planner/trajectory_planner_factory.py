"""Explicit trajectory algorithm selection; unsupported names fail early."""

from .los_trajectory_planner import LosTrajectoryPlanner


def create_trajectory_planner(name, model, config=None):
    if str(name).lower() != "los":
        raise ValueError(f"unsupported trajectory planner: {name!r}; available: los")
    return LosTrajectoryPlanner(model, config)
