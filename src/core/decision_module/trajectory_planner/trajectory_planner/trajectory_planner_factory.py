# 创建当前唯一支持的 LOS 轨迹规划器，拒绝未知名称以避免静默回退。
"""Explicit trajectory algorithm selection; unsupported names fail early."""

from .los_trajectory_planner import LosTrajectoryPlanner


def create_trajectory_planner(name, model, config=None):
    if str(name).lower() != "los":
        raise ValueError(f"unsupported trajectory planner: {name!r}; available: los")
    return LosTrajectoryPlanner(model, config)
