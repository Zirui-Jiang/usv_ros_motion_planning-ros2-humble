# 规划进程交接数据：请求携带状态、地图、参数与任务意图，响应携带参考轨迹及诊断。
"""Immutable planning handover types shared without dispatch imports."""
from dataclasses import dataclass, field
import numpy as np
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from trajectory_planner.trajectory import Trajectory


# 冻结字段绑定形成交接对象；内部数组和字典的隔离依赖调用方复制快照。
@dataclass(frozen=True)
class PlanningRequest:
    version: int
    goal: tuple
    state: VesselState
    grid: GridSnapshot
    route: np.ndarray
    parameters: dict
    mission_intent: object = None


# version 用于拒绝过期结果；elapsed 为秒，selected_goal 保存实际选中的任务候选点。
@dataclass(frozen=True)
class PlanningResponse:
    version: int
    status: AlgorithmStatus
    route: np.ndarray
    trajectory: Trajectory
    elapsed: float
    message: str = ''
    searched: bool = False
    expanded: int = 0
    selected_goal: tuple | None = None
    metadata: dict = field(default_factory=dict)


