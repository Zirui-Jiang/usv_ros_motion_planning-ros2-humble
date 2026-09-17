"""Immutable planning handover types shared without dispatch imports."""
from dataclasses import dataclass, field
import numpy as np
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from trajectory_planner.trajectory import Trajectory


@dataclass(frozen=True)
class PlanningRequest:
    version: int
    goal: tuple
    state: VesselState
    grid: GridSnapshot
    route: np.ndarray
    parameters: dict
    mission_intent: object = None


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


