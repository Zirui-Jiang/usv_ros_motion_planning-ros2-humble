"""ROS-independent planning worker and navigation freshness rules."""
from dataclasses import dataclass
import math
import time
import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, USVModel, VesselState
from common.math_helper import path_is_clear
from path_planner.path_planner_core import PathPlannerCore
from trajectory_planner.trajectory import Trajectory
from trajectory_planner.trajectory_planner_factory import create_trajectory_planner
from .configuration import C, T, section


def fresh(stamp, now, timeout):
    """Future data is invalid too (clock rewind or mismatched clock domain)."""
    return all(math.isfinite(v) for v in (stamp, now, timeout)) and -1e-6 <= now - stamp <= timeout


def readiness(now, state, grid, trajectory, config):
    if state is None or not fresh(state.stamp, now, config['navigation.state_timeout']):
        return 'WAITING_STATE'
    if grid is None or not fresh(grid.stamp, now, config['navigation.map_timeout']):
        return 'WAITING_MAP'
    if state.frame_id != grid.frame_id:
        return 'FRAME_MISMATCH'
    if trajectory is None or trajectory.empty() or not fresh(
            trajectory.stamp, now, config['navigation.trajectory_timeout']):
        return 'WAITING_REFERENCE'
    if trajectory.frame_id != state.frame_id:
        return 'FRAME_MISMATCH'
    return 'NAVIGATING'


@dataclass(frozen=True)
class PlanningRequest:
    version: int
    goal: tuple
    state: VesselState
    grid: GridSnapshot
    route: np.ndarray
    parameters: dict


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


def plan_navigation(request):
    """Runs in one spawned process; consumes only immutable/copy snapshots."""
    begun = time.perf_counter()
    cfg = request.parameters
    route = np.empty((0, 3))
    searched, expanded = False, 0

    def result(status, trajectory=None, message=''):
        return PlanningResponse(request.version, status, route,
                                trajectory if trajectory is not None else Trajectory(),
                                time.perf_counter() - begun, message, searched, expanded)

    try:
        if request.state.frame_id != request.grid.frame_id:
            return result(AlgorithmStatus.INVALID_INPUT, message='state/map frame mismatch')
        planner_cfg = section(cfg, 'path_planner.')
        planner_cfg.update(planner_name=cfg['planner_name'], obstacle_factor=cfg['obstacle_factor'],
                           timeout=cfg['navigation.planning_timeout'],
                           max_expansions=cfg['navigation.max_expansions'])
        planner = PathPlannerCore(planner_cfg)
        route = planner.prune(request.route, request.state)
        threshold = 254.0 * cfg['obstacle_factor']
        # Check the vessel-to-route connection as well as all remaining segments.
        connected = np.vstack((request.state.pose, route)) if len(route) else route
        if not len(route) or not path_is_clear(request.grid, connected, threshold):
            searched = True
            found = planner.make_plan(request.state.pose, request.goal, request.grid)
            expanded = found.expanded
            route = found.path
            if not found.success:
                return result(found.status, message=found.message)
        # Grid search returns cell centres. LOS must converge to the requested
        # position rather than stop in a neighbouring part of the goal cell.
        route = route.copy()
        route[-1] = request.goal
        model = USVModel.from_mapping(section(cfg, 'dynamics.'))
        los_config = section(cfg, T + 'los.')
        controller_tolerance = cfg[C + cfg[C + 'controller_name'] + '_controller.goal_distance_tolerance']
        los_config['goal_tolerance'] = min(los_config['goal_tolerance'], controller_tolerance)
        guidance = create_trajectory_planner(cfg[T + 'planner_name'], model, los_config)
        generated = guidance.plan(route, request.state)
        if generated.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            return result(generated.status, message=generated.message)
        trajectory = generated.trajectory
        # The exact requested goal, not a grid-cell centre, defines completion.
        trajectory.goal = tuple(request.goal[:2])
        # LOS can cut a sharp corner. Never hand a colliding rollout to control.
        if not path_is_clear(request.grid, trajectory.points[:, [2, 3, 4]], threshold):
            return result(AlgorithmStatus.NO_PATH, message='LOS rollout intersects costmap obstacle')
        return result(generated.status, trajectory)
    except (ValueError, TypeError, ArithmeticError) as error:
        return result(AlgorithmStatus.INVALID_INPUT, message=str(error))


class LatestPlanWorker:
    """One executing job and one replaceable pending snapshot, no unbounded queue."""
    def __init__(self, executor=None):
        if executor is None:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor
            executor = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn'))
        self.executor = executor
        self.future = None
        self.pending = None
        self.replaced = 0

    def offer(self, request):
        if self.pending is not None:
            self.replaced += 1
        self.pending = request
        self._start()

    def _start(self):
        if self.future is None and self.pending is not None:
            request, self.pending = self.pending, None
            self.future = self.executor.submit(plan_navigation, request)

    def poll(self):
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        try:
            return future.result()
        finally:
            self._start()

    def invalidate(self):
        self.pending = None
        if self.future is not None and self.future.cancel():
            self.future = None

    def close(self):
        self.invalidate()
        self.executor.shutdown(wait=True, cancel_futures=True)
