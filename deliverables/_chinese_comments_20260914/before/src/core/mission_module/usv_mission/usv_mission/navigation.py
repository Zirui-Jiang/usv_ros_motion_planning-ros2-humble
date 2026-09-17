"""Ordinary goal navigation using the public planning/controller packages."""
import math
import time
import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, USVModel, VesselState
from common.collision import footprint_path_is_clear
from path_planner.path_planner_core import PathPlannerCore, path_is_clear_with_inflation_escape
from trajectory_planner.trajectory import Trajectory
from trajectory_planner.trajectory_planner_factory import create_trajectory_planner
from .configuration import C, T, section


from .planning_types import PlanningResponse


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
        # Check the vessel-to-route connection and remaining segments on the
        # latest map with the same policy that accepted the global route.
        # A legal soft descent must not become a false invalidation merely
        # because the vessel is still in its original inflated start region.
        connected = np.vstack((request.state.pose, route)) if len(route) else route
        if not len(route) or not path_is_clear_with_inflation_escape(
                request.grid, connected, threshold, planner.planner.unknown_is_obstacle,
                outline_map=planner.planner.outline_map,
                allow_corner_cutting=planner.planner.allow_corner_cutting):
            searched = True
            found = planner.make_plan(request.state.pose, request.goal, request.grid)
            expanded = found.expanded
            route = found.path
            if not found.success:
                return result(found.status, message=found.message)
        model = USVModel.from_mapping(section(cfg, 'dynamics.'))
        los_config = section(cfg, T + 'los.')
        controller_tolerance = cfg[C + cfg[C + 'controller_name'] + '_controller.goal_distance_tolerance']
        endpoint_offset = float(np.linalg.norm(route[-1, :2] - request.goal[:2]))
        if endpoint_offset < controller_tolerance:
            # Preserve the grid endpoint and its final approach direction.
            # By the triangle inequality, this LOS stop disk lies entirely
            # within the controller's disk around the actual requested goal.
            los_config['goal_tolerance'] = min(
                los_config['goal_tolerance'], controller_tolerance - endpoint_offset)
        else:
            # A centre outside/on the actual-goal disk cannot support a
            # positive LOS stop radius. Keep it and validate a new connection.
            connection = np.vstack((route[-1], request.goal))
            if not path_is_clear_with_inflation_escape(
                    request.grid, connection, threshold, planner.planner.unknown_is_obstacle,
                    outline_map=planner.planner.outline_map,
                    allow_corner_cutting=planner.planner.allow_corner_cutting):
                return result(AlgorithmStatus.NO_PATH, message='goal connection failed grid collision validation')
            route = np.vstack((route, request.goal))
            los_config['goal_tolerance'] = min(los_config['goal_tolerance'], controller_tolerance)
        guidance = create_trajectory_planner(cfg[T + 'planner_name'], model, los_config)
        generated = guidance.plan(route, request.state)
        if generated.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            return result(generated.status, message=generated.message)
        trajectory = generated.trajectory
        # The exact requested goal, not a grid-cell centre, defines completion.
        trajectory.goal = tuple(request.goal[:2])
        # A* retains its original soft-cost threshold. Local collision safety
        # checks the complete hull against lethal/unknown cell areas instead:
        # rejecting the centre at every soft inflated cost can deadlock a
        # physically clear turn, while centre-only checks miss bow collisions.
        if not footprint_path_is_clear(request.grid, trajectory.points[:, [2, 3, 4]],
                                       cfg['navigation.footprint_length'], cfg['navigation.footprint_width']):
            return result(AlgorithmStatus.NO_PATH, message='LOS swept footprint intersects lethal or unknown map cells')
        return result(generated.status, trajectory)
    except (ValueError, TypeError, ArithmeticError) as error:
        return result(AlgorithmStatus.INVALID_INPUT, message=str(error))


