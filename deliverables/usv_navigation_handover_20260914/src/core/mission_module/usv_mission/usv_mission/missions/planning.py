"""Mission planning inside the existing bounded, single-process worker."""
from dataclasses import replace
import math
import time
import numpy as np
from common.collision import footprint_path_is_clear
from common import dynamics
from common.math_helper import wrap_angle
from common.models import AlgorithmStatus, USVModel
from path_planner.path_planner_core import PathPlannerCore, path_is_clear_with_inflation_escape
from trajectory_planner.trajectory import Trajectory
from trajectory_planner.trajectory_planner_factory import create_trajectory_planner
from ..configuration import C, T, section
from ..planning_types import PlanningResponse
from .. import navigation
from .geometry import pose_is_clear, turning_envelope_is_clear


def mission_parameters(parameters, spec):
    """Independent speed and completion disks; normal navigation is untouched."""
    cfg = dict(parameters)
    cfg['dynamics.cruising_speed'] = spec.speed
    cfg[T+'los.goal_tolerance'] = min(cfg[T+'los.goal_tolerance'], spec.position_tolerance)
    for name in ('pid', 'lqr'):
        cfg[C+name+'_controller.goal_distance_tolerance'] = spec.position_tolerance
        cfg[C+name+'_controller.goal_speed_tolerance'] = .1
    return cfg


def _route_cost(route, state, goal, turning_radius):
    xy = np.vstack(([state.x, state.y], route[:, :2]))
    deltas = np.diff(xy, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    headings = np.arctan2(deltas[lengths > 1e-8, 1], deltas[lengths > 1e-8, 0])
    angles = np.r_[state.psi, headings, goal[2]]
    turn = sum(abs(wrap_angle(b-a)) for a, b in zip(angles[:-1], angles[1:]))
    return float(lengths.sum()+turning_radius*turn)


def _route_after_backoff(request, state, goal, cfg, deadline):
    """Rebuild from the new departure position, preserving the original goal.

    An old route prefix can point straight back into the obstacle. A circle
    escape keeps its tangent approach, but its connection is planned anew.
    """
    if time.perf_counter() >= deadline:
        return None
    tangent = request.mission_intent.circle is not None
    if tangent:
        radius = request.mission_intent.spec.speed/cfg[T+'los.yaw_rate_max']
        distance = max(2., 2.*radius)
        approach = (goal[0]-distance*math.cos(goal[2]), goal[1]-distance*math.sin(goal[2]), goal[2])
        if not pose_is_clear(request.grid, approach, cfg):
            return None
    else:
        approach = goal

    def center_clear(route):
        return path_is_clear_with_inflation_escape(
            request.grid, route, 254.*cfg['obstacle_factor'], True,
            outline_map=cfg['path_planner.outline_map'])

    route = np.array([state.pose, approach], dtype=float)
    if not center_clear(route):
        planner_cfg = section(cfg, 'path_planner.')
        planner_cfg.update(planner_name=cfg['planner_name'], obstacle_factor=cfg['obstacle_factor'],
                           timeout=max(1e-6, deadline-time.perf_counter()),
                           max_expansions=cfg['navigation.max_expansions'])
        found = PathPlannerCore(planner_cfg).make_plan(state.pose, approach, request.grid)
        if not found.success:
            return None
        route = np.vstack((state.pose, found.path, approach))
    if tangent:
        route = np.vstack((route, goal))
    if (not center_clear(route) or not footprint_path_is_clear(
            request.grid, route, cfg['navigation.footprint_length'], cfg['navigation.footprint_width'])):
        return None
    return route


def _departure_alignment(request, goal, route, cfg, deadline, navigate):
    """Offer conditional brake/back/align departure after all geometry checks.

    The returned trajectory assumes the *predicted* stop and a zero-velocity
    alignment. It is evidence of a feasible continuation, never an executable
    handover from the current moving state. The adapter must brake, freeze the
    measured stop position, optionally back away, align, then request a new
    plan to the selected goal. Backoff is a geometric corridor, not an open-loop
    reverse command: each measured control tick needs execution/stop screening.
    """
    if not len(route) or time.perf_counter() >= deadline:
        return None, {}
    model = USVModel.from_mapping(section(cfg, 'dynamics.'))
    length, width = cfg['navigation.footprint_length'], cfg['navigation.footprint_width']
    margin = cfg.get('mission.safety_margin', .1)
    stopping = None
    for command in dynamics.braking_candidates(request.state, model):
        if time.perf_counter() >= deadline:
            return None, {}
        poses, stopped = dynamics.predict_control_path(request.state, command, model, .1)
        if stopped and footprint_path_is_clear(request.grid, poses, length+2*margin, width+2*margin):
            stopping = poses
            break
    if stopping is None:
        return None, {}
    stop = replace(request.state, x=float(stopping[-1, 0]), y=float(stopping[-1, 1]),
                   psi=float(stopping[-1, 2]), u=0., v=0., r=0.)

    # Keep the original position as the first choice; use the shortest safe
    # reverse corridor only when that position has no feasible continuation.
    for backoff in (0., .5, 1., 2., 3.):
        if time.perf_counter() >= deadline:
            break
        origin = replace(stop, x=stop.x-backoff*math.cos(stop.psi),
                         y=stop.y-backoff*math.sin(stop.psi))
        reference = route
        if backoff:
            if not footprint_path_is_clear(request.grid, [stop.pose, origin.pose],
                                           length+2*margin, width+2*margin):
                continue
            reference = _route_after_backoff(request, origin, goal, cfg, deadline)
            if reference is None:
                continue
        # Direction of the first two metres of the verified connection, then
        # a fixed 45-degree yaw set. All distances share the original deadline.
        previous = np.array(origin.pose[:2])
        remaining, ahead = 2., previous
        for point in np.asarray(reference)[:, :2]:
            delta = point-previous
            distance = float(np.linalg.norm(delta))
            if distance >= remaining and distance > 1e-9:
                ahead = previous+delta*(remaining/distance)
                break
            remaining -= distance
            previous, ahead = point, point
        preferred = math.atan2(ahead[1]-origin.y, ahead[0]-origin.x)
        angles = [preferred, *sorted((n*math.pi/4 for n in range(8)),
                                    key=lambda yaw: abs(wrap_angle(yaw-origin.psi)))]
        unique = []
        for yaw in angles:
            yaw = wrap_angle(yaw)
            if not any(abs(wrap_angle(yaw-old)) < 1e-6 for old in unique):
                unique.append(yaw)
        for yaw in unique:
            if time.perf_counter() >= deadline:
                break
            aligned = replace(origin, psi=yaw)
            if not footprint_path_is_clear(request.grid, [origin.pose, aligned.pose],
                                           length+2*margin, width+2*margin):
                continue
            trial_cfg = dict(cfg)
            trial_cfg['navigation.planning_timeout'] = max(1e-6, deadline-time.perf_counter())
            trial = navigate(replace(request, state=aligned, goal=goal, route=reference,
                                     parameters=trial_cfg, mission_intent=None))
            if trial.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
                continue
            connected = np.vstack((aligned.pose, trial.route))
            if not footprint_path_is_clear(request.grid, connected, length, width):
                continue
            metadata = {'departure_pose': aligned.pose, 'departure_requires_replan': True,
                        'predicted_braking_stop_pose': stop.pose,
                        'departure_braking_command': (command.left, command.right, command.lateral)}
            if backoff:
                metadata['backoff_distance'] = backoff
            return trial, metadata
    return None, {}


def _is_los_collision(response):
    return (response.status == AlgorithmStatus.NO_PATH
            and 'LOS swept footprint' in response.message)


def plan_mission(request):
    begun = time.perf_counter()
    intent = request.mission_intent
    cfg = mission_parameters(request.parameters, intent.spec)
    timeout = cfg['navigation.planning_timeout']
    deadline = begun+timeout
    metadata = {'map_stamp': request.grid.stamp, 'candidates_checked': 0,
                'candidate_cost': None, 'candidate_count': len(intent.candidates)}
    empty = np.empty((0, 3))

    def failure(status, reason):
        return PlanningResponse(request.version, status, empty, Trajectory(),
                                time.perf_counter()-begun, reason, metadata=metadata)

    try:
        intent.spec.validate(cfg)
        if request.state.frame_id != request.grid.frame_id or not request.state.is_finite():
            return failure(AlgorithmStatus.INVALID_INPUT, 'state/map frame mismatch or invalid state')
        if not pose_is_clear(request.grid, request.state.pose, cfg):
            return failure(AlgorithmStatus.NO_PATH, 'current hull intersects lethal or unknown map cells')
        if intent.kind == 'circle':
            model = USVModel.from_mapping(section(cfg, 'dynamics.'))
            guidance = create_trajectory_planner(cfg[T+'planner_name'], model, section(cfg, T+'los.'))
            generated = guidance.plan_circle(intent.circle, request.state)
            if generated.status != AlgorithmStatus.SUCCESS:
                return failure(generated.status, generated.message)
            if not footprint_path_is_clear(request.grid, generated.trajectory.points[:, [2, 3, 4]],
                                           cfg['navigation.footprint_length'], cfg['navigation.footprint_width']):
                return failure(AlgorithmStatus.NO_PATH, 'circle swept footprint intersects lethal or unknown map cells')
            return PlanningResponse(request.version, generated.status, empty, generated.trajectory,
                                    time.perf_counter()-begun, metadata=metadata)
        if intent.kind not in ('navigate', 'orbit_entry', 'escape'):
            return failure(AlgorithmStatus.INVALID_INPUT, 'intent does not request a trajectory')
        if intent.goal is not None:
            result = navigation.plan_navigation(replace(request, goal=intent.goal, parameters=cfg, mission_intent=None))
            if intent.kind == 'escape' and _is_los_collision(result):
                aligned, departure = _departure_alignment(request, intent.goal, result.route,
                                                         cfg, deadline, navigation.plan_navigation)
                if aligned is not None:
                    result = aligned
                    metadata.update(departure)
            return replace(result, selected_goal=intent.goal, metadata=metadata)
        candidates = intent.candidates
        if intent.kind == 'escape':
            candidates = candidates[:4]
        if not candidates:
            return failure(AlgorithmStatus.NO_PATH, 'no geometrically safe candidate')
        radius = intent.spec.speed/cfg[T+'los.yaw_rate_max']
        best, best_goal, best_cost, expanded, searched = None, None, math.inf, 0, False
        best_departure = {}
        for goal in candidates:
            if time.perf_counter() >= deadline:
                break
            # For circle entry candidates this is a true cost lower bound.
            # Exit candidates are ordered by heading+distance, so do not break.
            lower_bound = math.hypot(goal[0]-request.state.x, goal[1]-request.state.y)
            if intent.kind == 'orbit_entry' and lower_bound >= best_cost:
                break
            goal_clear = (turning_envelope_is_clear(request.grid, goal, cfg,
                                                  arrival_tolerance=intent.spec.position_tolerance)
                          if intent.kind == 'escape'
                          else pose_is_clear(request.grid, goal, cfg))
            if not goal_clear:
                continue
            metadata['candidates_checked'] += 1
            tangent = intent.circle is not None
            if tangent:
                distance = max(2., 2.*radius)
                approach = (goal[0]-distance*math.cos(goal[2]), goal[1]-distance*math.sin(goal[2]), goal[2])
                if not pose_is_clear(request.grid, approach, cfg):
                    continue
            else:
                approach = goal
            direct = np.array([request.state.pose, approach], dtype=float)
            threshold = 254.*cfg['obstacle_factor']
            def center_clear(route):
                return path_is_clear_with_inflation_escape(
                    request.grid, route, threshold, True,
                    outline_map=cfg['path_planner.outline_map'])
            if center_clear(direct):
                route = direct
            else:
                planner_cfg = section(cfg, 'path_planner.')
                planner_cfg.update(planner_name=cfg['planner_name'], obstacle_factor=cfg['obstacle_factor'],
                                   timeout=max(1e-6, deadline-time.perf_counter()),
                                   max_expansions=cfg['navigation.max_expansions'])
                found = PathPlannerCore(planner_cfg).make_plan(request.state.pose, approach, request.grid)
                searched = True
                expanded += found.expanded
                if not found.success:
                    continue
                route = np.vstack((found.path, approach))
            if tangent:
                route = np.vstack((route, goal))
            if not center_clear(route) or not footprint_path_is_clear(
                    request.grid, route, cfg['navigation.footprint_length'], cfg['navigation.footprint_width']):
                continue
            trial_cfg = dict(cfg)
            trial_cfg['navigation.planning_timeout'] = max(1e-6, deadline-time.perf_counter())
            trial = navigation.plan_navigation(replace(request, goal=goal, route=route,
                                           parameters=trial_cfg, mission_intent=None))
            expanded += trial.expanded
            searched |= trial.searched
            departure = {}
            if intent.kind == 'escape' and _is_los_collision(trial):
                aligned, departure = _departure_alignment(request, goal, route, cfg, deadline, navigation.plan_navigation)
                if aligned is not None:
                    trial = aligned
                    expanded += trial.expanded
                    searched |= trial.searched
            if trial.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
                continue
            cost = _route_cost(route, request.state, goal, radius)
            if cost < best_cost:
                best, best_goal, best_cost = trial, tuple(goal), cost
                best_departure = departure
        metadata['search_elapsed'] = time.perf_counter()-begun
        metadata['budget_exhausted'] = time.perf_counter() >= deadline
        if best is None:
            return failure(AlgorithmStatus.TIMEOUT if metadata['budget_exhausted'] else AlgorithmStatus.NO_PATH,
                           'no candidate passed global route and LOS swept hull validation')
        metadata['candidate_cost'] = best_cost
        metadata.update(best_departure)
        return replace(best, elapsed=time.perf_counter()-begun, selected_goal=best_goal,
                       metadata=metadata, expanded=expanded, searched=searched)
    except (ValueError, TypeError, ArithmeticError, AttributeError) as error:
        return failure(AlgorithmStatus.INVALID_INPUT, str(error))
