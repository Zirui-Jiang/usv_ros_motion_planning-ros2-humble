"""Actual PID + project planar Otter model, not just predicted anchor visits.

This deterministic still-water replay does not substitute for Gazebo/sea tests.
Open-water A* simplification produces the same straight legs used below.
"""
import math

import numpy as np
import pytest

from common.dynamics import step_dynamics
from common.models import AlgorithmStatus, USVModel, VesselState
from controller.pid_controller import PIDController
from trajectory_planner.route_trajectory import RouteTrajectoryPlanner
from usv_navigation.configuration import resolve_parameters, section, C, T


@pytest.mark.parametrize('horizon', [10., 20.])
@pytest.mark.parametrize('points', [
    ((12., 10., 0., 1.), (24., 10., 0., 1.), (36., 10., 0., 1.)),
    ((15., 10., 0., 2.), (15., 25., 0., 2.), (30., 25., 0., 1.)),
    ((15., 10., 0., 2*1852/3600), (40., 10., 0., 4*1852/3600),
     (60., 10., 0., 2*1852/3600)),
    ((15., 10., 0., 1.), (5., 10., 0., 1.)),
])
def test_measured_pid_route_passes_all_anchors_then_stops(points, horizon):
    # 10 s matches the user's current YAML; retain 20 s as the earlier baseline.
    cfg = resolve_parameters({'dynamics.velocity_max': 3., T+'los.trajectory_time': horizon})
    model = USVModel.from_mapping(section(cfg, 'dynamics.'))
    pid = PIDController(model, section(cfg, C+'pid_controller.'))
    guidance = RouteTrajectoryPlanner(model, section(cfg, T+'los.'))
    state = VesselState(5., 10., stamp=1.)
    index, passages, trace = 0, [], []
    trajectory = None
    for tick in range(2500):
        distance = math.hypot(state.x-points[index][0], state.y-points[index][1])
        if index < len(points)-1 and distance <= 1.5:
            passages.append((index, distance, math.hypot(state.u, state.v)))
            index += 1
            trajectory = None
        if tick % 2 == 0 or trajectory is None:
            legs, start = [], state.pose
            for point in points[index:]:
                legs.append(np.asarray([start, point[:3]]))
                start = point[:3]
            result = guidance.plan(legs, [p[3] for p in points[index:]], state,
                                   goal=points[-1][:2] if index == len(points)-1 else None)
            assert result.status == AlgorithmStatus.SUCCESS, result.message
            trajectory = result.trajectory
        command = pid.compute(state, trajectory, .1)
        trace.append((state.x, state.y, state.u, state.v, index))
        if command.status == AlgorithmStatus.GOAL_REACHED:
            break
        assert command.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.SETTLING)
        if index < len(points)-1:
            assert command.status == AlgorithmStatus.SUCCESS
        state = step_dynamics(state, command, model, .1)
    assert index == len(points)-1, (index, state, passages)
    assert command.status == AlgorithmStatus.GOAL_REACHED, (state, passages)
    assert len(passages) == len(points)-1
    assert all(distance <= 1.5 and speed > .02 for _, distance, speed in passages)
    assert all(speed <= points[i][3]+.1 for i, _, speed in passages)
    assert math.hypot(state.x-points[-1][0], state.y-points[-1][1]) <= 1.5
    assert math.hypot(state.u, state.v) <= .25
    print({'horizon': horizon, 'passages': passages, 'final_xy': (state.x, state.y),
           'minimum_distances': [float(np.linalg.norm(np.asarray(trace)[np.asarray(trace)[:, 4] >= i, :2]-p[:2], axis=1).min())
                                 for i, p in enumerate(points)],
           'final_speed': math.hypot(state.u, state.v), 'seconds': tick*.1})


def test_unknown_forward_leg_stops_without_terminal_completion():
    cfg = resolve_parameters({})
    model = USVModel.from_mapping(section(cfg, 'dynamics.'))
    pid = PIDController(model, section(cfg, C+'pid_controller.'))
    guidance = RouteTrajectoryPlanner(model, section(cfg, T+'los.'))
    state = VesselState(5., 10., u=1., stamp=1.)
    for tick in range(400):
        if tick % 2 == 0:
            trajectory = guidance.plan([np.array([state.pose, (15., 10., 0.)])],
                                       [1.], state, goal=None).trajectory
        command = pid.compute(state, trajectory, .1)
        assert command.status == AlgorithmStatus.SUCCESS
        state = step_dynamics(state, command, model, .1)
    assert state.x < 15.5
    assert math.hypot(state.u, state.v) < .05
