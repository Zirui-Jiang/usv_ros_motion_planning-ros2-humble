"""Continuous ordered-route rollout; mandatory anchors never use nearest pruning.

Each leg ends at an exact mandatory anchor. Only this prediction advances its
local node index; the decision node owns actual measured waypoint progress.
"""
import math
import time

import numpy as np

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus
from .los_trajectory_planner import LosTrajectoryPlanner
from .trajectory import Trajectory, TrajectoryResult


class RouteTrajectoryPlanner:
    def __init__(self, model, config=None, waypoint_radius=1.5, preview_time=2.6):
        self.model = model
        self.config = LosTrajectoryPlanner(model, config).config
        self.radius = float(waypoint_radius)
        self.preview = float(preview_time)
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('waypoint radius must be finite and positive')
        if not math.isfinite(self.preview) or self.preview < 0:
            raise ValueError('preview time must be finite and nonnegative')
        if model.acceleration_min >= 0:
            raise ValueError('continuous route requires a negative deceleration limit')

    def plan(self, legs, speeds, state, *, goal=None, deadline=math.inf):
        """Roll out a validated route prefix, stopping at its known endpoint.

        A prefix endpoint is a safety stop if later geometry is not yet known.
        ``goal`` is supplied only in the actual final-leg phase, independently
        of whether this finite prediction can see the end of the whole route.
        """
        begun = time.perf_counter()
        def result(status, trajectory=None, message=''):
            return TrajectoryResult(status, trajectory or Trajectory(), message,
                                    time.perf_counter()-begun)
        if not state.is_finite() or not legs or len(legs) != len(speeds):
            return result(AlgorithmStatus.INVALID_INPUT, message='invalid ordered route or state')
        nodes, limits = [np.asarray(state.pose[:2], float)], []
        for leg, speed in zip(legs, speeds):
            values = np.asarray(leg, dtype=float)
            if (values.ndim != 2 or values.shape[1] != 3 or not len(values)
                    or not np.isfinite(values).all() or not math.isfinite(speed)
                    or not 0 < speed <= self.model.velocity_max):
                return result(AlgorithmStatus.INVALID_INPUT, message='invalid route leg or speed')
            for point in values[:, :2]:
                if np.linalg.norm(point-nodes[-1]) > 1e-9:
                    nodes.append(point)
                    limits.append(float(speed))
        cfg = self.config
        dt, horizon = cfg['trajectory_time_interval'], cfg['trajectory_time']
        xy, yaw = np.asarray(state.pose[:2], float), float(state.psi)
        speed = math.hypot(state.u, state.v)
        rows = [[0., 0., *xy, yaw, 0., speed, 0.]]
        # Use a smaller prediction switching disk, reserving space for the
        # real controller's preview, tracking lag and finite turning radius.
        switch = min(.3, .2*self.radius)
        stop_radius = min(.5, .5*cfg['goal_tolerance'])
        brake = min(-self.model.acceleration_min, .5)
        accel = self.model.acceleration_max
        nodes = np.asarray(nodes)
        if len(nodes) == 1:
            nodes = np.vstack((nodes, nodes[-1]))
            limits = [float(speeds[-1])]
        delta = np.diff(nodes, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        headings = np.arctan2(delta[:, 1], delta[:, 0])
        # A conservative corner speed reserves half the passing disk for
        # preview travel and turning. Sharp corners are slow but nonzero.
        caps = np.array(limits, dtype=float)
        for i in range(len(caps)-1):
            angle = abs(wrap_angle(headings[i+1]-headings[i]))
            if angle > .1:
                turn_time = angle/cfg['yaw_rate_max'] + self.preview
                caps[i] = min(caps[i], .45*self.radius/turn_time)
            caps[i] = min(caps[i], limits[i+1])
        caps[-1] = 0.
        index, elapsed, travelled = 0, 0., 0.
        acceleration_holds = []
        for n in range(1, int(math.ceil(horizon/dt))+1):
            if time.perf_counter() >= deadline:
                return result(AlgorithmStatus.TIMEOUT, message='route rollout exceeded planning budget')
            output_time = min(n*dt, horizon)
            previous_speed, rate = speed, 0.
            while elapsed < output_time-1e-12:
                step = min(.1, output_time-elapsed)
                distance = float(np.linalg.norm(nodes[index+1]-xy))
                while index < len(limits)-1 and distance <= switch:
                    if limits[index+1] > limits[index]:
                        # The controller previews velocity as well as heading.
                        # Delay a predicted increase so it cannot accelerate
                        # the real boat to the next leg's speed before passage.
                        acceleration_holds.append((elapsed+self.preview, limits[index]))
                    index += 1
                    distance = float(np.linalg.norm(nodes[index+1]-xy))
                desired_yaw = math.atan2(nodes[index+1, 1]-xy[1], nodes[index+1, 0]-xy[0])
                error = wrap_angle(desired_yaw-yaw)
                rate = float(np.clip(cfg['kp_yaw']*error, -cfg['yaw_rate_max'], cfg['yaw_rate_max']))
                desired_speed = limits[index]
                for until, ceiling in acceleration_holds:
                    if elapsed < until:
                        desired_speed = min(desired_speed, ceiling)
                ahead = distance
                for j in range(index, len(limits)):
                    allowance = max(0., ahead-(stop_radius if j == len(limits)-1 else self.radius))
                    # Previewed velocity makes deceleration occur earlier in
                    # the actual PID, while this envelope keeps rollout feasible.
                    desired_speed = min(desired_speed, math.sqrt(caps[j]**2+2*brake*allowance))
                    if j+1 < len(limits):
                        ahead += lengths[j+1]
                if abs(error) > .2:
                    desired_speed = min(desired_speed, max(.06, .3*self.radius*cfg['yaw_rate_max']/abs(error)))
                if index == len(limits)-1 and distance <= stop_radius:
                    desired_speed, rate = 0., 0.
                new_speed = float(np.clip(desired_speed, max(0., speed+self.model.acceleration_min*step), speed+accel*step))
                motion = .5*(speed+new_speed)*step
                xy += motion*np.array([math.cos(yaw+.5*rate*step), math.sin(yaw+.5*rate*step)])
                yaw = wrap_angle(yaw+rate*step)
                speed = new_speed
                travelled += motion
                elapsed += step
            rows.append([output_time, travelled, *xy, yaw,
                         rate/speed if speed > 1e-9 else 0., speed,
                         (speed-previous_speed)/(output_time-rows[-1][0])])
            if index == len(limits)-1 and np.linalg.norm(nodes[-1]-xy) <= stop_radius and speed <= 1e-9:
                break
        return result(AlgorithmStatus.SUCCESS, Trajectory(rows, state.stamp, state.frame_id, goal))
