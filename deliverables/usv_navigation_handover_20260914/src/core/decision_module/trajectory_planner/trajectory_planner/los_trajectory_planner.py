"""Line-of-sight guidance with fixed output times and bounded yaw rate.

This keeps the original LOS rollout, with a nonnegative linear speed profile
based on remaining route length. The rollout uses that same speed, so the
position and velocity fields describe the same motion.
Ported from the original C++ implementation by Yang Haodong (2025).
"""

import math
import time

import numpy as np

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus, USVModel, VesselState
from .trajectory import Trajectory, TrajectoryResult
from .circle import CircleReference


class LosTrajectoryPlanner:
    DEFAULTS = {
        "trajectory_time_interval": 0.5, "trajectory_time": 20.0,
        "switch_dist": 5.0, "goal_tolerance": 1.0,
        "lookahead_distance": 5.0, "yaw_rate_max": 0.3, "kp_yaw": 1.0,
        "slowdown_distance": 3.0,
    }

    def __init__(self, model: USVModel, config=None):
        self.model = model
        supplied = dict(config or {})
        supplied = supplied.get("trajectory_planner", supplied)
        supplied = supplied.get("los", supplied)
        self.config = {**self.DEFAULTS, **supplied}
        for key in self.DEFAULTS:
            value = float(self.config[key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
            self.config[key] = value
        if not math.isfinite(model.cruising_speed) or model.cruising_speed < 0:
            raise ValueError("cruising_speed must be finite and nonnegative")
        sample_count = self.config["trajectory_time"] / self.config["trajectory_time_interval"]
        # Catch accidental parameter units before allocating an enormous plan.
        if sample_count > 100000 or self.config["trajectory_time"] / 0.1 > 1000000:
            raise ValueError("trajectory discretization exceeds the rollout limit")

    def plan_circle(self, circle: CircleReference, state: VesselState):
        """Roll out a continuous circle vector field, with curvature feedforward.

        The reference has no terminal goal or speed ramp. The normal correction
        is analytic; the yaw actuator bound remains the same as ordinary LOS.
        Midpoint integration avoids a numerical radius bias on small circles.
        """
        begun = time.perf_counter()
        try:
            if not state.is_finite() or circle.speed > self.model.velocity_max:
                raise ValueError('invalid circle state or speed beyond model limit')
            cfg = self.config
            if circle.speed / circle.radius > cfg['yaw_rate_max'] + 1e-12:
                raise ValueError('circle speed/radius exceeds yaw_rate_max')
            dt, horizon = cfg['trajectory_time_interval'], cfg['trajectory_time']
            xy = np.array([state.x, state.y], dtype=float)
            yaw, elapsed = float(wrap_angle(state.psi)), 0.
            sign, speed = circle.direction, circle.speed
            center = np.asarray(circle.center)

            def turn_rate(position, heading):
                delta = position-center
                radial = float(np.linalg.norm(delta))
                theta = math.atan2(delta[1], delta[0])
                error = radial-circle.radius
                desired = theta+sign*(math.pi/2+math.atan2(error, cfg['lookahead_distance']))
                # d(theta)/dt and d(arctan(error/L))/dt are the vector-field
                # heading's feedforward derivative, including off-circle flow.
                relative = wrap_angle(heading-theta)
                theta_dot = speed*math.sin(relative)/max(radial, 1e-6)
                radial_dot = speed*math.cos(relative)
                feedforward = theta_dot+sign*cfg['lookahead_distance']*radial_dot/(cfg['lookahead_distance']**2+error**2)
                return float(np.clip(feedforward+cfg['kp_yaw']*wrap_angle(desired-heading),
                                     -cfg['yaw_rate_max'], cfg['yaw_rate_max']))

            rate = turn_rate(xy, yaw)
            rows = [[0., 0., *xy, yaw, rate/speed, speed, 0.]]
            for n in range(1, int(math.ceil(horizon/dt))+1):
                output_time = min(n*dt, horizon)
                while elapsed < output_time-1e-12:
                    step = min(.05, output_time-elapsed)
                    initial_rate = turn_rate(xy, yaw)
                    middle_yaw = yaw+.5*initial_rate*step
                    middle_xy = xy+.5*speed*step*np.array([math.cos(yaw), math.sin(yaw)])
                    rate = turn_rate(middle_xy, middle_yaw)
                    xy += speed*step*np.array([math.cos(middle_yaw), math.sin(middle_yaw)])
                    yaw = wrap_angle(yaw+rate*step)
                    elapsed += step
                rows.append([output_time, speed*output_time, *xy, yaw, rate/speed, speed, 0.])
            return TrajectoryResult(AlgorithmStatus.SUCCESS,
                                    Trajectory(rows, state.stamp, state.frame_id, None),
                                    elapsed=time.perf_counter()-begun)
        except (ValueError, TypeError, ArithmeticError, AttributeError) as error:
            return TrajectoryResult(AlgorithmStatus.INVALID_INPUT, Trajectory(), str(error), time.perf_counter()-begun)

    def plan(self, path, state: VesselState):
        started = time.perf_counter()

        def result(status, trajectory=None, message=""):
            return TrajectoryResult(status, trajectory if trajectory is not None else Trajectory(),
                                    message, time.perf_counter() - started)

        try:
            waypoints = np.asarray(path, dtype=float)
            if waypoints.size == 0:
                return result(AlgorithmStatus.NO_PATH, message="path is empty")
            if waypoints.ndim != 2 or waypoints.shape[1] != 3 or not np.isfinite(waypoints).all():
                return result(AlgorithmStatus.INVALID_INPUT, message="path must be finite with shape (N, 3)")
            if not np.isfinite([state.x, state.y, state.psi, state.u, state.v, state.r, state.stamp]).all():
                return result(AlgorithmStatus.INVALID_INPUT, message="vessel state must be finite")
        except (TypeError, ValueError, AttributeError) as error:
            return result(AlgorithmStatus.INVALID_INPUT, message=str(error))

        keep = np.r_[True, np.linalg.norm(np.diff(waypoints[:, :2], axis=0), axis=1) > 1e-9]
        waypoints = waypoints[keep]
        cfg = self.config
        dt, horizon = cfg["trajectory_time_interval"], cfg["trajectory_time"]
        xy = np.array([state.x, state.y], dtype=float)
        yaw = float(wrap_angle(state.psi))
        goal = tuple(waypoints[-1, :2])
        goal_tolerance = cfg["goal_tolerance"]
        if np.linalg.norm(xy - goal) <= goal_tolerance:
            rows = [[0, 0, *xy, yaw, 0, 0, 0], [min(dt, horizon), 0, *xy, yaw, 0, 0, 0]]
            return result(AlgorithmStatus.GOAL_REACHED, Trajectory(rows, state.stamp, state.frame_id, goal))
        if self.model.cruising_speed == 0:
            return result(AlgorithmStatus.INVALID_INPUT, message="positive cruising_speed is required to move")

        index = 0
        travelled = 0.0
        elapsed = 0.0
        # This is a reference trajectory, so its first speed must follow the
        # reference profile, not inherit the measured (possibly zero) speed.
        # A stationary vessel near its goal otherwise receives an all-zero
        # trajectory when the first rollout sample enters the goal tolerance.
        if len(waypoints) == 1:
            initial_distance = np.linalg.norm(waypoints[0, :2] - xy)
        else:
            initial_distance = np.linalg.norm(waypoints[1, :2] - xy) + np.linalg.norm(
                np.diff(waypoints[1:, :2], axis=0), axis=1).sum()
        initial_distance = max(float(initial_distance), goal_tolerance)
        # A new near-goal rollout must not reset its initial speed to cruise.
        # The lower denominator bound preserves the original far-field ratio
        # while making repeated short approach plans consistently decelerate.
        speed_denominator = max(initial_distance, cfg["slowdown_distance"])
        speed = self.model.cruising_speed * min(1.0, initial_distance / speed_denominator)
        rows = [[0.0, 0.0, *xy, yaw, 0.0, speed, 0.0]]
        reached = False
        steps = int(math.ceil(horizon / dt))
        for output_index in range(1, steps + 1):
            output_time = min(output_index * dt, horizon)
            previous_speed = speed
            yaw_rate = 0.0
            allow_switch = True
            while elapsed < output_time - 1e-12:
                inner_dt = min(0.1, output_time - elapsed)
                if np.linalg.norm(xy - goal) <= goal_tolerance:
                    reached = True
                    speed = 0.0
                    elapsed = output_time
                    break
                if len(waypoints) == 1:
                    target = waypoints[0, :2]
                    desired_yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
                    remaining = np.linalg.norm(target - xy)
                else:
                    start = waypoints[index, :2]
                    target = waypoints[index + 1, :2]
                    delta = target - start
                    segment_yaw = math.atan2(delta[1], delta[0])
                    offset = xy - start
                    cross_track = -offset[0] * math.sin(segment_yaw) + offset[1] * math.cos(segment_yaw)
                    desired_yaw = segment_yaw - math.atan2(cross_track, cfg["lookahead_distance"])
                    if index == len(waypoints) - 2 and np.linalg.norm(target - xy) < cfg["switch_dist"]:
                        desired_yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
                    remaining = np.linalg.norm(target - xy) + np.linalg.norm(
                        np.diff(waypoints[index + 1:, :2], axis=0), axis=1).sum()
                    # Preserve the original waypoint progression: calculate
                    # guidance from the current segment first, then advance at
                    # most once at t=0, output_dt, 2*output_dt, ... . Jumping
                    # over every point within switch_dist changes cornering.
                    if (allow_switch and index < len(waypoints) - 2
                            and np.linalg.norm(target - xy) < cfg["switch_dist"]):
                        index += 1
                allow_switch = False
                heading_error = wrap_angle(desired_yaw - yaw)
                yaw_rate = float(np.clip(cfg["kp_yaw"] * heading_error, -cfg["yaw_rate_max"], cfg["yaw_rate_max"]))
                speed = self.model.cruising_speed * float(np.clip(remaining / speed_denominator, 0.0, 1.0))
                yaw = wrap_angle(yaw + yaw_rate * inner_dt)
                motion = speed * inner_dt
                xy += motion * np.array([math.cos(yaw), math.sin(yaw)])
                travelled += motion
                elapsed += inner_dt
            sample_dt = output_time - rows[-1][0]
            curvature = yaw_rate / speed if speed > 1e-9 and not reached else 0.0
            rows.append([output_time, travelled, *xy, yaw, curvature, speed,
                         (speed - previous_speed) / sample_dt])
            if reached:
                break
        trajectory = Trajectory(rows, state.stamp, state.frame_id, goal)
        # GOAL_REACHED describes the current vessel, not a predicted future point.
        return result(AlgorithmStatus.SUCCESS, trajectory)
