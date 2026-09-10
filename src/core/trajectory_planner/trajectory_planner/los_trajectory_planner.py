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


class LosTrajectoryPlanner:
    DEFAULTS = {
        "trajectory_time_interval": 0.5, "trajectory_time": 20.0,
        "switch_dist": 5.0, "goal_tolerance": 1.0,
        "lookahead_distance": 5.0, "yaw_rate_max": 0.3, "kp_yaw": 1.0,
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
        speed = min(math.hypot(state.u, state.v), self.model.cruising_speed)
        rows = [[0.0, 0.0, *xy, yaw, 0.0, speed, 0.0]]
        if len(waypoints) == 1:
            initial_distance = np.linalg.norm(waypoints[0, :2] - xy)
        else:
            initial_distance = np.linalg.norm(waypoints[1, :2] - xy) + np.linalg.norm(
                np.diff(waypoints[1:, :2], axis=0), axis=1).sum()
        initial_distance = max(float(initial_distance), goal_tolerance)
        reached = False
        steps = int(math.ceil(horizon / dt))
        for output_index in range(1, steps + 1):
            output_time = min(output_index * dt, horizon)
            previous_speed = speed
            yaw_rate = 0.0
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
                    while index < len(waypoints) - 2 and np.linalg.norm(waypoints[index + 1, :2] - xy) < cfg["switch_dist"]:
                        index += 1
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
                heading_error = wrap_angle(desired_yaw - yaw)
                yaw_rate = float(np.clip(cfg["kp_yaw"] * heading_error, -cfg["yaw_rate_max"], cfg["yaw_rate_max"]))
                speed = self.model.cruising_speed * float(np.clip(remaining / initial_distance, 0.0, 1.0))
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
