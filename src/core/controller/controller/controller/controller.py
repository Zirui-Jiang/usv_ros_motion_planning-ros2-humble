"""Common validation and reference selection for normalized thruster control."""

import math

import numpy as np

from common.models import AlgorithmStatus, ControlCommand


def select_config(config, name):
    values = dict(config or {})
    values = values.get("controller", values)
    return dict(values.get(name + "_controller", values))


def stop(status, message=""):
    return ControlCommand(left=0.0, right=0.0, lateral=0.0, status=status, message=message)


class Controller:
    def __init__(self, model):
        self.model = model

    def reset(self):
        """Reset persistent algorithm state when starting a new navigation goal."""

    def _reference(self, state, trajectory, dt):
        try:
            if not math.isfinite(dt) or dt <= 0:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "control dt must be finite and positive")
            if not np.isfinite([state.x, state.y, state.psi, state.u, state.v, state.r, state.stamp]).all():
                return None, stop(AlgorithmStatus.INVALID_INPUT, "vessel state must be finite")
            if trajectory.empty():
                return None, stop(AlgorithmStatus.NO_PATH, "trajectory is empty")
            if state.frame_id and trajectory.frame_id and state.frame_id != trajectory.frame_id:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "state and trajectory frames differ")
            # A horizon endpoint is not necessarily the final navigation goal.
            if trajectory.goal is not None and math.hypot(state.x - trajectory.goal[0], state.y - trajectory.goal[1]) <= self.config["goal_distance_tolerance"]:
                self.reset()
                return None, stop(AlgorithmStatus.GOAL_REACHED)
            age = max(0.0, state.stamp - trajectory.stamp)
            reference = trajectory.reference_at_time(age + self.config["lookahead_time"])
            if reference.velocity < 0:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "reverse reference velocity is unsupported")
            # Looking ahead to a predicted arrival must not stop the actual
            # vessel before it enters the goal tolerance. Keep preview pose/
            # heading, but use an available positive approach speed until the
            # actual goal check above succeeds. Do not override a commanded
            # intermediate stop away from the navigation goal.
            terminal = trajectory[-1]
            if (trajectory.goal is not None and reference.velocity <= 1e-9
                    and terminal.velocity <= 1e-9
                    and math.hypot(terminal.x - trajectory.goal[0], terminal.y - trajectory.goal[1])
                    <= self.config["goal_distance_tolerance"]):
                current_speed = trajectory.reference_at_time(age).velocity
                if current_speed <= 1e-9:
                    positive = np.flatnonzero(trajectory.points[:, 6] > 1e-9)
                    current_speed = float(trajectory.points[positive[-1], 6]) if positive.size else 0.0
                if current_speed > 1e-9:
                    reference = reference._replace(velocity=current_speed)
            return reference, None
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            return None, stop(AlgorithmStatus.INVALID_INPUT, str(error))

    @staticmethod
    def _command(left, right):
        if not np.isfinite([left, right]).all():
            return stop(AlgorithmStatus.INVALID_INPUT, "nonfinite controller output")
        return ControlCommand(left=float(np.clip(left, -1.0, 1.0)),
                              right=float(np.clip(right, -1.0, 1.0)), lateral=0.0,
                              status=AlgorithmStatus.SUCCESS)
