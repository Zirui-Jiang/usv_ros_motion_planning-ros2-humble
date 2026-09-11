"""Original Otter speed/course PID and differential thrust allocation.

Gains retain the legacy normalized-command calibration: the allocation is
[[1, 1], [-D, D]], without another max_force division. ``u, v`` already use
the vessel frame. The original speed feedback is hypot(u, v).
Ported from the original C++ implementation by Yang Haodong (2025).
"""

import math

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus
from .controller import Controller, select_config, stop


class PIDController(Controller):
    DEFAULTS = {"goal_distance_tolerance": 1.0, "goal_speed_tolerance": 0.25, "lookahead_time": 3.0,
                "k_p_speed": 0.0, "k_i_speed": 0.0, "k_d_speed": 0.0,
                "k_p_course": 0.0, "k_i_course": 0.0, "k_d_course": 0.0}

    def __init__(self, model, config=None):
        super().__init__(model)
        if not math.isfinite(model.D) or model.D <= 0:
            raise ValueError("thruster arm D must be finite and positive")
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        self.reset()

    def update_config(self, config):
        """Validate atomically and retain integral/error history for gain tuning."""
        candidate = {**self.config, **select_config(config, "pid")}
        for key in self.DEFAULTS:
            value = float(candidate[key])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
            candidate[key] = value
        self.config = candidate

    def reset(self):
        self.integral_speed = 0.0
        self.integral_course = 0.0
        self.last_speed_error = 0.0
        self.last_course_error = 0.0

    def compute(self, state, trajectory, dt):
        reference, failure = self._reference(state, trajectory, dt)
        if failure is not None:
            return failure
        cfg = self.config
        speed_error = reference.velocity - math.hypot(state.u, state.v)
        course_error = wrap_angle(reference.theta - state.psi)
        integral_speed = self.integral_speed + cfg["k_i_speed"] * speed_error * dt
        integral_course = self.integral_course + cfg["k_i_course"] * course_error * dt
        surge = (cfg["k_p_speed"] * speed_error + integral_speed
                 + cfg["k_d_speed"] * (speed_error - self.last_speed_error) / dt)
        yaw = (cfg["k_p_course"] * course_error + integral_course
               + cfg["k_d_course"] * wrap_angle(course_error - self.last_course_error) / dt)
        if not all(math.isfinite(v) for v in [integral_speed, integral_course, surge, yaw]):
            return stop(AlgorithmStatus.INVALID_INPUT, "PID arithmetic produced nonfinite values")
        # Preserve legacy course-command regularization for calibrated gains.
        yaw = wrap_angle(yaw)
        command = self._command(0.5 * (surge - yaw / self.model.D),
                                0.5 * (surge + yaw / self.model.D))
        if command.status == AlgorithmStatus.SUCCESS:
            self.integral_speed = integral_speed
            self.integral_course = integral_course
            self.last_speed_error = speed_error
            self.last_course_error = course_error
        return command


PIDControllerOtter = PIDController
