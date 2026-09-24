"""Heading controller with fixed ADRC-style feedback gains for the CN interface."""

import math

try:
    from .heading_linear_regression_filter import HeadingLinearRegressionFilter
except ImportError:
    from heading_linear_regression_filter import HeadingLinearRegressionFilter


def _wrap_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


class ADRCController:
    """Fixed-gain heading controller for a 30 kg surface USV.

    Both heading inputs are radians. The measured heading is filtered internally
    to obtain yaw rate and yaw acceleration; no neural-network tuning is used.
    """

    DEFAULTS = {
        "sample_time": 0.1,
        "k_p_heading": 1.5,
        "k_d_heading": 0.5,
        "k_i_heading": 0.03,
        "moment_min": -2.0,
        "moment_max": 2.0,
        "integral_min": -0.25,
        "integral_max": 0.25,
        "error_reset_deg": 20.0,
        "acceleration_gain": 0.0,
        "filter_window_length": 10,
        "filter_history_length": 50,
    }

    def __init__(self, config=None):
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        cfg = self.config
        self.heading_filter = HeadingLinearRegressionFilter(
            cfg["sample_time"], cfg["filter_window_length"],
            cfg["filter_history_length"]
        )
        self.reset()

    def update_config(self, config):
        candidate = {**self.config, **dict(config or {})}
        unknown = set(candidate) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown ADRC config: {sorted(unknown)}")
        for key, value in candidate.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["sample_time"] <= 0:
            raise ValueError("sample_time must be positive")
        if candidate["moment_min"] >= candidate["moment_max"]:
            raise ValueError("moment_min must be smaller than moment_max")
        if candidate["filter_window_length"] < 2:
            raise ValueError("filter_window_length must be at least 2")
        if candidate["k_p_heading"] < 0 or candidate["k_d_heading"] < 0 or candidate["k_i_heading"] < 0:
            raise ValueError("heading gains must be nonnegative")
        self.config = candidate

    def reset(self):
        self.integral_heading = 0.0
        self.previous_error = None
        self.yaw_moment = 0.0
        if hasattr(self, "heading_filter"):
            self.heading_filter.reset()

    def compute(self, desire_psi, measured_psi, desire_speed,
                measured_speed, dt=None):
        del desire_speed, measured_speed
        cfg = self.config
        step = cfg["sample_time"] if dt is None else float(dt)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("dt must be finite and positive")
        desire_psi = float(desire_psi)
        measured_psi = float(measured_psi)
        if not math.isfinite(desire_psi) or not math.isfinite(measured_psi):
            raise ValueError("heading inputs must be finite")

        self.heading_filter.update(measured_psi)
        filtered_psi, yaw_rate, yaw_acceleration = self.heading_filter.output()
        error = _wrap_angle(filtered_psi - desire_psi)
        if self.previous_error is None:
            error_rate = 0.0
        else:
            error_rate = _wrap_angle(error - self.previous_error) / step
        self.previous_error = error

        integral_candidate = _saturate(
            self.integral_heading + cfg["k_i_heading"] * error * step,
            cfg["integral_min"], cfg["integral_max"]
        )
        moment_candidate = -(
            cfg["k_p_heading"] * error
            + cfg["k_d_heading"] * error_rate
            + integral_candidate
        ) + cfg["acceleration_gain"] * yaw_acceleration
        moment = _saturate(moment_candidate, cfg["moment_min"], cfg["moment_max"])

        saturated_against_error = (
            (moment_candidate > cfg["moment_max"] and error < 0.0)
            or (moment_candidate < cfg["moment_min"] and error > 0.0)
        )
        if not saturated_against_error:
            self.integral_heading = integral_candidate
        if abs(error) > math.radians(cfg["error_reset_deg"]):
            self.integral_heading = 0.0
        self.yaw_moment = float(moment)
        return self.yaw_moment


ADRCControllerUSV = ADRCController
