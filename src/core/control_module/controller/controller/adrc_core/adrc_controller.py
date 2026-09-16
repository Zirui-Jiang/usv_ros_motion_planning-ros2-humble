"""Heading controller with fixed ADRC-style feedback gains for the CN interface."""

import math

try:
    from .heading_linear_regression_filter import HeadingLinearRegressionFilter
except ImportError:
    from heading_linear_regression_filter import HeadingLinearRegressionFilter


def _wrap_angle(angle):
    if not math.isfinite(float(angle)):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _saturate(value, lower, upper):
    if not math.isfinite(float(value)):
        raise ValueError("controller arithmetic must be finite")
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
            if key in ("filter_window_length", "filter_history_length") and isinstance(value, bool):
                raise ValueError(f"{key} must be an integer")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["sample_time"] <= 0:
            raise ValueError("sample_time must be positive")
        if not candidate["moment_min"] <= 0 <= candidate["moment_max"] or candidate["moment_min"] >= candidate["moment_max"]:
            raise ValueError("moment_min must be smaller than moment_max")
        if candidate["filter_window_length"] < 2:
            raise ValueError("filter_window_length must be at least 2")
        if candidate["k_p_heading"] < 0 or candidate["k_d_heading"] < 0 or candidate["k_i_heading"] < 0:
            raise ValueError("heading gains must be nonnegative")
        if candidate["acceleration_gain"] < 0:
            raise ValueError("acceleration_gain must be nonnegative")
        if candidate["integral_min"] >= candidate["integral_max"]:
            raise ValueError("integral_min must be smaller than integral_max")
        if not 0 < candidate["error_reset_deg"] <= 180:
            raise ValueError("error_reset_deg must lie in (0, 180]")
        for key in ("filter_window_length", "filter_history_length"):
            if not candidate[key].is_integer():
                raise ValueError(f"{key} must be an integer")
            candidate[key] = int(candidate[key])
        if candidate["filter_history_length"] < candidate["filter_window_length"]:
            raise ValueError("filter_history_length must be at least filter_window_length")
        structural_change = any(candidate[key] != self.config[key] for key in
                                ("sample_time", "filter_window_length", "filter_history_length"))
        self.config = candidate
        if structural_change and hasattr(self, "heading_filter"):
            self.heading_filter = HeadingLinearRegressionFilter(candidate["sample_time"], candidate["filter_window_length"], candidate["filter_history_length"])
            self.reset()

    def reset(self):
        self.integral_heading = 0.0
        self.previous_error = None
        self.yaw_moment = 0.0
        self.non_integral_output = 0.0
        self.integral_reset = False
        if hasattr(self, "heading_filter"):
            self.heading_filter.reset()

    def refresh_output(self):
        """Reapply output limits without advancing measurement/derivative history."""
        self.yaw_moment = _saturate(
            self.non_integral_output - self.integral_heading,
            self.config["moment_min"], self.config["moment_max"])
        return self.yaw_moment

    def compute(self, desire_psi, measured_psi, desire_speed,
                measured_speed, dt=None, *, measurement_stamp=None):
        if not all(math.isfinite(float(v)) for v in (desire_psi, measured_psi, desire_speed, measured_speed)):
            raise ValueError("controller inputs must be finite")
        cfg = self.config
        step = cfg["sample_time"] if dt is None else float(dt)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("dt must be finite and positive")
        desire_psi = float(desire_psi)
        measured_psi = float(measured_psi)
        if not math.isfinite(desire_psi) or not math.isfinite(measured_psi):
            raise ValueError("heading inputs must be finite")

        self.heading_filter.update(measured_psi, stamp=measurement_stamp, dt=step)
        filtered_psi, yaw_rate, yaw_acceleration = self.heading_filter.output()
        error = _wrap_angle(filtered_psi - desire_psi)
        if self.previous_error is None:
            error_rate = 0.0
        else:
            error_rate = _wrap_angle(error - self.previous_error) / step

        integral_candidate = _saturate(
            self.integral_heading + cfg["k_i_heading"] * error * step,
            cfg["integral_min"], cfg["integral_max"]
        )
        integral_reset = abs(error) > math.radians(cfg["error_reset_deg"])
        if integral_reset:
            integral_candidate = 0.0
        non_integral = -(
            cfg["k_p_heading"] * error
            + cfg["k_d_heading"] * error_rate
        ) + cfg["acceleration_gain"] * yaw_acceleration
        moment_candidate = non_integral - integral_candidate
        _saturate(moment_candidate, cfg["moment_min"], cfg["moment_max"])

        saturated_against_error = (
            (moment_candidate > cfg["moment_max"] and error < 0.0)
            or (moment_candidate < cfg["moment_min"] and error > 0.0)
        )
        if not saturated_against_error:
            self.integral_heading = integral_candidate
        if integral_reset:
            self.integral_heading = 0.0
        self.integral_reset = integral_reset
        self.previous_error = error
        self.non_integral_output = non_integral
        return self.refresh_output()


ADRCControllerUSV = ADRCController
