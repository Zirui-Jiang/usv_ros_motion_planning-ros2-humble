"""Longitudinal PI controller using the CN scalar control interface."""

import math


def _saturate(value, lower, upper):
    if not math.isfinite(float(value)):
        raise ValueError("controller arithmetic must be finite")
    return min(max(float(value), float(lower)), float(upper))


class PIController:
    """PI speed controller for a 30 kg surface USV.

    Inputs to compute are desired/current heading in radians and desired/current
    speed in m/s. Heading inputs are accepted for interface consistency and are
    not used by the longitudinal PI channel.
    """

    DEFAULTS = {
        "sample_time": 0.1,
        "k_p_speed": 8.0,
        "k_i_speed": 0.8,
        "force_min": -8.0,
        "force_max": 8.0,
        "integral_min": -8.0,
        "integral_max": 8.0,
    }

    def __init__(self, config=None):
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        self.reset()

    def update_config(self, config):
        candidate = {**self.config, **dict(config or {})}
        unknown = set(candidate) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown PI config: {sorted(unknown)}")
        for key, value in candidate.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["sample_time"] <= 0:
            raise ValueError("sample_time must be positive")
        if not candidate["force_min"] <= 0 <= candidate["force_max"] or candidate["force_min"] >= candidate["force_max"]:
            raise ValueError("force_min must be smaller than force_max")
        if candidate["integral_min"] >= candidate["integral_max"]:
            raise ValueError("integral_min must be smaller than integral_max")
        if candidate["k_p_speed"] < 0 or candidate["k_i_speed"] < 0:
            raise ValueError("PI gains must be nonnegative")
        self.config = candidate

    def reset(self):
        self.integral_speed = 0.0
        self.last_speed_error = 0.0
        self.surge_force = 0.0
        self.non_integral_output = 0.0

    def refresh_output(self):
        """Reapply output limits after outer allocation freezes an integral."""
        self.surge_force = _saturate(
            self.non_integral_output + self.integral_speed,
            self.config["force_min"], self.config["force_max"])
        return self.surge_force

    def compute(self, desire_psi, measured_psi, desire_speed,
                measured_speed, dt=None):
        if not all(math.isfinite(float(v)) for v in (desire_psi, measured_psi, desire_speed, measured_speed)):
            raise ValueError("controller inputs must be finite")
        cfg = self.config
        step = cfg["sample_time"] if dt is None else float(dt)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("dt must be finite and positive")
        desired_speed = float(desire_speed)
        measured_speed = float(measured_speed)
        if not math.isfinite(desired_speed) or not math.isfinite(measured_speed):
            raise ValueError("speed inputs must be finite")

        error = desired_speed - measured_speed
        integral = _saturate(
            self.integral_speed + cfg["k_i_speed"] * error * step,
            cfg["integral_min"], cfg["integral_max"]
        )
        non_integral = cfg["k_p_speed"] * error
        force_candidate = non_integral + integral
        _saturate(force_candidate, cfg["force_min"], cfg["force_max"])
        if not ((force_candidate > cfg["force_max"] and error > 0.0)
                or (force_candidate < cfg["force_min"] and error < 0.0)):
            self.integral_speed = integral
        self.last_speed_error = error
        self.non_integral_output = non_integral
        return self.refresh_output()


PIControllerUSV = PIController
