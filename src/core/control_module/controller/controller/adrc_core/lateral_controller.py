"""Lateral-position controller for the CN control interface.

This module produces a lateral force command only. It assumes the vessel has
fully-rotatable thrusters, so the force can later be allocated to thruster
angles and magnitudes by a separate allocator.
"""

import math


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


class LateralController:
    """Lateral controller for a 30 kg surface USV.

    Inputs:
        mode: 0 = lateral control disabled, 1 = lateral control enabled.
        lateral_distance: desired lateral offset in meters, positive to the right.
        measured_lateral_distance: current lateral offset in meters, positive
            to the right. If not provided, it is treated as 0.0.
        dt: optional sample time in seconds. If None, config sample_time is used.

    Output:
        lateral_force: commanded lateral force in newtons, positive to the right.
    """

    DEFAULTS = {
        "sample_time": 0.1,
        "k_p_lateral": 4.0,
        "k_d_lateral": 1.2,
        "k_i_lateral": 0.1,
        "force_min": -6.0,
        "force_max": 6.0,
        "integral_min": -2.0,
        "integral_max": 2.0,
        "distance_deadband": 0.02,
        "velocity_deadband": 0.02,
        "error_reset_distance": 1.0,
    }

    def __init__(self, config=None):
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        self.reset()

    def update_config(self, config):
        candidate = {**self.config, **dict(config or {})}
        unknown = set(candidate) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown lateral config: {sorted(unknown)}")
        for key, value in candidate.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["sample_time"] <= 0.0:
            raise ValueError("sample_time must be positive")
        if candidate["force_min"] >= candidate["force_max"]:
            raise ValueError("force_min must be smaller than force_max")
        if candidate["integral_min"] >= candidate["integral_max"]:
            raise ValueError("integral_min must be smaller than integral_max")
        if candidate["k_p_lateral"] < 0.0:
            raise ValueError("k_p_lateral must be nonnegative")
        if candidate["k_d_lateral"] < 0.0:
            raise ValueError("k_d_lateral must be nonnegative")
        if candidate["k_i_lateral"] < 0.0:
            raise ValueError("k_i_lateral must be nonnegative")
        if candidate["distance_deadband"] < 0.0:
            raise ValueError("distance_deadband must be nonnegative")
        if candidate["velocity_deadband"] < 0.0:
            raise ValueError("velocity_deadband must be nonnegative")
        if candidate["error_reset_distance"] < 0.0:
            raise ValueError("error_reset_distance must be nonnegative")
        self.config = candidate

    def reset(self):
        self.integral_lateral = 0.0
        self.previous_error = None
        self.lateral_force = 0.0
        self.last_error = 0.0
        self.last_error_rate = 0.0

    def compute(self, mode, lateral_distance, measured_lateral_distance=0.0, dt=None):
        cfg = self.config

        mode = int(mode)
        if mode not in (0, 1):
            raise ValueError("mode must be 0 or 1")

        if mode == 0:
            self.reset()
            return 0.0

        step = cfg["sample_time"] if dt is None else float(dt)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("dt must be finite and positive")

        desired = float(lateral_distance)
        measured = float(measured_lateral_distance)
        if not math.isfinite(desired) or not math.isfinite(measured):
            raise ValueError("lateral distance inputs must be finite")

        error = desired - measured
        if abs(error) < cfg["distance_deadband"]:
            error = 0.0

        if self.previous_error is None:
            error_rate = 0.0
        else:
            error_rate = (error - self.previous_error) / step
        if abs(error_rate) < cfg["velocity_deadband"]:
            error_rate = 0.0
        self.previous_error = error
        self.last_error = error
        self.last_error_rate = error_rate

        integral_candidate = _saturate(
            self.integral_lateral + cfg["k_i_lateral"] * error * step,
            cfg["integral_min"],
            cfg["integral_max"],
        )

        force_candidate = (
            cfg["k_p_lateral"] * error
            + cfg["k_d_lateral"] * error_rate
            + integral_candidate
        )
        force = _saturate(force_candidate, cfg["force_min"], cfg["force_max"])

        saturated_against_error = (
            (force_candidate > cfg["force_max"] and error < 0.0)
            or (force_candidate < cfg["force_min"] and error > 0.0)
        )
        if not saturated_against_error:
            self.integral_lateral = integral_candidate

        if abs(error) > cfg["error_reset_distance"]:
            self.integral_lateral = 0.0

        self.lateral_force = float(force)
        return self.lateral_force


LateralControllerUSV = LateralController