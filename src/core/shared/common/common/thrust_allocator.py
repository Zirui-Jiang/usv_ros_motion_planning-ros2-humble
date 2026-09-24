"""Fixed twin-propeller force allocation and optional RPM conversion.

The navigation stack still publishes normalized left/right commands.  This
module keeps that contract while retaining the physical force and RPM values
needed by a future actuator adapter.
"""

from dataclasses import dataclass
import math

from .models import ControlCommand


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


@dataclass(frozen=True)
class FixedForceAllocation:
    """Result of one fixed-direction twin-propeller allocation."""

    command: ControlCommand
    left_force: float
    right_force: float
    left_rpm: float
    right_rpm: float
    applied_surge_force: float
    applied_yaw_moment: float
    saturated: bool


class FixedThrustAllocator:
    """Allocate surge/yaw force to fixed left and right propellers.

    The project convention is ``Mz = (F_right - F_left) * D``.  The 0924
    implementation uses the total propeller spacing, so this adapter derives
    that spacing as ``2 * model.D`` and preserves the existing command API.
    """

    DEFAULTS = {
        "water_density": 1000.0,
        "propeller_diameter": 0.259,
        "thrust_coefficient": 0.5,
        "left_rpm_min": -5000.0,
        "left_rpm_max": 5000.0,
        "right_rpm_min": -5000.0,
        "right_rpm_max": 5000.0,
        "force_deadband": 1.0e-9,
    }

    def __init__(self, model, config=None):
        if not math.isfinite(float(model.D)) or model.D <= 0:
            raise ValueError("thruster arm D must be finite and positive")
        if not math.isfinite(float(model.max_force)) or model.max_force <= 0:
            raise ValueError("maximum thruster force must be finite and positive")
        self.model = model
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})

    def update_config(self, config):
        candidate = {**self.config, **dict(config or {})}
        unknown = set(candidate) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown fixed thrust allocator config: {sorted(unknown)}")
        for key, value in candidate.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["water_density"] <= 0 or candidate["propeller_diameter"] <= 0:
            raise ValueError("water density and propeller diameter must be positive")
        if candidate["thrust_coefficient"] <= 0:
            raise ValueError("thrust coefficient must be positive")
        for side in ("left", "right"):
            if candidate[side + "_rpm_min"] >= candidate[side + "_rpm_max"]:
                raise ValueError(f"{side} RPM limits must be ordered")
        if candidate["force_deadband"] < 0:
            raise ValueError("force deadband must be nonnegative")
        self.config = candidate

    def force_to_rpm(self, force):
        """Convert signed thrust in N to signed revolutions per minute."""
        force = float(force)
        if not math.isfinite(force):
            raise ValueError("force must be finite")
        if abs(force) <= self.config["force_deadband"]:
            return 0.0
        cfg = self.config
        denominator = (cfg["thrust_coefficient"] * cfg["water_density"]
                       * cfg["propeller_diameter"] ** 4)
        revolutions_per_second = math.copysign(math.sqrt(abs(force) / denominator), force)
        return 60.0 * revolutions_per_second

    def rpm_to_force(self, rpm):
        """Convert signed revolutions per minute back to signed thrust in N."""
        rpm = float(rpm)
        if not math.isfinite(rpm):
            raise ValueError("RPM must be finite")
        if abs(rpm) <= 1.0e-12:
            return 0.0
        cfg = self.config
        revolutions_per_second = rpm / 60.0
        return (cfg["thrust_coefficient"] * cfg["water_density"]
                * cfg["propeller_diameter"] ** 4
                * math.copysign(revolutions_per_second ** 2, revolutions_per_second))

    def allocate(self, surge_force, yaw_moment, command_limit=1.0):
        """Return normalized commands plus the physical achieved allocation."""
        surge_force, yaw_moment, command_limit = map(float, (surge_force, yaw_moment,
                                                               command_limit))
        if not all(math.isfinite(value) for value in (surge_force, yaw_moment, command_limit)):
            raise ValueError("force allocation inputs must be finite")
        if not 0 < command_limit <= 1:
            raise ValueError("command limit must be in (0, 1]")

        # 0924's fixed allocator uses total spacing; the current model stores
        # the one-sided arm D, so these equations are equivalent to the
        # existing common.dynamics allocation.
        spacing = 2.0 * self.model.D
        requested_left = 0.5 * surge_force - yaw_moment / spacing
        requested_right = 0.5 * surge_force + yaw_moment / spacing
        left_command = requested_left / self.model.max_force
        right_command = requested_right / self.model.max_force
        scale = max(1.0, abs(left_command) / command_limit,
                    abs(right_command) / command_limit)
        requested_left /= scale
        requested_right /= scale

        left_rpm = _saturate(self.force_to_rpm(requested_left),
                             self.config["left_rpm_min"], self.config["left_rpm_max"])
        right_rpm = _saturate(self.force_to_rpm(requested_right),
                              self.config["right_rpm_min"], self.config["right_rpm_max"])
        left_force = self.rpm_to_force(left_rpm)
        right_force = self.rpm_to_force(right_rpm)
        left_command = left_force / self.model.max_force
        right_command = right_force / self.model.max_force
        applied_surge = left_force + right_force
        applied_yaw = (right_force - left_force) * self.model.D
        saturated = (not math.isclose(left_force, requested_left, rel_tol=1e-12, abs_tol=1e-12)
                     or not math.isclose(right_force, requested_right, rel_tol=1e-12,
                                          abs_tol=1e-12))
        command = ControlCommand(left=left_command, right=right_command)
        if not command.is_valid():
            raise ValueError("fixed thrust allocation produced invalid command")
        return FixedForceAllocation(command, left_force, right_force, left_rpm, right_rpm,
                                    applied_surge, applied_yaw, saturated)
