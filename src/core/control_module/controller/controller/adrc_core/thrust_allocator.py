"""Twin full-rotatable thruster allocator for the CN control interface.

Inputs:
    desired_surge_force: surge force in N, positive forward.
    desired_yaw_moment: yaw moment in N*m, positive counter-clockwise.
    mode: 0 = lateral disabled, 1 = lateral enabled.
    desired_lateral_force: lateral force in N, positive to the right.

Outputs:
    left_angle, left_rpm, right_angle, right_rpm

When mode == 0, both thruster angles are fixed to 0 rad and the allocator
behaves like a conventional twin fixed-propeller allocator.
When mode == 1, thruster angles are solved so that the combined thrust of the
two thrusters produces the requested surge force, lateral force and yaw moment.
"""

import math


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


def _wrap_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


class ThrustAllocator:
    """Allocate surge force, lateral force and yaw moment to two full-rotatable
    thrusters.

    Force inputs are in newtons, yaw moment in N*m. Angles are in radians,
    rpm are signed revolutions per minute. Positive rpm means the propeller
    produces positive thrust along its own angle direction.
    """

    DEFAULTS = {
        "water_density": 1000.0,
        "propeller_diameter": 0.259,
        "propeller_spacing": 15.0,
        "thrust_coefficient": 0.5,
        "left_rpm_min": -5000.0,
        "left_rpm_max": 5000.0,
        "right_rpm_min": -5000.0,
        "right_rpm_max": 5000.0,
        "force_deadband": 1.0e-9,
        "angle_min": -math.pi,
        "angle_max": math.pi,
        "angle_rate_limit": 0.0,  # rad/s, 0 means no limit
    }

    def __init__(self, config=None):
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        self.reset()

    def update_config(self, config):
        candidate = {**self.config, **dict(config or {})}
        unknown = set(candidate) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown thrust allocator config: {sorted(unknown)}")
        for key, value in candidate.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            candidate[key] = value
        if candidate["water_density"] <= 0.0:
            raise ValueError("water_density must be positive")
        if candidate["propeller_diameter"] <= 0.0:
            raise ValueError("propeller_diameter must be positive")
        if candidate["propeller_spacing"] <= 0.0:
            raise ValueError("propeller_spacing must be positive")
        if candidate["thrust_coefficient"] <= 0.0:
            raise ValueError("thrust_coefficient must be positive")
        if candidate["left_rpm_min"] >= candidate["left_rpm_max"]:
            raise ValueError("left_rpm_min must be smaller than left_rpm_max")
        if candidate["right_rpm_min"] >= candidate["right_rpm_max"]:
            raise ValueError("right_rpm_min must be smaller than right_rpm_max")
        if candidate["force_deadband"] < 0.0:
            raise ValueError("force_deadband must be nonnegative")
        if candidate["angle_min"] >= candidate["angle_max"]:
            raise ValueError("angle_min must be smaller than angle_max")
        if candidate["angle_rate_limit"] < 0.0:
            raise ValueError("angle_rate_limit must be nonnegative")
        self.config = candidate

    def reset(self):
        self.left_force = 0.0
        self.right_force = 0.0
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.leftrpm = 0.0
        self.rightrpm = 0.0

    def _force_to_rpm(self, force):
        cfg = self.config
        force = float(force)
        if abs(force) <= cfg["force_deadband"]:
            return 0.0
        denominator = (
            cfg["thrust_coefficient"]
            * cfg["water_density"]
            * cfg["propeller_diameter"] ** 4
        )
        revolutions_per_second = math.copysign(
            math.sqrt(abs(force) / denominator),
            force,
        )
        return 60.0 * revolutions_per_second

    def _rpm_to_force(self, rpm):
        cfg = self.config
        rpm = float(rpm)
        if abs(rpm) <= 1.0e-12:
            return 0.0
        revolutions_per_second = rpm / 60.0
        return (
            cfg["thrust_coefficient"]
            * cfg["water_density"]
            * cfg["propeller_diameter"] ** 4
            * math.copysign(revolutions_per_second ** 2, revolutions_per_second)
        )

    def _limit_angle(self, angle):
        cfg = self.config
        angle = _wrap_angle(angle)
        return _saturate(angle, cfg["angle_min"], cfg["angle_max"])

    def _rate_limit_angle(self, previous_angle, target_angle, dt):
        cfg = self.config
        if cfg["angle_rate_limit"] <= 0.0 or dt is None:
            return target_angle
        step = float(dt)
        if not math.isfinite(step) or step <= 0.0:
            return target_angle
        delta = _wrap_angle(target_angle - previous_angle)
        max_delta = cfg["angle_rate_limit"] * step
        return _wrap_angle(previous_angle + _saturate(delta, -max_delta, max_delta))

    def _allocate_fixed(self, surge_force, yaw_moment):
        """Conventional fixed-propeller allocation, mode == 0."""
        spacing = self.config["propeller_spacing"]
        left_force = 0.5 * surge_force - yaw_moment / spacing
        right_force = 0.5 * surge_force + yaw_moment / spacing

        left_rpm = self._force_to_rpm(left_force)
        right_rpm = self._force_to_rpm(right_force)
        left_rpm = _saturate(
            left_rpm, self.config["left_rpm_min"], self.config["left_rpm_max"]
        )
        right_rpm = _saturate(
            right_rpm, self.config["right_rpm_min"], self.config["right_rpm_max"]
        )
        return 0.0, left_rpm, 0.0, right_rpm

    def _allocate_rotatable(self, surge_force, lateral_force, yaw_moment):
        """Full-rotatable allocation, mode == 1.

        Each thruster is at y = +spacing/2 (left) and y = -spacing/2 (right).
        Let the left thruster produce force vector (Fx_l, Fy_l) and the right
        thruster produce (Fx_r, Fy_r). The rigid-body wrench must satisfy:

            Fx_l + Fx_r = surge_force
            Fy_l + Fy_r = lateral_force
            (spacing/2) * Fx_l - (spacing/2) * Fx_r = yaw_moment

        This is 3 equations with 4 unknowns. The extra degree of freedom is
        resolved by minimizing the sum of squared thruster force magnitudes,
        which gives the minimum-energy solution:

            Fx_l = surge_force / 2 + yaw_moment / spacing
            Fx_r = surge_force / 2 - yaw_moment / spacing
            Fy_l = lateral_force / 2
            Fy_r = lateral_force / 2
        """
        spacing = self.config["propeller_spacing"]

        fx_left = 0.5 * surge_force + yaw_moment / spacing
        fx_right = 0.5 * surge_force - yaw_moment / spacing
        fy_left = 0.5 * lateral_force
        fy_right = 0.5 * lateral_force

        left_force = math.hypot(fx_left, fy_left)
        right_force = math.hypot(fx_right, fy_right)

        left_angle = self._limit_angle(math.atan2(fy_left, fx_left))
        right_angle = self._limit_angle(math.atan2(fy_right, fx_right))

        left_rpm = self._force_to_rpm(left_force)
        right_rpm = self._force_to_rpm(right_force)

        left_rpm = _saturate(
            left_rpm, self.config["left_rpm_min"], self.config["left_rpm_max"]
        )
        right_rpm = _saturate(
            right_rpm, self.config["right_rpm_min"], self.config["right_rpm_max"]
        )

        return left_angle, left_rpm, right_angle, right_rpm

    def compute(
        self,
        desired_surge_force,
        desired_yaw_moment,
        mode=0,
        desired_lateral_force=0.0,
        dt=None,
    ):
        surge_force = float(desired_surge_force)
        yaw_moment = float(desired_yaw_moment)
        lateral_force = float(desired_lateral_force)
        mode = int(mode)

        if mode not in (0, 1):
            raise ValueError("mode must be 0 or 1")
        if not math.isfinite(surge_force) or not math.isfinite(yaw_moment):
            raise ValueError("allocator inputs must be finite")
        if not math.isfinite(lateral_force):
            raise ValueError("lateral force must be finite")

        if mode == 0:
            left_angle, left_rpm, right_angle, right_rpm = self._allocate_fixed(
                surge_force, yaw_moment
            )
        else:
            left_angle, left_rpm, right_angle, right_rpm = self._allocate_rotatable(
                surge_force, lateral_force, yaw_moment
            )

        left_angle = self._rate_limit_angle(self.left_angle, left_angle, dt)
        right_angle = self._rate_limit_angle(self.right_angle, right_angle, dt)

        self.left_angle = float(left_angle)
        self.right_angle = float(right_angle)
        self.leftrpm = float(left_rpm)
        self.rightrpm = float(right_rpm)
        self.left_force = self._rpm_to_force(left_rpm)
        self.right_force = self._rpm_to_force(right_rpm)

        return self.left_angle, self.leftrpm, self.right_angle, self.rightrpm


ThrustAllocatorUSV = ThrustAllocator