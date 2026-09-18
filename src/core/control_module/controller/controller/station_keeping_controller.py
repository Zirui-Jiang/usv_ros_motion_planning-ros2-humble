"""Persistent position/fixed-yaw control using independent ADRC inner loops."""
import math

from common.dynamics import allocate_force
from common.math_helper import wrap_angle
from common.models import AlgorithmStatus, ControlCommand
from .adrc_core import PIController, ADRCController as HeadingController


def _clip(value, limit):
    return max(-limit, min(limit, value))


class StationKeepingController:
    """ADRC-backed persistent station keeping for a differential USV."""

    DEFAULTS = {
        "position_tolerance": .5, "heading_tolerance_deg": 5.,
        "position_speed_gain": .5, "recover_speed": .3,
        "recover_speed_tolerance": .04,
        "k_p_position": 15., "k_d_surge": 30., "k_d_yaw": 10.,
        "k_p_heading": 12., "k_d_heading": 10., "k_i_heading": .03,
        "force_limit": 20., "moment_limit": 8.,
        "force_min": -20., "force_max": 20., "moment_min": -8., "moment_max": 8.,
        "speed_integral_min": -8., "speed_integral_max": 8.,
        "heading_integral_min": -.25, "heading_integral_max": .25,
        "k_p_speed": 30., "k_i_speed": .8,
        "sample_time": .1, "max_sample_gap": .5,
        "error_reset_deg": 20., "filter_window_length": 10,
        "filter_history_length": 50, "acceleration_gain": 0.,
        "command_limit": .4, "command_slew_rate": .6,
        "alignment_time": 1., "alignment_yaw_rate": .02,
        "free_heading_yaw_damping": 10.,
    }

    def __init__(self, model, config=None):
        self.model = model
        supplied = dict(config or {})
        unknown = set(supplied) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown station keeping config: {sorted(unknown)}")
        cfg = {**self.DEFAULTS, **supplied}
        if "force_limit" in supplied and "force_max" not in supplied:
            cfg["force_max"], cfg["force_min"] = cfg["force_limit"], -cfg["force_limit"]
        if "moment_limit" in supplied and "moment_max" not in supplied:
            cfg["moment_max"], cfg["moment_min"] = cfg["moment_limit"], -cfg["moment_limit"]
        if "k_d_yaw" in supplied and "k_d_heading" not in supplied:
            cfg["k_d_heading"] = cfg["k_d_yaw"]
        if "k_p_position" in supplied and "position_speed_gain" not in supplied:
            cfg["position_speed_gain"] = 1.0 / max(1.0, cfg["k_p_position"])
        for key, value in cfg.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"{key} must be finite and numeric")
            cfg[key] = float(value)
        if not 0 < cfg["command_limit"] <= 1 or cfg["command_slew_rate"] <= 0:
            raise ValueError("Invalid command envelope")
        if cfg["heading_tolerance_deg"] <= 0 or cfg["heading_tolerance_deg"] > 180:
            raise ValueError("Invalid heading tolerance")
        if (cfg["position_tolerance"] <= 0 or cfg["recover_speed"] <= 0
                or cfg["recover_speed_tolerance"] <= 0 or cfg["alignment_time"] <= 0):
            raise ValueError("Position and recovery limits must be positive")
        if (cfg["position_speed_gain"] < 0 or cfg["alignment_yaw_rate"] < 0
                or cfg["free_heading_yaw_damping"] < 0):
            raise ValueError("Position gain and free-heading damping must be nonnegative")
        if cfg["sample_time"] <= 0 or cfg["max_sample_gap"] <= 0:
            raise ValueError("Sample times must be positive")
        if cfg["filter_window_length"] < 2 or cfg["filter_history_length"] < cfg["filter_window_length"]:
            raise ValueError("Invalid heading filter lengths")
        if not cfg["filter_window_length"].is_integer() or not cfg["filter_history_length"].is_integer():
            raise ValueError("Heading filter lengths must be integers")
        cfg["filter_window_length"] = int(cfg["filter_window_length"])
        cfg["filter_history_length"] = int(cfg["filter_history_length"])
        for prefix in ("force", "moment", "speed_integral", "heading_integral"):
            low, high = cfg[prefix + "_min"], cfg[prefix + "_max"]
            if low >= high or not low <= 0 <= high:
                raise ValueError(f"{prefix} limits must include zero")
        if not 0 < cfg["error_reset_deg"] <= 180:
            raise ValueError("Invalid error reset angle")
        for key in ("k_p_speed", "k_i_speed", "k_p_heading", "k_d_heading", "k_i_heading", "acceleration_gain"):
            if cfg[key] < 0:
                raise ValueError(f"{key} must be nonnegative")
        allocate_force(0., 0., model, cfg["command_limit"])
        self.config = cfg
        self.speed_loop = PIController({
            "sample_time": cfg["sample_time"], "k_p_speed": cfg["k_p_speed"],
            "k_i_speed": cfg["k_i_speed"], "force_min": cfg["force_min"],
            "force_max": cfg["force_max"], "integral_min": cfg["speed_integral_min"],
            "integral_max": cfg["speed_integral_max"],
        })
        self.heading_loop = HeadingController({
            "sample_time": cfg["sample_time"], "k_p_heading": cfg["k_p_heading"],
            "k_d_heading": cfg["k_d_heading"], "k_i_heading": cfg["k_i_heading"],
            "moment_min": cfg["moment_min"], "moment_max": cfg["moment_max"],
            "integral_min": cfg["heading_integral_min"], "integral_max": cfg["heading_integral_max"],
            "error_reset_deg": cfg["error_reset_deg"], "acceleration_gain": cfg["acceleration_gain"],
            "filter_window_length": cfg["filter_window_length"],
            "filter_history_length": cfg["filter_history_length"],
        })
        self.reset()

    def reset(self):
        self.phase = "RECOVER"
        self._aligned_duration = 0.
        self._last = ControlCommand()
        self.last_sample_stamp = None
        self._pending_tracking = None
        self.speed_loop.reset()
        self.heading_loop.reset()

    def observe_command(self, command):
        if not command.is_valid() or command.lateral != 0:
            raise ValueError("Observed station command exceeds actuator limits")
        if self._pending_tracking is not None:
            self._tracking_anti_windup(command, self._pending_tracking)
            self._pending_tracking = None
        self._last = command

    def _tracking_anti_windup(self, command, pending):
        """Freeze an integrator when the actually published command under-delivers.

        ``pending`` contains the force/moment request and integral snapshot for
        the preceding control computation.  This is deliberately applied after
        slew limiting and again when a safety gate reports its replacement
        command through :meth:`observe_command`.
        """
        actual_surge = (command.left + command.right) * self.model.max_force
        actual_yaw = ((command.right - command.left) * self.model.max_force * self.model.D)
        speed_delta = self.speed_loop.integral_speed - pending["speed_integral"]
        if (pending["surge"] - actual_surge) * speed_delta > 1e-12:
            self.speed_loop.integral_speed = pending["speed_integral"]
            self.speed_loop.refresh_output()
        if pending["heading_active"] and not self.heading_loop.integral_reset:
            heading_delta = self.heading_loop.integral_heading - pending["heading_integral"]
            if (pending["yaw"] - actual_yaw) * (-heading_delta) > 1e-12:
                self.heading_loop.integral_heading = pending["heading_integral"]
                self.heading_loop.refresh_output()

    def _allocate(self, surge, yaw, previous_speed_integral, previous_heading_integral):
        command = allocate_force(surge, yaw, self.model, self.config["command_limit"])
        applied_surge = (command.left + command.right) * self.model.max_force
        applied_yaw = (command.right - command.left) * self.model.max_force * self.model.D
        speed_delta = self.speed_loop.integral_speed - previous_speed_integral
        heading_delta = self.heading_loop.integral_heading - previous_heading_integral
        changed = False
        if (surge - applied_surge) * speed_delta > 1e-12:
            self.speed_loop.integral_speed = previous_speed_integral
            surge = self.speed_loop.refresh_output()
            changed = True
        if (not self.heading_loop.integral_reset
                and (yaw - applied_yaw) * (-heading_delta) > 1e-12):
            self.heading_loop.integral_heading = previous_heading_integral
            yaw = self.heading_loop.refresh_output()
            changed = True
        return allocate_force(surge, yaw, self.model, self.config["command_limit"]) if changed else command

    def compute(self, state, target_pose, dt):
        try:
            target = tuple(None if v is None else float(v) for v in target_pose)
            valid = (len(target) == 3 and all(v is not None and math.isfinite(v) for v in target[:2])
                     and (target[2] is None or math.isfinite(target[2]))
                     and state.is_finite() and math.isfinite(float(dt)) and float(dt) > 0)
        except (TypeError, ValueError, AttributeError):
            valid = False
        if not valid:
            self.reset()
            return ControlCommand(status=AlgorithmStatus.INVALID_INPUT, message="Invalid station control input")
        try:
            # If no safety observer arrived since the previous tick, the value
            # returned last tick is treated as the actual published command.
            self._pending_tracking = None
            stamp = float(state.stamp)
            if self.last_sample_stamp is not None:
                elapsed = stamp - self.last_sample_stamp
                if elapsed < 0:
                    raise ValueError("measurement timestamp moved backwards")
                if elapsed > self.config["max_sample_gap"] + 1e-9:
                    self.speed_loop.reset(); self.heading_loop.reset(); self.last_sample_stamp = None
                    sample_dt = self.config["sample_time"]
                else:
                    sample_dt = elapsed if elapsed > 0 else self.config["sample_time"]
                duplicate = elapsed == 0
            else:
                sample_dt, duplicate = self.config["sample_time"], False
            cfg = self.config
            dx, dy = target[0] - state.x, target[1] - state.y
            c, s = math.cos(state.psi), math.sin(state.psi)
            ex, ey = c * dx + s * dy, -s * dx + c * dy
            distance, speed = math.hypot(dx, dy), math.hypot(state.u, state.v)
            tolerance, heading_enabled = cfg["position_tolerance"], target[2] is not None
            heading_error = wrap_angle(target[2] - state.psi) if heading_enabled else 0.
            if self.phase != "RECOVER" and (distance > .7 * tolerance or abs(ey) > .5 * tolerance):
                self.phase, self._aligned_duration = "RECOVER", 0.
            if self.phase == "RECOVER" and distance <= .3 * tolerance and speed <= cfg["recover_speed_tolerance"]:
                self.phase, self._aligned_duration = ("ALIGN" if heading_enabled else "HOLD"), 0.
            if self.phase == "ALIGN":
                aligned = (abs(heading_error) <= math.radians(.4 * cfg["heading_tolerance_deg"])
                           and abs(state.r) <= cfg["alignment_yaw_rate"])
                # Alignment is a runtime phase timer; duplicate source stamps
                # must not advance ADRC integrators, but the control callback
                # may still spend wall-clock time in this phase.
                # Alignment is tied to fresh source observations.  A steady
                # timer may invoke this method repeatedly while the odometry
                # stamp is unchanged; those callbacks must not make a stale
                # pose satisfy the persistent alignment dwell time.
                self._aligned_duration += sample_dt if aligned and not duplicate else 0.
                if self._aligned_duration >= cfg["alignment_time"]:
                    self.phase = "HOLD"
            if self.phase == "RECOVER":
                bearing = math.atan2(dy, dx)
                forward_error = wrap_angle(bearing - state.psi)
                reverse_error = wrap_angle(bearing + math.pi - state.psi)
                course_error = forward_error if abs(forward_error) <= abs(reverse_error) else reverse_error
                desired_heading = wrap_angle(state.psi + course_error)
                desired_speed = _clip(cfg["position_speed_gain"] * ex, cfg["recover_speed"])
                desired_speed *= max(0., math.cos(course_error)) ** 2
            elif self.phase == "ALIGN":
                desired_heading, desired_speed = target[2], 0.
            else:
                desired_speed = _clip(cfg["position_speed_gain"] * ex, cfg["recover_speed"])
                desired_heading = target[2] if heading_enabled else None
            previous_speed_integral = self.speed_loop.integral_speed
            surge = (self.speed_loop.compute(state.psi if desired_heading is None else desired_heading,
                                             state.psi, desired_speed, state.u, sample_dt)
                     if not duplicate else self.speed_loop.refresh_output())
            if desired_heading is None:
                self.heading_loop.reset()
                previous_heading_integral = 0.0
                yaw = _clip(-cfg["free_heading_yaw_damping"] * state.r, cfg["moment_max"])
            else:
                previous_heading_integral = self.heading_loop.integral_heading
                yaw = (self.heading_loop.compute(desired_heading, state.psi, desired_speed, state.u,
                                                 sample_dt, measurement_stamp=stamp)
                       if not duplicate else self.heading_loop.refresh_output())
            requested = self._allocate(surge, yaw, previous_speed_integral, previous_heading_integral)
            effective_surge = self.speed_loop.refresh_output()
            effective_yaw = (self.heading_loop.refresh_output() if desired_heading is not None else yaw)
            delta = cfg["command_slew_rate"] * float(dt)
            result = ControlCommand(self._last.left + _clip(requested.left - self._last.left, delta),
                                    self._last.right + _clip(requested.right - self._last.right, delta), message=self.phase)
            if not result.is_valid():
                raise ValueError("Invalid station ADRC command")
            pending = {"surge": effective_surge, "yaw": effective_yaw,
                       "speed_integral": previous_speed_integral,
                       "heading_integral": previous_heading_integral,
                       "heading_active": desired_heading is not None}
            self._tracking_anti_windup(result, pending)
            self._pending_tracking = pending
            self._last = result
            self.last_sample_stamp = stamp
            return result
        # Any failure after either inner loop has mutated state must be
        # fail-safe and atomic: a caller receives zero thrust and the next
        # valid sample starts with fresh ADRC histories.  This deliberately
        # includes unexpected runtime errors from an injected/core component;
        # continuing with a partially updated integrator would be unsafe.
        except Exception:
            self.reset()
            return ControlCommand(status=AlgorithmStatus.INVALID_INPUT, message="Invalid station control input")
