"""Persistent position/fixed-yaw control for a differential, underactuated Otter.

RECOVER chooses forward/reverse to minimise turn, ALIGN turns to the requested
fixed yaw, and HOLD applies longitudinal/yaw PD. A lateral displacement requires
another recovery manoeuvre; no unavailable lateral thruster is assumed.
"""
import math

from common.dynamics import allocate_force
from common.math_helper import wrap_angle
from common.models import AlgorithmStatus, ControlCommand


def _clip(value, limit):
    return max(-limit, min(limit, value))


class StationKeepingController:
    DEFAULTS = {
        "position_tolerance": .5, "heading_tolerance_deg": 5.,
        "k_p_position": 15., "k_d_surge": 30.,
        "k_p_heading": 12., "k_d_yaw": 10.,
        "recover_speed": .3, "recover_speed_tolerance": .04,
        "force_limit": 20., "moment_limit": 8.,
        "command_limit": .4, "command_slew_rate": .6,
        "alignment_time": 1., "alignment_yaw_rate": .02,
    }

    def __init__(self, model, config=None):
        self.model = model
        supplied = dict(config or {})
        if set(supplied) - set(self.DEFAULTS):
            raise ValueError(f"Unknown station keeping config: {sorted(set(supplied) - set(self.DEFAULTS))}")
        self.config = {**self.DEFAULTS, **supplied}
        for key, value in self.config.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if self.config["command_limit"] > 1 or self.config["heading_tolerance_deg"] > 180:
            raise ValueError("Invalid command or heading tolerance")
        allocate_force(0, 0, model, self.config["command_limit"])
        self.reset()

    def reset(self):
        self.phase = "RECOVER"
        self._aligned_duration = 0.
        self._last = ControlCommand()

    def observe_command(self, command):
        """Synchronise slew history with the command actually sent to hardware.

        Call after controller handover or a braking/zero safety substitution.
        Accept the hardware range [-1, 1], including a navigation command above
        the normal station limit. Subsequent compute() calls slew toward a
        request within command_limit; the transitional output may exceed that
        station limit while remaining within hardware limits. The runtime must
        screen the actual transitional output with its swept-hull safety gate.
        Invalid observations are rejected atomically. Status is immaterial:
        this method records normalized values actually sent to propulsion.
        """
        if not command.is_valid() or command.lateral != 0:
            raise ValueError("Observed station command exceeds actuator limits")
        self._last = command

    def compute(self, state, target_pose, dt):
        """Return normalized thrust; target_pose is (world x, y, ROS yaw rad).

        Successful holding never emits GOAL_REACHED. A runtime cancellation or
        stale-data gate must bypass this controller and publish zero immediately.
        """
        try:
            target = tuple(float(v) for v in target_pose)
            valid = len(target) == 3 and all(math.isfinite(v) for v in target)
            valid = valid and state.is_finite() and math.isfinite(dt) and dt > 0
        except (TypeError, ValueError, AttributeError):
            valid = False
        if not valid:
            self.reset()
            return ControlCommand(status=AlgorithmStatus.INVALID_INPUT, message="Invalid station control input")

        cfg = self.config
        dx, dy = target[0] - state.x, target[1] - state.y
        c, s = math.cos(state.psi), math.sin(state.psi)
        ex, ey = c * dx + s * dy, -s * dx + c * dy
        distance, speed = math.hypot(dx, dy), math.hypot(state.u, state.v)
        tolerance = cfg["position_tolerance"]
        heading_error = wrap_angle(target[2] - state.psi)

        if self.phase != "RECOVER" and (distance > .7 * tolerance or abs(ey) > .5 * tolerance):
            self.phase = "RECOVER"
            self._aligned_duration = 0.
        if self.phase == "RECOVER" and distance <= .3 * tolerance and speed <= cfg["recover_speed_tolerance"]:
            self.phase = "ALIGN"
            self._aligned_duration = 0.
        if self.phase == "ALIGN":
            aligned = (abs(heading_error) <= math.radians(.4 * cfg["heading_tolerance_deg"])
                       and abs(state.r) <= cfg["alignment_yaw_rate"])
            self._aligned_duration = self._aligned_duration + dt if aligned else 0.
            if self._aligned_duration >= cfg["alignment_time"]:
                self.phase = "HOLD"

        if self.phase == "RECOVER":
            bearing = math.atan2(dy, dx)
            forward_error = wrap_angle(bearing - state.psi)
            reverse_error = wrap_angle(bearing + math.pi - state.psi)
            course_error = forward_error if abs(forward_error) <= abs(reverse_error) else reverse_error
            # Clip proportional demand to enforce a low recovery speed; at
            # large course errors only signed damping remains (no acceleration).
            drive = _clip(cfg["k_p_position"] * ex, cfg["k_d_surge"] * cfg["recover_speed"])
            drive *= max(0., math.cos(course_error)) ** 2
            surge = drive - cfg["k_d_surge"] * state.u
        else:
            course_error = heading_error
            surge = cfg["k_p_position"] * ex - cfg["k_d_surge"] * state.u
        yaw = cfg["k_p_heading"] * course_error - cfg["k_d_yaw"] * state.r
        requested = allocate_force(_clip(surge, cfg["force_limit"]),
                                   _clip(yaw, cfg["moment_limit"]), self.model, cfg["command_limit"])
        # Do not clamp an inherited navigation command to command_limit in one
        # step. A continuous handover converges into the station envelope at
        # the same slew rate, with actual outputs checked by the runtime gate.
        delta = cfg["command_slew_rate"] * dt
        result = ControlCommand(self._last.left + _clip(requested.left - self._last.left, delta),
                                self._last.right + _clip(requested.right - self._last.right, delta),
                                message=self.phase)
        self._last = result
        return result
