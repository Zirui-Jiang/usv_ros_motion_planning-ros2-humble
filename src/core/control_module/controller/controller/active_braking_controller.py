"""Bounded signed-velocity damping with measured, sustained stop confirmation.

Defaults are simulation starting values, not a minimum stopping-distance claim.
This controller has no position target or velocity integral. Runtime freshness,
module and swept-hull checks remain mandatory before publishing its command.
Stop confirmation uses successive source observations with at most a 0.5-second
gap; a time reversal or longer gap restarts confirmation, even if both samples
show low speed. Duplicate source timestamps never advance the confirmation.
"""
import math

from common.dynamics import braking_candidates
from common.models import AlgorithmStatus, ControlCommand


class ActiveBrakingController:
    DEFAULTS = {'speed_tolerance': .05, 'yaw_rate_tolerance': .02,
                'settle_time': 1., 'command_limit': .4, 'max_sample_gap': .5}

    def __init__(self, model, config=None):
        supplied = dict(config or {})
        if set(supplied) - set(self.DEFAULTS):
            raise ValueError('Unknown active braking configuration')
        self.config = {**self.DEFAULTS, **supplied}
        if any(isinstance(v, bool) or not isinstance(v, (float, int))
               or not math.isfinite(v) or v <= 0 for v in self.config.values()):
            raise ValueError('Active braking parameters must be finite and positive')
        if self.config['command_limit'] > 1:
            raise ValueError('Active braking command_limit must not exceed 1')
        self.model = model
        self.reset()

    def reset(self):
        self._quiet_since = self._last_sample_stamp = None

    def compute(self, state, dt):
        if not state.is_finite() or not math.isfinite(dt) or dt <= 0:
            self.reset()
            return ControlCommand(status=AlgorithmStatus.INVALID_INPUT,
                                  message='Invalid active braking input')
        if self._last_sample_stamp is not None:
            sample_gap = state.stamp - self._last_sample_stamp
            if sample_gap < 0 or sample_gap > self.config['max_sample_gap']:
                self.reset()
        quiet = (math.hypot(state.u, state.v) <= self.config['speed_tolerance']
                 and abs(state.r) <= self.config['yaw_rate_tolerance'])
        if not quiet:
            self._quiet_since = None
        elif self._quiet_since is None:
            self._quiet_since = state.stamp
        self._last_sample_stamp = state.stamp
        # Source timestamps, not timer ticks: rereading one quiet observation
        # cannot certify that the boat remained stopped for a full second.
        if quiet and state.stamp - self._quiet_since >= self.config['settle_time'] - 1e-9:
            return ControlCommand(status=AlgorithmStatus.GOAL_REACHED, message='STOPPED')
        return braking_candidates(state, self.model, self.config['command_limit'])[-1]
