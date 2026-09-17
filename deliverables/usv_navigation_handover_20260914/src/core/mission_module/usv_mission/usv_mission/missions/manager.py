"""Deterministic task state machine with safety preemption in every phase.

    tick is cheap: it checks current envelopes and enumerates geometry. A* and
    LOS are only evaluated by the existing planning worker. No ROS dependencies.
"""
import math
import numpy as np
from common.math_helper import wrap_angle
from common.models import AlgorithmStatus
from trajectory_planner.circle import CircleReference
from .geometry import entry_candidates, escape_candidates, pose_is_clear, turning_envelope_is_clear
from .models import MissionIntent


class MissionManager:
    def __init__(self, spec, target_pose, now, mission_id='', config=None):
        self.config = dict(config or {})
        self.spec = spec.validate(self.config)
        if len(target_pose) != 3 or not np.isfinite(target_pose).all() or not math.isfinite(now):
            raise ValueError('target_pose and start time must be finite')
        self.target_pose = tuple(float(v) for v in target_pose)
        self.mission_id = str(mission_id)
        self.started_at = self.last_now = float(now)
        self.revision = 0
        self.phase = 'STARTING'
        self.blocked_reason = ''
        self.temporary_goal = None
        self._circle = (CircleReference(self.target_pose[:2], spec.orbit_radius, spec.speed, spec.clockwise)
                        if spec.mode == 'orbit' else None)
        self._returning = False
        self._standby = False
        self._arrival_since = None
        self._last_angle = None
        self._last_angle_stamp = None
        self._net_angle = 0.
        self._safety_active = False
        self._safe_since = None
        self._safe_stamp = None
        self._blocked_map_stamp = None
        self._clock_invalid = False
        self._planner_blocked = False
        self._station_holding = False
        self._departure_heading = None
        self._departure_pose = None
        self._departure_since = None
        self._departure_last_stamp = None
        self._departure_seen = set()
        self._backoff_distance = 0.
        self._backoff_pose = None
        self.intent = MissionIntent('stop', self.phase, self.revision, spec)

    def _emit(self, kind, phase, goal=None, candidates=()):
        if (kind, phase, goal) != (self.intent.kind, self.intent.phase, self.intent.goal):
            self.revision += 1
        self.phase = phase
        self.intent = MissionIntent(kind, phase, self.revision, self.spec, goal, self._circle, tuple(candidates))
        return self.intent

    def _fresh_inputs(self, state, grid, now):
        for value, name, timeout in ((state, 'STATE', self.config.get('navigation.state_timeout', .5)),
                                     (grid, 'MAP', self.config.get('navigation.map_timeout', 1.5))):
            if value is None or not math.isfinite(value.stamp) or not -1e-6 <= now-value.stamp <= timeout:
                return 'WAITING_'+name
        if not state.is_finite():
            return 'WAITING_STATE'
        if state.frame_id != grid.frame_id:
            return 'FRAME_MISMATCH'
        return ''

    def _reset_progress_anchor(self):
        self._last_angle = None
        self._last_angle_stamp = None

    def _clear_departure(self, reset_seen=True):
        self._departure_heading = self._departure_pose = None
        self._departure_since = self._departure_last_stamp = None
        self._backoff_distance = 0.
        self._backoff_pose = None
        if reset_seen:
            self._departure_seen.clear()

    def _prepare_departure(self, state, now):
        """Stop, optionally back off at the stopped heading, then align.

        Worker XY values are geometric evidence, never live control anchors.
        Both the rearward target and the rotation point use measured stops;
        completing the prelude requires a new plan to the original safe goal.
        """
        if self._departure_heading is None:
            return None
        if self._departure_pose is None:
            if self._backoff_pose is None:
                self._departure_since = self._departure_last_stamp = None
                if math.hypot(state.u, state.v) > .04 or abs(state.r) > .02:
                    return self._emit('brake', 'BRAKING_ESCAPE')
                if self._backoff_distance > 0:
                    self._backoff_pose = (
                        float(state.x-self._backoff_distance*math.cos(state.psi)),
                        float(state.y-self._backoff_distance*math.sin(state.psi)), float(state.psi))
            if self._backoff_pose is not None:
                # A .25 m disk would allow turning halfway through a .5 m
                # backoff, before the planned rotation clearance is reached.
                tolerance = min(.05, self.spec.position_tolerance/10.)
                if not self._departure_settled(state, self._backoff_pose, tolerance):
                    return self._emit('hold', 'BACKING_ESCAPE', self._backoff_pose)
                self._backoff_pose = None
                self._departure_since = self._departure_last_stamp = None
            self._departure_pose = (float(state.x), float(state.y), self._departure_heading)
        if self._departure_settled(state, self._departure_pose, min(.25, self.spec.position_tolerance*.5)):
            self._clear_departure(reset_seen=False)
            return None
        return self._emit('hold', 'ALIGN_ESCAPE', self._departure_pose)

    def _departure_settled(self, state, pose, tolerance):
        """Require all pose/velocity constraints across .5 s of fresh odometry."""
        aligned = (self._at_goal(state, pose, tolerance)
                   and abs(wrap_angle(pose[2]-state.psi)) <= math.radians(5.)
                   and math.hypot(state.u, state.v) <= .04 and abs(state.r) <= .02)
        if self._departure_last_stamp is not None and state.stamp < self._departure_last_stamp:
            self._departure_since = self._departure_last_stamp = None
        if not aligned:
            self._departure_since = self._departure_last_stamp = None
        elif self._departure_last_stamp is None or state.stamp > self._departure_last_stamp:
            if (self._departure_since is None or (self._departure_last_stamp is not None
                    and state.stamp-self._departure_last_stamp > self.config.get('navigation.state_timeout', .5))):
                self._departure_since = state.stamp
            self._departure_last_stamp = state.stamp
            if state.stamp-self._departure_since >= .5-1e-9:
                return True
        return False

    def _at_goal(self, state, goal, tolerance=None):
        return math.hypot(state.x-goal[0], state.y-goal[1]) <= (tolerance or self.spec.position_tolerance)

    def _settled_at_center(self, state, now):
        if self._at_goal(state, self.target_pose) and math.hypot(state.u, state.v) <= .1:
            if self._arrival_since is None:
                self._arrival_since = now
            return now-self._arrival_since >= 1.-1e-9
        self._arrival_since = None
        return False

    def _resume_nominal(self):
        self._safety_active = False
        self.temporary_goal = None
        self._safe_since = self._safe_stamp = None
        self._blocked_map_stamp = None
        self._planner_blocked = False
        self._clear_departure()
        self._reset_progress_anchor()

    def _safety(self, state, grid, now):
        if not pose_is_clear(grid, state.pose, self.config):
            self.blocked_reason = 'HULL_COLLISION'
            self._clear_departure()
            self._reset_progress_anchor()
            return self._emit('stop', 'BLOCKED')
        invaded = not pose_is_clear(grid, state.pose, self.config, margin=3.)
        if invaded and not self._safety_active:
            self._safety_active = True
            self._station_holding = False
            self.temporary_goal = None
            self._blocked_map_stamp = None
            self._arrival_since = None
            self._safe_since = self._safe_stamp = None
            self._clear_departure()
        if not self._safety_active:
            return None
        self._reset_progress_anchor()
        if self.temporary_goal is not None:
            # An obstacle can invalidate a previously safe destination.
            if not pose_is_clear(grid, self.temporary_goal, self.config, margin=3.):
                self.temporary_goal = None
                self._safe_since = self._safe_stamp = None
            else:
                departure = self._prepare_departure(state, now)
                if departure is not None:
                    return departure
                if not self._at_goal(state, self.temporary_goal) or math.hypot(state.u, state.v) > .1:
                    return self._emit('escape', 'AVOIDING', self.temporary_goal)
                safe_here = pose_is_clear(grid, state.pose, self.config, margin=5.)
                safe_target = (self._circle is not None and not self._returning) or pose_is_clear(
                    grid, self.target_pose, self.config, margin=5.)
                if not safe_here or not safe_target:
                    self._safe_since = self._safe_stamp = None
                    self.blocked_reason = 'TARGET_OCCUPIED' if not safe_target else 'WAITING_CLEARANCE'
                elif self._safe_stamp is None or grid.stamp > self._safe_stamp:
                    if (self._safe_since is None or (self._safe_stamp is not None
                            and grid.stamp-self._safe_stamp > self.config.get('navigation.map_timeout', 1.5))):
                        self._safe_since = now
                    elif now-self._safe_since >= 2. and grid.stamp != self._safe_stamp:
                        self._resume_nominal()
                        return None
                    self._safe_stamp = grid.stamp
                return self._emit('stop', 'WAITING_CLEARANCE')
            self._clear_departure()
        if self._blocked_map_stamp == grid.stamp:
            self.blocked_reason = 'NO_SAFE_EXIT'
            return self._emit('stop', 'AVOIDANCE_BLOCKED')
        if self._circle is not None and not self._returning:
            candidates = tuple(p for p in entry_candidates(self._circle, state, forward_only=True)
                               if turning_envelope_is_clear(
                                   grid, p, self.config, arrival_tolerance=self.spec.position_tolerance))
            # Preserve the original forward-half choice when it has a clear
            # destination. Mixing the other half into distance/cost selection
            # can favour a short upstream re-entry and repeatedly meet the same
            # obstacle. The remaining half is only a geometric-empty fallback.
            cx, cy = self._circle.center
            theta = math.atan2(state.y-cy, state.x-cx)
            primary = tuple(p for p in candidates if (
                self._circle.direction*(math.atan2(p[1]-cy, p[0]-cx)-theta)
                ) % (2*math.pi) <= math.pi+1e-9)
            if primary:
                candidates = primary
        else:
            candidates = escape_candidates(state, grid, self.config,
                                           arrival_tolerance=self.spec.position_tolerance)
        if not candidates:
            self._blocked_map_stamp = grid.stamp
            self.blocked_reason = 'NO_SAFE_EXIT'
            return self._emit('stop', 'AVOIDANCE_BLOCKED')
        return self._emit('escape', 'AVOIDING', candidates=candidates)

    def tick(self, state, grid, now):
        """Update once per control/planning tick using simulation time."""
        if not math.isfinite(now) or now < self.last_now-1e-6:
            self._clock_invalid = True
        if self._clock_invalid:
            self.blocked_reason = 'CLOCK_REWIND'
            self._clear_departure()
            return self._emit('stop', 'BLOCKED')
        self.last_now = now
        self.blocked_reason = self._fresh_inputs(state, grid, now)
        if self.blocked_reason:
            self._arrival_since = self._safe_since = self._safe_stamp = None
            self._departure_since = self._departure_last_stamp = None
            if self._backoff_distance > 0:
                # After an observation gap neither the rearward anchor nor
                # its conditional continuation is trusted. Select afresh.
                self._clear_departure()
                self.temporary_goal = None
                self._blocked_map_stamp = None
            self._reset_progress_anchor()
            return self._emit('stop', 'WAITING_INPUT')
        if self._circle is not None and self.spec.duration and now-self.started_at >= self.spec.duration:
            self._returning = True
        safety = self._safety(state, grid, now)
        if safety is not None:
            return safety
        if self._planner_blocked:
            if self._blocked_map_stamp == grid.stamp:
                self.blocked_reason = 'NO_PATH'
                return self._emit('stop', 'BLOCKED')
            self._planner_blocked = False
        if self.spec.mode == 'station_keep':
            if not self._station_holding:
                if not self._at_goal(state, self.target_pose, max(2., 4*self.spec.position_tolerance)):
                    return self._emit('navigate', 'APPROACH_STATION', self.target_pose)
                self._station_holding = True
            return self._emit('hold', 'STATION_KEEP', self.target_pose)
        if self.spec.mode == 'standby' or self._returning:
            if self._standby and not self._at_goal(state, self.target_pose, self.spec.watch_radius):
                self._standby = False
                self._arrival_since = None
            if not self._standby and self._settled_at_center(state, now):
                self._standby = True
            if self._standby:
                return self._emit('stop', 'STANDBY')
            return self._emit('navigate', 'RETURN_CENTER', self.target_pose)
        delta = (state.x-self.target_pose[0], state.y-self.target_pose[1])
        angle, radius = math.atan2(delta[1], delta[0]), math.hypot(*delta)
        tangent_yaw = self._circle.pose_at(angle)[2]
        aligned = abs(wrap_angle(tangent_yaw-state.psi)) <= math.radians(15.)
        in_band = abs(radius-self.spec.orbit_radius) <= self.spec.position_tolerance
        if in_band and (aligned or self.phase == 'ORBITING'):
            # Net signed displacement cancels back-and-forth jitter. Never
            # stitch a skipped arc across an excursion or a safety preemption.
            if self._last_angle is not None:
                increment = wrap_angle(angle-self._last_angle)
                elapsed = state.stamp-self._last_angle_stamp
                # Position error at both endpoints may contribute to measured
                # arc length. Larger-than-physical jumps are reanchored, never
                # counted as successful patrol. Repeated state stamps add zero.
                allowed = self.config.get('dynamics.velocity_max', 1.)*max(0., elapsed)+2*self.spec.position_tolerance
                if elapsed > 0 and abs(increment)*self.spec.orbit_radius <= allowed:
                    self._net_angle += self._circle.direction*increment
            self._last_angle = angle
            self._last_angle_stamp = state.stamp
            self.temporary_goal = None
            if self.spec.laps and self._net_angle >= self.spec.laps*2*math.pi-1e-8:
                self._returning = True
                return self._emit('navigate', 'RETURN_CENTER', self.target_pose)
            return self._emit('circle', 'ORBITING')
        self._reset_progress_anchor()
        if self.phase == 'ORBITING':
            # The vector field restores radial excursions, without replacing
            # the original circle or turning a temporary deviation into entry.
            return self._emit('circle', 'ORBITING')
        if in_band and math.hypot(state.u, state.v) <= .1:
            if self.temporary_goal is None:
                self.temporary_goal = self._circle.pose_at(angle)
            return self._emit('hold', 'ALIGN_ENTRY', self.temporary_goal)
        if self.temporary_goal is not None:
            if self._at_goal(state, self.temporary_goal) and math.hypot(state.u, state.v) <= .1:
                return self._emit('hold', 'ALIGN_ENTRY', self.temporary_goal)
            return self._emit('orbit_entry', 'ENTERING_ORBIT', self.temporary_goal)
        return self._emit('orbit_entry', 'ENTERING_ORBIT', candidates=entry_candidates(self._circle, state))

    def on_plan(self, response):
        """Adapter calls this only for a matching reference/mission version."""
        if response.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            selected = getattr(response, 'selected_goal', None)
            if selected is not None and self.intent.kind in ('escape', 'orbit_entry'):
                if self.temporary_goal != tuple(selected):
                    self._clear_departure()
                self.temporary_goal = tuple(selected)
            metadata = getattr(response, 'metadata', None) or {}
            departure = metadata.get('departure_pose')
            backoff = metadata.get('backoff_distance', 0.)
            if (isinstance(backoff, bool) or not isinstance(backoff, (int, float))
                    or not math.isfinite(backoff) or not 0. <= backoff <= 3.
                    or (backoff > 0 and departure is None)):
                self._clear_departure()
                self.temporary_goal = None
                self.blocked_reason = 'INVALID_BACKOFF_DISTANCE'
                self._blocked_map_stamp = metadata.get('map_stamp')
                return
            if departure is not None and self.intent.kind == 'escape' and self.temporary_goal is not None:
                try:
                    pose = tuple(float(v) for v in departure)
                    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                        raise ValueError('Invalid departure pose')
                except (ValueError, TypeError, OverflowError):
                    self._clear_departure()
                    self.temporary_goal = None
                    self.blocked_reason = 'INVALID_DEPARTURE_POSE'
                    self._blocked_map_stamp = (getattr(response, 'metadata', None) or {}).get('map_stamp')
                    return
                heading = wrap_angle(pose[2])
                key = (self.temporary_goal, round(heading, 9), float(backoff))
                if key not in self._departure_seen and self._departure_heading is None:
                    self._departure_seen.add(key)
                    self._departure_heading = heading
                    self._departure_pose = None
                    self._departure_since = self._departure_last_stamp = None
                    self._backoff_distance = float(backoff)
                    self._backoff_pose = None
            self._planner_blocked = False
            return
        self.blocked_reason = response.message or 'NO_PATH'
        if self.intent.kind == 'circle':
            self.request_escape(self.blocked_reason)
        elif self.intent.kind == 'escape' or self._departure_heading is not None:
            self._blocked_map_stamp = getattr(response, 'metadata', {}).get('map_stamp')
            self.temporary_goal = None
            self._clear_departure()
        else:
            self._planner_blocked = True
            self._blocked_map_stamp = getattr(response, 'metadata', {}).get('map_stamp')

    def request_escape(self, reason='unsafe predicted motion'):
        """Safety controller can preempt a hold before near-field intrusion."""
        self._safety_active = True
        self._station_holding = False
        self.temporary_goal = None
        self._blocked_map_stamp = None
        self._arrival_since = self._safe_since = self._safe_stamp = None
        self._clear_departure()
        self.blocked_reason = str(reason)
        self._reset_progress_anchor()

    def on_arrival(self, now):
        """Completion is determined by measured state in tick, not PID disks."""
        return self.intent

    def status(self, state, now):
        error = (math.hypot(state.x-self.target_pose[0], state.y-self.target_pose[1])
                 if state is not None and state.is_finite() else None)
        return {
            'mission_id': self.mission_id, 'mode': self.spec.mode, 'phase': self.phase,
            'revision': self.revision,
            'original_target': {'latitude': self.spec.latitude, 'longitude': self.spec.longitude,
                                'heading_deg': self.spec.heading_deg, 'local_pose': self.target_pose},
            'temporary_goal': self.temporary_goal, 'position_error': error,
            'departure_heading': self._departure_heading,
            'departure_pose': self._departure_pose,
            'backoff_distance': self._backoff_distance,
            'backoff_pose': self._backoff_pose,
            'heading_error_deg': (math.degrees(wrap_angle(self.target_pose[2]-state.psi))
                                  if state is not None and state.is_finite() else None),
            'radial_error': (error-self.spec.orbit_radius if error is not None and self._circle else None),
            'completed_laps': max(0., self._net_angle/(2*math.pi)),
            'elapsed_time': max(0., now-self.started_at),
            'inside_watch_region': error <= self.spec.watch_radius if error is not None else None,
            'blocked_reason': self.blocked_reason,
            'orbit_completed': self._returning if self._circle else False,
        }
