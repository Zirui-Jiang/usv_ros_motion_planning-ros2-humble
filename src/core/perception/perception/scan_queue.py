"""Bounded scan waiting and clock handling, kept testable without ROS."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


@dataclass(frozen=True)
class PendingScan:
    payload: object
    frame_id: str
    stamp_ns: int
    received_at: float


class ScanResult(Enum):
    WAITING = 'waiting'
    PROCESSED = 'processed'
    EXPIRED = 'expired'
    INVALID = 'invalid'


@dataclass
class QueueCounters:
    accepted: int = 0
    processed: int = 0
    overflow: int = 0
    expired: int = 0
    duplicate: int = 0
    invalid: int = 0
    invalid_consumed: int = 0
    resets: int = 0


class ScanQueue:
    def __init__(self, capacity=5, tf_wait_timeout=0.2):
        if capacity < 1 or not math.isfinite(tf_wait_timeout) or tf_wait_timeout <= 0:
            raise ValueError("queue capacity and TF waiting timeout must be positive")
        self.capacity = capacity
        self.tf_wait_timeout = tf_wait_timeout
        self.counters = QueueCounters()
        self._pending = deque()
        self._last_stamp_by_frame = {}
        # Diagnostic observations do not participate in admission or expiry.
        self._observations = deque()
        self.observation_window_s = 5.0

    def __len__(self):
        return len(self._pending)

    def enqueue(self, payload, *, frame_id, stamp_ns, received_at):
        # A zero ROS Time requests 'latest TF'; rejecting it avoids silently
        # projecting an unstamped scan using a different sensor pose.
        if not frame_id or stamp_ns <= 0 or not math.isfinite(received_at):
            self.counters.invalid += 1
            return False
        if stamp_ns <= self._last_stamp_by_frame.get(frame_id, -1):
            self.counters.duplicate += 1
            return False
        self._last_stamp_by_frame[frame_id] = stamp_ns
        if len(self._pending) >= self.capacity:
            self._pending.popleft()
            self.counters.overflow += 1
        self._pending.append(PendingScan(payload, frame_id, stamp_ns, received_at))
        self.counters.accepted += 1
        return True

    def drain(self, now, process):
        """Process in arrival order; count only successful map integrations.

        ``now`` is monotonic wall time or a callable returning it. A callable
        rechecks the deadline between scans, including time spent processing.
        An expired or displaced scan can never be re-enqueued in this epoch.
        The callback returns ScanResult; legacy False/True mean WAITING/PROCESSED.
        EXPIRED is a post-TF admission rejection; INVALID consumes malformed input.
        """
        completed = 0
        while self._pending:
            pending = self._pending[0]
            current = now() if callable(now) else now
            if current - pending.received_at >= self.tf_wait_timeout:
                self._pending.popleft()
                self.counters.expired += 1
                continue
            result = process(pending)
            if result is False:
                result = ScanResult.WAITING
            elif result is True:
                result = ScanResult.PROCESSED
            if result == ScanResult.WAITING:
                # A nonblocking TF lookup can still cross the deadline while
                # executing or being descheduled. Do not defer this expiry.
                current = now() if callable(now) else now
                if current - pending.received_at >= self.tf_wait_timeout:
                    self._pending.popleft()
                    self.counters.expired += 1
                    continue
                break
            if result not in (ScanResult.PROCESSED, ScanResult.EXPIRED, ScanResult.INVALID):
                raise ValueError('scan callback must return ScanResult or bool')
            self._pending.popleft()
            if result == ScanResult.EXPIRED:
                self.counters.expired += 1
            elif result == ScanResult.INVALID:
                self.counters.invalid_consumed += 1
            else:
                self.counters.processed += 1
                completed += 1
        return completed

    def expire_pending(self, now):
        """Discard expired entries at one explicit monotonic checkpoint.

        A caller may then observe_pending(now) with this same timestamp. This
        is actual queue maintenance, not a diagnostic clamp or an extra retry.
        """
        if not math.isfinite(now):
            raise ValueError('expiry time must be finite monotonic wall time')
        retained = deque(scan for scan in self._pending
                         if now - scan.received_at < self.tf_wait_timeout)
        expired = len(self._pending) - len(retained)
        self._pending = retained
        self.counters.expired += expired
        return expired

    def observe_pending(self, now):
        """Record queue length and oldest wall age after a completed drain.

        Sampling is explicit so diagnostics never add clock reads between
        deadline checks or alter which scans the drain processes.
        """
        if not math.isfinite(now):
            raise ValueError("observation time must be finite monotonic wall time")
        if self._observations and now < self._observations[-1][0]:
            raise ValueError("observation time must not move backwards")
        oldest = min(self._pending, key=lambda scan: scan.received_at) if self._pending else None
        oldest_ms = max(0.0, now - oldest.received_at) * 1000.0 if oldest else None
        identity = (oldest.frame_id, oldest.stamp_ns, oldest.received_at) if oldest else None
        self._observations.append((now, len(self._pending), oldest_ms, identity))
        self._prune_observations(now)

    def _prune_observations(self, now):
        while self._observations and self._observations[0][0] < now - self.observation_window_s:
            self._observations.popleft()

    def observation_summary(self, now):
        """Describe only actual post-drain observations in the last five seconds.

        Empty queues contribute length zero but no pending age. Missing samples
        therefore remain distinguishable from an observed empty queue.
        """
        if not math.isfinite(now):
            raise ValueError("observation time must be finite monotonic wall time")
        if self._observations and now < self._observations[-1][0]:
            raise ValueError("summary time must not precede the latest observation")
        self._prune_observations(now)
        observations = self._observations
        ages = [row[2] for row in observations if row[2] is not None]
        identities = [row[3] for row in observations if row[3] is not None]
        gaps_ms = [(b[0] - a[0]) * 1000.0 for a, b in zip(observations, list(observations)[1:])]
        latest_identity = observations[-1][3] if observations else None
        age_summary = {'count': len(ages)}
        if ages:
            age_summary.update({name: float(value) for name, value in zip(
                ('p50', 'p95', 'p99'), np.percentile(ages, [50, 95, 99]))})
            age_summary['max'] = max(ages)
        return {
            'window_s': self.observation_window_s,
            'sampling_phase': 'after_drain',
            'capacity': self.capacity,
            'tf_wait_timeout_s': self.tf_wait_timeout,
            'observation_count': len(observations),
            'pending_observations': len(ages),
            'empty_observations': len(observations) - len(ages),
            'max_queue_size': max((row[1] for row in observations), default=None),
            'latest_queue_size': observations[-1][1] if observations else None,
            'latest_oldest_pending_age_ms': observations[-1][2] if observations else None,
            'oldest_pending_age_ms': age_summary,
            'oldest_pending_distinct_count': len(set(identities)),
            'latest_oldest_pending': (dict(zip(('frame_id', 'stamp_ns', 'received_at_monotonic_s'),
                                               latest_identity)) if latest_identity else None),
            'max_observation_gap_ms': max(gaps_ms, default=None),
            'observed_span_s': observations[-1][0] - observations[0][0] if observations else 0.0,
            'first_observed_monotonic_s': observations[0][0] if observations else None,
            'last_observed_monotonic_s': observations[-1][0] if observations else None,
        }

    def reset(self):
        self._pending.clear()
        self._last_stamp_by_frame.clear()
        self._observations.clear()
        self.counters.resets += 1


class ClockGuard:
    def __init__(self):
        self._previous = None

    def observe(self, stamp_ns):
        rollback = self._previous is not None and stamp_ns < self._previous
        self._previous = stamp_ns
        return rollback
