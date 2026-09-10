"""Bounded scan waiting and clock handling, kept testable without ROS."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PendingScan:
    payload: object
    frame_id: str
    stamp_ns: int
    received_at: float


@dataclass
class QueueCounters:
    accepted: int = 0
    processed: int = 0
    overflow: int = 0
    expired: int = 0
    duplicate: int = 0
    invalid: int = 0
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
        """Process in arrival order; False means its exact-time TF is pending.

        ``now`` is monotonic wall time or a callable returning it. A callable
        rechecks the deadline between scans, including time spent processing.
        An expired or displaced scan can never be re-enqueued in this epoch.
        """
        completed = 0
        while self._pending:
            pending = self._pending[0]
            current = now() if callable(now) else now
            if current - pending.received_at >= self.tf_wait_timeout:
                self._pending.popleft()
                self.counters.expired += 1
                continue
            if not process(pending):
                break
            self._pending.popleft()
            self.counters.processed += 1
            completed += 1
        return completed

    def reset(self):
        self._pending.clear()
        self._last_stamp_by_frame.clear()
        self.counters.resets += 1


class ClockGuard:
    def __init__(self):
        self._previous = None

    def observe(self, stamp_ns):
        rollback = self._previous is not None and stamp_ns < self._previous
        self._previous = stamp_ns
        return rollback
