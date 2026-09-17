"""ROS-independent liveness and retired process identity bookkeeping."""
from dataclasses import dataclass, field
import math

HEARTBEAT_PERIOD = 0.2
PEER_TIMEOUT = 1.5


@dataclass
class PeerSession:
    session: str = ''
    sequence: int = -1
    last_seen: float = float('-inf')
    ready: bool = False
    retired: set = field(default_factory=set)

    def observe(self, session, sequence, now, ready=True):
        if (not session or session in self.retired or not math.isfinite(now)
                or sequence < 0 or (session == self.session and sequence <= self.sequence)):
            return 'ignored'
        change = 'update'
        if session != self.session:
            change = 'restart' if self.session else 'new'
            if self.session:
                self.retired.add(self.session)
            self.session = session
        self.sequence, self.last_seen, self.ready = sequence, now, ready
        return change

    def fresh(self, now, timeout=PEER_TIMEOUT):
        return bool(self.session) and math.isfinite(now) and 0 <= now - self.last_seen <= timeout
