"""Fail-closed, latched guard for this task node's actuator publication rights.

ROS graph observation is a defence against accidentally running two controllers;
it is not a distributed actuator lease. The launch composition must already have
one writer. Once a second writer is observed, only restarting can clear the latch.
"""
from dataclasses import dataclass, field


@dataclass
class ControlAuthority:
    conflicted: bool = False
    reason: str = ''
    publishers: dict = field(default_factory=dict)

    def observe(self, publishers):
        """Allow output only if every actual (remapped) topic has one writer."""
        if self.conflicted:
            return False
        self.publishers = dict(publishers)
        competing = [name for name, count in self.publishers.items() if count > 1]
        if competing:
            self.conflicted = True
            self.reason = ('Multiple control publishers on ' + ', '.join(sorted(competing))
                           + '; no commands will be published. Stop the competing controller '
                             'and restart mission_manager to restore control authority.')
            return False
        if not self.publishers or any(count != 1 for count in self.publishers.values()):
            self.reason = 'Waiting for exclusive publisher discovery on every actuator topic'
            return False
        self.reason = ''
        return True

    def unavailable(self, error):
        """Graph errors inhibit output without silently clearing a real conflict."""
        if not self.conflicted:
            self.reason = 'Cannot verify actuator publisher ownership: ' + str(error)
        return False

    def status(self):
        return {'conflicted': self.conflicted, 'reason': self.reason,
                'publishers': dict(self.publishers), 'restart_required': self.conflicted}
