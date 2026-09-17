"""Execution history: each 0211 receipt is new; internal retries reuse its ID."""
from collections import OrderedDict
from dataclasses import dataclass
from uuid import uuid4


@dataclass
class LegRecord:
    leg: object
    request_id: str
    state: str = 'PENDING'
    reason: str = ''
    planner_session: str = ''
    task_id: int = -1
    generation: int = 0
    timeout_retries: int = 0
    # Arrival feedback is a one-shot milestone, separate from task completion.
    arrival_confirmed: bool = False


class LegBook:
    def __init__(self, capacity=128, max_timeout_retries=3):
        if capacity < 1:
            raise ValueError('History capacity must be positive')
        if max_timeout_retries < 0:
            raise ValueError('Timeout retry limit must not be negative')
        self.capacity = capacity
        self.max_timeout_retries = max_timeout_retries
        self.records = OrderedDict()
        self.active = None

    def receive(self, leg, planner_session=''):
        if self.active is not None:
            return 'busy', self.active
        record = self.prepare(leg)
        self.activate(record, planner_session)
        return 'submit', record

    def prepare(self, leg):
        # travelId identifies the route. Even identical route content received
        # again means another execution under the sender's confirmed contract.
        return LegRecord(leg, 'ladds-' + uuid4().hex)

    def activate(self, record, planner_session=''):
        if self.active is not None:
            raise ValueError('A navigation execution is still active')
        if record.state != 'PENDING' or record.request_id in self.records:
            raise ValueError('Execution was already submitted or ended')
        record.planner_session = planner_session
        self.records[record.request_id] = record
        while len(self.records) > self.capacity:
            self.records.popitem(last=False)
        self.active = record
        return record

    def abort(self, record, reason):
        if record is self.active:
            record.state, record.reason = 'ABORTED', reason
            self.active = None

    def defer_busy(self, record):
        # A BUSY response explicitly confirms this request was not accepted.
        # After cancelling the competing goal it is safe to resubmit this ID.
        if (self.active is not None or record.state != 'REJECTED'
                or record.task_id >= 0 or not record.reason.upper().startswith('BUSY')):
            return False
        self.records.pop(record.request_id, None)
        record.state, record.reason = 'PENDING', ''
        return True

    def begin_submission(self, request_id):
        record = self.active
        if record is None or record.request_id != request_id:
            raise ValueError('Navigation request is no longer active')
        if record.state not in {'PENDING', 'RETRY_PENDING'}:
            raise ValueError(f'Navigation request cannot submit from {record.state}')
        record.generation += 1
        record.state = 'SUBMITTING'
        return record.generation

    def acknowledge(self, request_id, accepted, planner_session, task_id, reason,
                    generation=None):
        record = self.active
        if record is None or record.request_id != request_id:
            return
        if generation is not None and generation != record.generation:
            return
        if record.planner_session and planner_session != record.planner_session:
            if not accepted:
                record.state = 'ABORTED'
                record.reason = 'Decision session changed while submitting navigation'
                self.active = None
                return
            # A new server may accept this in-flight request before its first
            # status arrives. Preserve that explicit acceptance so its eventual
            # arrival remains correlated; the node filters retired sessions.
            record.planner_session = planner_session
            record.task_id = -1
        if not record.planner_session:
            record.planner_session = planner_session
        record.reason = reason
        if not accepted:
            record.state, self.active = 'REJECTED', None
            return
        if record.task_id >= 0 and record.task_id != task_id:
            record.state = 'ABORTED'
            record.reason = 'Navigation service and status reported different task IDs'
            self.active = None
            return
        record.planner_session, record.task_id = planner_session, task_id
        if not record.arrival_confirmed:
            record.state = 'ACTIVE'

    def uncertain(self, request_id, reason):
        if self.active is not None and self.active.request_id == request_id:
            self.active.state, self.active.reason = 'SUBMIT_UNKNOWN', reason

    def submission_timed_out(self, request_id, generation):
        return self._submission_failed(
            request_id, generation, 'Navigation service reply timed out')

    def submission_failed(self, request_id, generation, reason):
        return self._submission_failed(
            request_id, generation, f'Navigation service call failed: {reason}')

    def _submission_failed(self, request_id, generation, reason):
        record = self.active
        if (record is None or record.request_id != request_id
                or record.generation != generation or record.state != 'SUBMITTING'):
            return 'stale', record
        if record.timeout_retries < self.max_timeout_retries:
            record.timeout_retries += 1
            record.state = 'RETRY_PENDING'
            record.reason = (
                f'{reason}; retry {record.timeout_retries}/'
                f'{self.max_timeout_retries} awaits fresh navigation status')
            return 'retry', record
        record.state = 'SUBMIT_UNKNOWN'
        record.reason = (
            f'{reason} after {self.max_timeout_retries} retries; '
            'confirm navigation is cancelled, then restart this adapter or the decision node')
        return 'unknown', record

    def observe(self, status):
        """Return a newly confirmed arrival; guarding remains the active task."""
        record = self.active
        if record is None or not isinstance(status, dict):
            return None
        modules = status.get('modules')
        if not isinstance(modules, dict):
            return None
        session, task = modules.get('decision'), status.get('task_id')
        if not isinstance(session, str) or not session or type(task) is not int or task < 0:
            return None
        request = status.get('request_id')
        if record.planner_session and session != record.planner_session:
            record.state, record.reason = 'ABORTED', 'Navigation decision session changed'
            self.active = None
            return None
        if not record.planner_session:
            if request != record.request_id:
                return None
            record.planner_session, record.task_id = session, task
        current_task = status.get('current_task_id', task)
        if type(current_task) is not int or current_task < 0:
            return None
        if session == record.planner_session and current_task < record.task_id:
            return None  # Late status from before our service request.
        if request != record.request_id:
            if record.task_id < 0:
                return None  # Status published before this request was accepted.
            record.state, record.reason = 'ABORTED', 'Navigation was restarted or replaced'
            self.active = None
            return None
        if record.task_id < 0:
            record.task_id = task
        elif task != record.task_id:
            return None
        state = status.get('state')
        if state == 'GOAL_REACHED':
            if getattr(record.leg, 'guard_on_arrival', False):
                # Require confirmed guard takeover, not an old controller's
                # ordinary completion report that would end the hold task.
                record.state, record.reason = 'ACTIVE', 'Waiting for terminal position guard'
                return None
            if task != record.task_id:
                return None
            record.state, record.reason = 'COMPLETED', 'Navigation confirmed arrival and stop'
            record.arrival_confirmed = True
            self.active = None
            return record
        if state in {'CANCELLED', 'MODULE_RESTART', 'CLOCK_RESET', 'SHUTDOWN', 'INVALID_GOAL', 'STOPPING', 'STOPPED'}:
            record.state, record.reason = 'ABORTED', str(state)
            self.active = None
        elif state == 'GUARDING':
            record.state, record.reason = 'GUARDING', str(status.get('reason', ''))
        elif record.state != 'SUBMITTING':
            record.state, record.reason = 'ACTIVE', str(status.get('reason', ''))
        # HOLD is latched by decision only after terminal arrival. It survives
        # brief safety waits, so a missed first GUARDING status cannot lose 0052.
        if (self.active is record and getattr(record.leg, 'guard_on_arrival', False)
                and not record.arrival_confirmed
                and (state == 'GUARDING' or status.get('motion_mode') == 'HOLD')):
            record.arrival_confirmed = True
            return record
        return None
