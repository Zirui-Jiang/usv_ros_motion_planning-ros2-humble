"""Each received route starts a distinct execution; service retries reuse it."""
from types import SimpleNamespace

import pytest

from usv_ladds_bridge.leg_state import LegBook


def leg(travel=53, fingerprint='route-a'):
    return SimpleNamespace(identity=(1000, travel), fingerprint=fingerprint,
                           travel_id=travel, x=83.38367675, y=91.68293913,
                           speed_mps=4 * 1852 / 3600)


def status(record, state='NAVIGATING', task=7, session='planner-a', request=None):
    return dict(state=state, task_id=task,
                request_id=record.request_id if request is None else request,
                modules={'decision': session})


def test_active_execution_is_not_silently_reused_for_another_received_command():
    book = LegBook()
    action, record = book.receive(leg())
    assert action == 'submit'
    assert book.receive(leg())[0] == 'busy'
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    assert book.receive(leg())[0] == 'busy'
    assert book.receive(leg(54))[0] == 'busy'
    assert book.receive(leg(fingerprint='route-b'))[0] == 'busy'


def test_only_matching_actual_completion_generates_feedback():
    book = LegBook()
    _, record = book.receive(leg())
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    assert book.observe(status(record, task=6, request='old')) is None
    assert book.observe(status(record, state='NAVIGATING')) is None
    assert book.observe(status(record, state='SETTLING')) is None
    assert book.observe(status(record, state='GOAL_REACHED')) is record
    assert record.state == 'COMPLETED'
    assert book.active is None
    assert book.observe(status(record, state='GOAL_REACHED')) is None
    action, repeated = book.receive(leg())
    assert action == 'submit'
    assert repeated.request_id != record.request_id
    assert repeated.leg.travel_id == record.leg.travel_id
    assert repeated.state == 'PENDING'


def test_guard_arrival_returns_feedback_once_without_completing_the_task():
    book = LegBook()
    guarded = leg()
    guarded.guard_on_arrival = True
    _, record = book.receive(guarded)
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    assert book.observe(status(record, state='GOAL_REACHED')) is None
    assert book.active is record
    assert book.observe(status(record, state='GUARDING')) is record
    assert record.state == 'GUARDING'
    assert book.active is record
    assert book.observe(status(record, state='GUARDING')) is None
    book.acknowledge(record.request_id, True, 'planner-a', 7, 'late acceptance')
    assert record.state == 'GUARDING'
    assert book.observe(status(record, state='GUARDING')) is None
    assert book.observe(status(record, state='STOPPING')) is None
    assert record.state == 'ABORTED'
    assert book.observe(status(record, state='GUARDING')) is None


@pytest.mark.parametrize('control_state', ['GUARD_BRAKING', 'GUARD_BLOCKED', 'WAITING_MAP'])
def test_hold_mode_remembers_arrival_even_if_first_guarding_status_was_missed(control_state):
    book = LegBook()
    guarded = leg()
    guarded.guard_on_arrival = True
    _, record = book.receive(guarded)
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    report = status(record, state=control_state)
    report['motion_mode'] = 'HOLD'
    assert book.observe(report) is record
    assert book.active is record
    assert book.observe(report) is None
    assert book.observe(status(record, state='GUARDING')) is None
    assert book.active is record


def test_ordinary_route_cannot_report_arrival_from_a_guarding_status():
    book = LegBook()
    _, record = book.receive(leg())
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    assert book.observe(status(record, state='GUARDING')) is None
    assert book.active is record


@pytest.mark.parametrize('changed', [dict(task=6), dict(task=8),
                                    dict(request='another-request'),
                                    dict(session='another-session'), dict(state='CANCELLED')])
def test_guard_arrival_requires_matching_live_execution(changed):
    book = LegBook()
    guarded = leg()
    guarded.guard_on_arrival = True
    _, record = book.receive(guarded)
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    report = status(record, **{'state': 'GUARDING', **changed})
    report['motion_mode'] = 'HOLD'
    assert book.observe(report) is None


def test_cancel_replacement_and_restart_cannot_finish_original_task():
    for changed in [dict(state='CANCELLED'), dict(session='planner-b'),
                    dict(request='manual-goal', task=8), dict(state='CLOCK_RESET')]:
        book = LegBook()
        _, record = book.receive(leg())
        book.acknowledge(record.request_id, True, 'planner-a', 7, '')
        assert book.observe(status(record, **changed)) is None
        assert record.state == 'ABORTED'
        assert book.active is None
        action, repeated = book.receive(leg())
        assert action == 'submit'
        assert repeated.request_id != record.request_id


def test_service_failure_or_timeout_never_fabricates_completion():
    book = LegBook()
    _, record = book.receive(leg())
    book.uncertain(record.request_id, 'reply timeout')
    assert record.state == 'SUBMIT_UNKNOWN'
    assert book.receive(leg())[0] == 'busy'
    # A late, correlated status may recover an accepted request.
    assert book.observe(status(record, state='GOAL_REACHED')) is record
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    assert record.state == 'COMPLETED'
    _, rejected = book.receive(leg(54))
    book.acknowledge(rejected.request_id, False, 'planner-a', 8, 'speed exceeds limit')
    assert rejected.state == 'REJECTED'
    assert book.active is None
    action, repeated = book.receive(leg(54))
    assert action == 'submit'
    assert repeated.request_id != rejected.request_id


def test_adapter_restart_does_not_reuse_an_old_execution_id_and_history_is_bounded():
    a, b = LegBook(capacity=2), LegBook(capacity=2)
    assert a.receive(leg())[1].request_id != b.receive(leg())[1].request_id
    a.acknowledge(a.active.request_id, False, '', 0, 'rejected')
    request_ids = set()
    for _ in range(3):
        _, record = a.receive(leg())
        request_ids.add(record.request_id)
        a.acknowledge(record.request_id, False, '', 0, 'rejected')
    assert len(request_ids) == 3
    assert len(a.records) == 2


def test_same_route_can_be_received_with_updated_coordinates_and_speed():
    book = LegBook()
    _, previous = book.receive(leg())
    book.acknowledge(previous.request_id, True, 'planner-a', 7, '')
    book.observe(status(previous, state='GOAL_REACHED'))
    changed = leg(fingerprint='route-b')
    changed.x, changed.speed_mps = 20.0, 1.0

    action, current = book.receive(changed)

    assert action == 'submit'
    assert current.request_id != previous.request_id
    assert current.leg.travel_id == previous.leg.travel_id
    assert current.leg.x == 20.0
    assert current.leg.speed_mps == 1.0


def test_manual_goal_replacement_uses_internal_counter_even_without_external_id():
    book = LegBook()
    _, record = book.receive(leg())
    book.acknowledge(record.request_id, True, 'planner-a', 7, '')
    replaced = status(record, request='', task=0)
    replaced['current_task_id'] = 8
    assert book.observe(replaced) is None
    assert record.state == 'ABORTED'


def test_busy_rejection_does_not_wait_for_a_repeated_dds_command_to_retry():
    book = LegBook()
    action, record = book.receive(leg(), planner_session='planner-a')
    assert action == 'submit'
    first_generation = book.begin_submission(record.request_id)

    book.acknowledge(
        record.request_id, False, 'planner-a', 0,
        'BUSY: an active navigation goal already exists', first_generation)

    assert record.state == 'REJECTED'
    assert book.active is None
    action, repeated = book.receive(leg(), planner_session='planner-a')
    assert action == 'submit'
    assert repeated.request_id != record.request_id


def test_submission_timeout_has_three_retries_then_stays_unknown_and_blocks_takeover():
    book = LegBook(max_timeout_retries=3)
    _, record = book.receive(leg(), planner_session='planner-a')
    generation = book.begin_submission(record.request_id)

    for retry_number in (1, 2, 3):
        action, pending = book.submission_timed_out(record.request_id, generation)
        assert action == 'retry'
        assert pending.state == 'RETRY_PENDING'
        assert pending.timeout_retries == retry_number
        generation = book.begin_submission(record.request_id)

    action, unknown = book.submission_timed_out(record.request_id, generation)
    assert action == 'unknown'
    assert unknown.state == 'SUBMIT_UNKNOWN'
    assert 'confirm navigation is cancelled' in unknown.reason.lower()
    assert 'restart' in unknown.reason.lower()
    assert book.receive(leg())[0] == 'busy'
    assert book.receive(leg(54))[0] == 'busy'


def test_decision_session_change_aborts_old_task_without_retrying_it():
    book = LegBook()
    _, record = book.receive(leg(), planner_session='planner-a')
    generation = book.begin_submission(record.request_id)
    book.submission_timed_out(record.request_id, generation)

    restarted = status(record, state='IDLE', task=0, session='planner-b', request='')
    restarted['current_task_id'] = 0
    assert book.observe(restarted) is None

    assert record.state == 'ABORTED'
    assert book.active is None
    action, repeated = book.receive(leg())
    assert action == 'submit'
    assert repeated.request_id != record.request_id


def test_late_service_reply_cannot_override_completed_or_cancelled_record():
    for terminal_status, terminal_state in [('GOAL_REACHED', 'COMPLETED'),
                                            ('CANCELLED', 'ABORTED')]:
        book = LegBook()
        _, record = book.receive(leg(), planner_session='planner-a')
        generation = book.begin_submission(record.request_id)
        book.acknowledge(record.request_id, True, 'planner-a', 7, '', generation)
        book.observe(status(record, state=terminal_status))

        book.acknowledge(
            record.request_id, False, 'planner-a', 0,
            'late rejection', generation)

        assert record.state == terminal_state
        assert book.active is None


def test_prepare_waits_for_previous_cancellation_before_activation():
    book = LegBook(capacity=1)
    _, previous = book.receive(leg(), planner_session='planner-a')
    waiting = book.prepare(leg())
    assert waiting.request_id != previous.request_id
    assert book.active is previous
    with pytest.raises(ValueError):
        book.activate(waiting, 'planner-a')

    book.abort(previous, 'Superseded after confirmed cancellation')
    book.activate(waiting, 'planner-a')

    assert previous.state == 'ABORTED'
    assert book.active is waiting
    assert waiting.planner_session == 'planner-a'
    assert len(book.records) == 1
    # A stale cancellation callback must not clear the new execution.
    book.abort(previous, 'Late cancellation')
    assert book.active is waiting


def test_busy_before_acceptance_can_wait_for_cancellation_without_changing_execution_id():
    book = LegBook()
    _, record = book.receive(leg(), planner_session='planner-a')
    book.begin_submission(record.request_id)
    book.acknowledge(record.request_id, False, 'planner-a', 0, 'BUSY: GUI goal active')
    original_id = record.request_id

    assert book.defer_busy(record)
    book.activate(record, 'planner-a')
    assert record.request_id == original_id
    assert book.begin_submission(record.request_id) == 2


def test_only_an_explicit_busy_rejection_can_be_deferred():
    book = LegBook()
    _, record = book.receive(leg(), planner_session='planner-a')
    assert not book.defer_busy(record)
    book.acknowledge(record.request_id, False, 'planner-a', 0, 'speed exceeds limit')
    assert not book.defer_busy(record)
    assert record.state == 'REJECTED'
    assert book.active is None


def test_previous_execution_status_and_reply_cannot_complete_or_abort_new_execution():
    book = LegBook()
    _, previous = book.receive(leg(), planner_session='planner-a')
    generation = book.begin_submission(previous.request_id)
    book.acknowledge(previous.request_id, True, 'planner-a', 7, '', generation)
    book.abort(previous, 'Superseded after confirmed cancellation')
    _, current = book.receive(leg(), planner_session='planner-a')
    book.begin_submission(current.request_id)

    assert book.observe(status(previous, state='GOAL_REACHED')) is None
    assert current.state == 'SUBMITTING'
    book.acknowledge(current.request_id, True, 'planner-a', 9, '')
    assert book.observe(status(previous, state='GOAL_REACHED')) is None
    book.acknowledge(previous.request_id, False, 'planner-a', 7, 'late error', generation)

    assert book.active is current
    assert current.state == 'ACTIVE'
    assert current.task_id == 9
    assert book.observe(status(current, state='GOAL_REACHED', task=9)) is current
    assert book.observe(status(current, state='GOAL_REACHED', task=9)) is None


def test_new_planner_acceptance_before_its_status_preserves_completion_correlation():
    book = LegBook()
    _, record = book.receive(leg(), planner_session='planner-a')
    generation = book.begin_submission(record.request_id)

    book.acknowledge(record.request_id, True, 'planner-b', 1, 'ACCEPTED', generation)

    assert book.active is record
    assert record.state == 'ACTIVE'
    assert record.planner_session == 'planner-b'
    assert record.task_id == 1
    assert book.observe(status(record, state='GOAL_REACHED', task=1, session='planner-b')) is record
