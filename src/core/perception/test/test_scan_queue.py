from perception.scan_queue import ClockGuard, ScanQueue
import pytest


def put(queue, stamp, received=0.0, frame="lidar"):
    return queue.enqueue(stamp, frame_id=frame, stamp_ns=stamp, received_at=received)


def test_fifo_integrates_each_scan_once_and_rejects_older_duplicates():
    queue = ScanQueue()
    assert put(queue, 10)
    assert not put(queue, 10)
    assert not put(queue, 9)
    processed = []
    assert queue.drain(0.1, lambda entry: processed.append(entry.payload) or True) == 1
    assert queue.drain(0.15, lambda entry: processed.append(entry.payload) or True) == 0
    assert not put(queue, 10)
    assert processed == [10]
    assert queue.counters.duplicate == 3


def test_queue_keeps_latest_five_and_counts_overflow():
    queue = ScanQueue()
    for value in range(1, 9):
        put(queue, value)
    processed = []
    queue.drain(0.01, lambda entry: processed.append(entry.payload) or True)
    assert processed == [4, 5, 6, 7, 8]
    assert queue.counters.overflow == 3


def test_missing_tf_waits_without_reordering_and_expires_by_wall_time():
    queue = ScanQueue(tf_wait_timeout=0.2)
    put(queue, 1, received=1.0)
    put(queue, 2, received=1.1)
    assert queue.drain(1.15, lambda entry: False) == 0
    processed = []
    queue.drain(1.21, lambda entry: processed.append(entry.payload) or True)
    assert processed == [2]
    assert queue.counters.expired == 1


def test_zero_timestamp_or_empty_frame_never_requests_latest_tf():
    queue = ScanQueue()
    assert not put(queue, 0)
    assert not put(queue, -1)
    assert not put(queue, 1, frame="")
    assert len(queue) == 0
    assert queue.counters.invalid == 3


def test_rollback_clears_pending_and_duplicate_history():
    queue = ScanQueue()
    put(queue, 20)
    queue.reset()
    assert len(queue) == 0
    assert put(queue, 10)
    assert queue.counters.resets == 1


def test_same_stamp_in_different_frames_is_not_a_duplicate():
    queue = ScanQueue()
    assert put(queue, 10, frame="one")
    assert put(queue, 10, frame="two")


def test_clock_guard_tolerates_pauses_and_detects_any_backward_jump():
    clock = ClockGuard()
    assert not clock.observe(0)
    assert not clock.observe(100)
    assert not clock.observe(100)
    assert clock.observe(10)
    assert not clock.observe(20)


def test_deadline_is_rechecked_after_slow_processing():
    queue = ScanQueue()
    put(queue, 1)
    put(queue, 2)
    times = iter([0.15, 0.21])
    assert queue.drain(lambda: next(times), lambda entry: True) == 1
    assert queue.counters.expired == 1


def test_single_frame_pipeline_is_distinguished_from_one_stuck_scan():
    queue = ScanQueue()
    for stamp in range(1, 11):
        received = stamp * 0.1
        put(queue, stamp, received=received)
        queue.drain(received + 0.01, lambda entry: entry.stamp_ns < stamp)
        queue.observe_pending(received + 0.01)
    report = queue.observation_summary(1.02)
    assert report['sampling_phase'] == 'after_drain'
    assert report['max_queue_size'] == report['latest_queue_size'] == 1
    assert report['empty_observations'] == 0
    assert report['oldest_pending_distinct_count'] == 10
    assert report['oldest_pending_age_ms']['max'] == pytest.approx(10.0)
    assert report['latest_oldest_pending']['stamp_ns'] == 10
    assert queue.counters.processed == 9
    assert queue.counters.expired == queue.counters.overflow == 0


def test_post_drain_age_does_not_hide_expiry_counter_or_change_deadlines():
    queue = ScanQueue()
    for stamp in range(1, 4):
        put(queue, stamp, received=0.0)
    queue.drain(0.1, lambda entry: False)
    queue.observe_pending(0.1)
    queue.drain(0.2, lambda entry: False)
    queue.observe_pending(0.2)
    report = queue.observation_summary(0.2)
    assert report['capacity'] == 5
    assert report['tf_wait_timeout_s'] == 0.2
    assert report['max_queue_size'] == 3
    assert report['latest_queue_size'] == 0
    assert report['latest_oldest_pending_age_ms'] is None
    assert report['empty_observations'] == 1
    assert report['oldest_pending_age_ms']['max'] == 100.0
    assert queue.counters.expired == 3
    assert queue.counters.processed == 0


def test_observations_expire_by_wall_window_and_reset_has_no_fake_zero_sample():
    queue = ScanQueue()
    assert queue.observation_summary(0.0)['max_queue_size'] is None
    put(queue, 1, received=0.0)
    queue.observe_pending(0.1)
    queue.drain(0.3, lambda entry: False)
    queue.observe_pending(0.3)
    queue.observe_pending(5.2)
    report = queue.observation_summary(5.2)
    assert report['observation_count'] == 2
    assert report['max_queue_size'] == 0
    assert report['oldest_pending_age_ms'] == {'count': 0}
    assert report['max_observation_gap_ms'] == pytest.approx(4900.0)
    assert report['observed_span_s'] == pytest.approx(4.9)
    assert queue.observation_summary(10.21)['observation_count'] == 0
    queue.observe_pending(10.3)
    queue.reset()
    assert queue.observation_summary(10.3)['observation_count'] == 0


def test_observation_clock_contract_rejects_nonfinite_and_backward_values():
    queue = ScanQueue()
    queue.observe_pending(1.0)
    for invalid in (float('nan'), float('inf'), 0.5):
        with pytest.raises(ValueError):
            queue.observe_pending(invalid)
        with pytest.raises(ValueError):
            queue.observation_summary(invalid)
