from perception.scan_queue import ClockGuard, ScanQueue


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
