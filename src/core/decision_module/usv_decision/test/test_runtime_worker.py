"""Regression tests for the decision planner worker lifecycle."""

import concurrent.futures

from usv_decision import runtime


def test_default_worker_starts_its_process_before_first_request(monkeypatch):
    submitted = []

    class Future:
        def __init__(self):
            self.result_called = False

        def cancel(self):
            return False

        def result(self):
            self.result_called = True

    class Executor:
        def __init__(self, *args, **kwargs):
            pass

        def submit(self, function, *args):
            submitted.append((function, args))
            future = Future()
            submitted[-1] += (future,)
            return future

        def shutdown(self, **kwargs):
            pass

    monkeypatch.setattr(concurrent.futures, "ProcessPoolExecutor", Executor)
    worker = runtime.LatestPlanWorker()
    try:
        assert len(submitted) == 1
        assert submitted[0][0].__name__ == "_warmup_worker"
        assert submitted[0][2].result_called
    finally:
        worker.close()
