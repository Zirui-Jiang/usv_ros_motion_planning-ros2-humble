"""Result aggregation tests using the actual acceptance summary/event schema."""

import importlib.util
import json
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("acceptance_compare", Path(__file__).with_name("compare_results.py"))
compare = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(compare)


def result(mode="python", repeat=1, rmse=0.2, duration=10.0, passed=True, status="completed"):
    return {"trial": f"{mode}_pid_straight_{repeat:02d}", "mode": mode, "controller": "pid",
            "scenario": "straight", "repeat": repeat, "fault": None, "result": status,
            "passed": passed, "cross_track_initial_m": {"count": 5, "rms": rmse},
            "duration_sim_s": duration, "minimum_footprint_clearance_m": None,
            "status": None, "mapping_status": None}


def write_run(path, rows, *, timestamp=1.0, events=None, finished=True, planned=None):
    path.mkdir()
    (path / "summary.json").write_text(json.dumps({"results": rows, "run_finished": finished,
        "completed": sum(r["result"] == "completed" for r in rows),
        "passed": sum(r["passed"] is True for r in rows)}), encoding="utf-8")
    (path / "manifest.json").write_text(json.dumps({"mode": "live", "created_unix_s": timestamp,
        "matrix": planned or [], "python_git_commit": "python_fixture", "cpp_git_commit": "cpp_fixture"}), encoding="utf-8")
    for row in rows:
        child = path / row["trial"]
        child.mkdir()
        (child / "summary.json").write_text(json.dumps(row), encoding="utf-8")
        (child / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in (events or [])), encoding="utf-8")
    return path


def performance_events(*, control=3.0, processing=20.0, period=100.0,
                       times=(100.0, 105.0, 110.0), queues=(0, 0, 0),
                       overflow=(0, 0, 0), expired=(0, 0, 0), progress=(10, 60, 110)):
    """Synthetic actual-schema telemetry; counters follow queue conservation."""
    events = [{"event": "goal_published", "wall_monotonic_s": times[0]}]
    for index, stamp in enumerate(times):
        count = progress[index]
        nav = {"metrics": {"control_ms": {"count": count, "p99": control}},
               "process": {"pid": 11, "monotonic": stamp}}
        mapping = {"integrated_scans": count, "invalid_scans": 0, "queue_size": queues[index],
                   "processing_ms": {"count": count, "p95": processing, "p99": processing},
                   "observed_scan_period_ms": {"count": count, "p50": period, "p95": period},
                   "queue_counters": {"accepted": count + overflow[index] + expired[index] + queues[index],
                       "processed": count, "overflow": overflow[index], "expired": expired[index],
                       "invalid": 0, "duplicate": 0, "resets": 0},
                   "process": {"pid": 12, "monotonic": stamp}}
        events.extend([{"event": "navigation_status", "wall_monotonic_s": stamp, "payload": nav},
                       {"event": "mapping_status", "wall_monotonic_s": stamp, "payload": mapping}])
    events.append({"event": "trial_finished", "wall_monotonic_s": times[-1]})
    return events


def paired_report(tmp_path, events):
    py = write_run(tmp_path / "py", [result()], events=events)
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    return compare.build_report([("python", py), ("cpp", cpp)])


def test_source_selection_preserves_legacy_and_raw_summary(tmp_path):
    import runpy
    events = runpy.run_path(str(Path(__file__).with_name("test_tracking_metrics.py")))["source_events"]()
    py_row, cpp_row = result(rmse=3.), result("cpp", rmse=1.)
    py = write_run(tmp_path / "py", [py_row], events=events)
    cpp = write_run(tmp_path / "cpp", [cpp_row], events=events)
    legacy = compare.build_report([("python", py), ("cpp", cpp)])
    source = compare.build_report([("python", py), ("cpp", cpp)], rmse_metric="source_time_trapezoid", frame_contract="otter_fake_localization")
    assert legacy["comparisons"][0]["passed"] is False
    assert legacy["comparisons"][0]["metrics"]["cross_track_rmse_m"]["ratio"] == 3.
    assert source["comparisons"][0]["passed"] is True
    assert source["comparisons"][0]["metrics"]["cross_track_rmse_m"]["ratio"] == 1.
    assert source["comparisons"][0]["rmse_alternatives"]["legacy_wall"]["ratio"] == 3.
    assert source["trials"][0]["result"] == py_row
    assert source["method"]["rmse_metric"] == "source_time_trapezoid"
    assert "source_time_trapezoid" in compare.markdown(source)


def test_selected_source_without_contract_is_unknown_not_wall_fallback(tmp_path):
    py = write_run(tmp_path / "py", [result()])
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    report = compare.build_report([("python", py), ("cpp", cpp)], rmse_metric="source_time_trapezoid")
    pair = report["comparisons"][0]
    assert pair["passed"] is None and not pair["comparable"]
    assert "explicit_frame_contract_required" in str(pair["reasons"])
    assert pair["rmse_alternatives"]["legacy_wall"]["passed"] is True


def test_nonfinite_source_pose_yields_serializable_unknown(tmp_path):
    import runpy
    events = runpy.run_path(str(Path(__file__).with_name("test_tracking_metrics.py")))["source_events"]()
    events[3]["pose"][0] = float("nan")
    py = write_run(tmp_path / "py", [result()], events=events)
    report = compare.build_report([("python", py)], rmse_metric="source_time_trapezoid", frame_contract="otter_fake_localization")
    score = report["trials"][0]["observations"]["tracking"]["source_time_trapezoid"]
    assert score["status"] == "unknown" and score["rmse_m"] is None
    json.dumps(report, allow_nan=False)
    with pytest.raises(ValueError, match="nonfinite JSON"):
        compare.build_report([("python", py)])


def test_completed_pairs_at_ten_percent_limit_and_group_counts(tmp_path):
    py = write_run(tmp_path / "py", [result(rmse=0.22, duration=11), result(repeat=2, rmse=0.1, duration=9)], events=performance_events())
    cpp = write_run(tmp_path / "cpp", [result("cpp"), result("cpp", repeat=2)])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    assert report["summary"]["all_passed"]
    assert report["summary"]["passed_pairs"] == 2
    group = next(g for g in report["groups"] if g["mode"] == "python")
    assert group["observed_repeats"] == [1, 2]
    assert group["completed_trials"] == group["passed_trials"] == 2
    assert group["metrics"]["cross_track_rmse_m"]["mean"] == pytest.approx(0.16)
    assert group["metrics"]["minimum_footprint_clearance_m"]["min"] is None


@pytest.mark.parametrize("status,passed", [("completed", False), ("blocked", True), ("not_run", False), ("interrupted", False)])
def test_invalid_baseline_never_computes_ratio_or_counts_as_passed(tmp_path, status, passed):
    py = write_run(tmp_path / "py", [result()])
    cpp = write_run(tmp_path / "cpp", [result("cpp", status=status, passed=passed)])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    pair = report["comparisons"][0]
    assert pair["passed"] is None and not pair["comparable"]
    assert pair["metrics"] == {}
    assert "cpp_not_completed_and_passed" in pair["reasons"][0]
    assert report["summary"]["passed_pairs"] == 0


def test_missing_baseline_is_explicit_and_missing_metric_is_not_zero(tmp_path):
    py = write_run(tmp_path / "py", [result()])
    report = compare.build_report([("python", py)])
    assert report["comparisons"][0]["reasons"] == ["cpp_result_missing"]
    assert not report["summary"]["all_passed"]
    cpp = write_run(tmp_path / "cpp", [result("cpp", duration=None)])
    pair = compare.build_report([("python", py), ("cpp", cpp)])["comparisons"][0]
    assert not pair["comparable"] and pair["passed"] is None
    assert pair["metrics"]["duration_sim_s"]["cpp"] is None


def test_completed_passed_trials_can_still_fail_relative_ten_percent_requirement(tmp_path):
    py = write_run(tmp_path / "py", [result(rmse=0.3)])
    cpp = write_run(tmp_path / "cpp", [result("cpp", rmse=0.2)])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    assert report["comparisons"][0]["comparable"]
    assert report["comparisons"][0]["passed"] is False
    assert report["summary"]["failed_pairs"] == 1
    assert not report["summary"]["all_passed"]


@pytest.mark.parametrize("value,passed", [(0.0, True), (1e-6, True), (1.01e-6, False)])
def test_exact_zero_baseline_has_absolute_tolerance_and_no_invented_ratio(value, passed):
    compared = compare.ratio_comparison(value, 0.0)
    assert compared["comparable"] and compared["passed"] is passed
    assert compared["ratio"] is None
    assert compared["method"] == "zero_baseline_absolute_tolerance"
    assert compare.ratio_comparison(2e-8, 1e-8)["passed"] is False  # Nonzero denominator still uses 1.10.


def test_thruster_variation_uses_observed_changes_without_padding_zeros(tmp_path):
    commands = {"left": [0.2, -0.1, 0.5], "right": [0, 0.3, 0.2], "lateral": [0, 0]}
    events = [{"event": "thruster_command", "axis": axis, "normalized_command": value}
              for axis, values in commands.items() for value in values]
    run = write_run(tmp_path / "py", [result()], events=events)
    observed = compare.build_report([("python", run)])["trials"][0]["observations"]
    assert observed["thrusters"]["axes"]["left"]["total_variation"] == pytest.approx(0.9)
    assert observed["thrusters"]["total_variation"] == pytest.approx(1.3)


def test_missing_thruster_channel_cannot_be_imputed_as_zero(tmp_path):
    events = [{"event": "thruster_command", "axis": "left", "normalized_command": value} for value in [0, 0.5]]
    run = write_run(tmp_path / "py", [result()], events=events)
    observed = compare.build_report([("python", run)])["trials"][0]["observations"]
    assert observed["thrusters"]["total_variation"] is None
    assert observed["thrusters"]["axes"]["lateral"]["total_variation"] is None


def test_cpu_delta_rss_and_final_status_mapping_counters(tmp_path):
    def payload(cpu, mono, peak):
        return {"process": {"pid": 7, "cpu_seconds": cpu, "monotonic": mono, "peak_rss_kib": peak},
                "metrics": {"control_ms": {"count": 20, "p99": 3.0}, "planning_ms": {"count": 4, "p99": 8.0}}}
    events = [{"event": "navigation_status", "payload": payload(10, 100, 1024)},
              {"event": "navigation_status", "payload": payload(10.4, 102, 2048)}]
    row = result()
    row["status"] = payload(10.4, 102, 2048)
    row["mapping_status"] = {"processing_ms": {"count": 5, "p99": 6.0},
        "queue_ms": {"count": 5, "p99": 7.0}, "queue_counters": {"accepted": 12, "processed": 10, "expired": 2}}
    run = write_run(tmp_path / "py", [row], events=events)
    report = compare.build_report([("python", run)])
    observed = report["trials"][0]["observations"]
    assert observed["metrics"]["navigation_cpu_percent"] == pytest.approx(20)
    assert observed["metrics"]["navigation_peak_rss_kib"] == 2048
    assert observed["metrics"]["mapping_cpu_percent"] is None
    assert observed["metrics"]["mapping_processing_p99_ms"] == 6
    assert report["groups"][0]["mapping_final_counters"]["queue_counters.expired"]["sum_across_observed_final_snapshots"] == 2


def test_process_cpu_is_not_differenced_across_pids_or_clock_resets():
    samples = [{"pid": 1, "cpu_seconds": 100, "monotonic": 100, "peak_rss_kib": 3},
               {"pid": 2, "cpu_seconds": 1, "monotonic": 101, "peak_rss_kib": 4},
               {"pid": 2, "cpu_seconds": 1.5, "monotonic": 102, "peak_rss_kib": 5}]
    observed = compare.process_metrics(samples)
    assert observed["mean_cpu_percent"] == 50
    assert observed["per_pid"][0]["mean_cpu_percent"] is None
    assert observed["peak_rss_kib"] == 5


def test_duplicates_require_explicit_selection_and_keep_sources(tmp_path):
    older = write_run(tmp_path / "older", [result(duration=20)], timestamp=1)
    newer = write_run(tmp_path / "newer", [result(duration=10)], timestamp=2)
    with pytest.raises(ValueError, match="duplicate trial"):
        compare.build_report([("python", newer), ("python", older)])
    latest = compare.build_report([("python", newer), ("python", older)], "latest")
    provided = compare.build_report([("python", newer), ("python", older)], "provided-list")
    assert latest["trials"][0]["result"]["duration_sim_s"] == 10
    assert provided["trials"][0]["result"]["duration_sim_s"] == 20
    assert latest["selection"]["decisions"][0]["selected"]["run_directory"] == str(newer)
    assert latest["selection"]["decisions"][0]["discarded"][0]["run_directory"] == str(older)


def test_planned_but_missing_trial_and_faults_are_not_counted_as_comparable(tmp_path):
    expected = [{"mode": "python", "controller": "pid", "scenario": "straight", "repeat": 2}]
    py = write_run(tmp_path / "py", [result()], planned=expected)
    fault = {"trial": "python_pid_odom_dropout", "mode": "python", "fault": "odom_dropout", "result": "completed", "passed": True}
    extra = write_run(tmp_path / "faults", [fault, result(repeat=3)])
    report = compare.build_report([("python", py), ("extra", extra)])
    assert report["summary"]["pairs"] == 2
    assert report["summary"]["fault_trials"] == 1
    assert report["summary"]["fault_passed"] == 1
    assert report["sources"][1]["ignored_trials"] == ["python_pid_straight_03"]
    assert not report["summary"]["all_passed"]


def test_cli_writes_new_reports_and_never_overwrites_raw_or_previous_report(tmp_path):
    py = write_run(tmp_path / "py", [result()], events=performance_events())
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    original = (py / "summary.json").read_bytes()
    out = tmp_path / "report"
    args = ["--python", str(py), "--baseline", str(cpp), "--output", str(out)]
    assert compare.main(args) == 0
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    assert report["summary"]["all_passed"]
    assert "不可比" in (out / "report.md").read_text(encoding="utf-8")
    with pytest.raises(SystemExit) as error:
        compare.main(args)
    assert error.value.code == 2
    with pytest.raises(SystemExit):
        compare.main(["--python", str(py), "--output", str(py / "report")])
    assert (py / "summary.json").read_bytes() == original


def test_missing_performance_cannot_report_all_passed(tmp_path):
    report = paired_report(tmp_path, [])
    assert report["summary"]["passed_pairs"] == 1
    assert not report["summary"]["all_passed"]
    assert report["summary"]["performance_unknown"] == 1
    assert report["performance"][0]["control_timing"]["status"] == "unknown"


@pytest.mark.parametrize("p99,status", [(99.999, "pass"), (100.0, "fail"), (250.0, "fail")])
def test_control_p99_strict_limit_and_cpp_without_telemetry(tmp_path, p99, status):
    report = paired_report(tmp_path, performance_events(control=p99))
    assert report["performance"][0]["control_timing"]["status"] == status
    assert report["summary"]["all_passed"] == (status == "pass")
    assert report["summary"]["performance_trials"] == 1  # No C++ telemetry gate.


def test_slow_control_and_mapping_fail_despite_passing_navigation_ratios(tmp_path):
    report = paired_report(tmp_path, performance_events(control=250, processing=500, period=40,
                                                       overflow=(0, 50, 100)))
    performance = report["performance"][0]
    assert report["comparisons"][0]["passed"]
    assert performance["status"] == "fail"
    assert all(performance[k]["status"] == "fail" for k in ("control_timing", "mapping_timing", "backlog"))
    assert not report["summary"]["all_passed"]


@pytest.mark.parametrize("missing", ["processing_ms", "observed_scan_period_ms", "integrated_scans"])
def test_mapping_requires_integrated_computation_and_observed_period(tmp_path, missing):
    events = performance_events()
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"].pop(missing)
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "unknown"
    assert performance["status"] == "unknown"


def test_mapping_budget_uses_median_scan_period_not_long_gap_p95(tmp_path):
    events = performance_events(processing=50, period=40)
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"]["observed_scan_period_ms"]["p95"] = 2000
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "fail"


def test_two_nonempty_gauges_do_not_prove_sustained_backlog(tmp_path):
    performance = paired_report(tmp_path, performance_events(times=(100, 105), queues=(1, 1)))["performance"][0]
    assert performance["backlog"]["status"] == "unknown"
    assert performance["backlog"]["full_intervals"] == 1


def test_balanced_progress_with_nonempty_queue_does_not_mean_backlog_failure(tmp_path):
    report = paired_report(tmp_path, performance_events(queues=(1, 2, 1)))
    assert report["performance"][0]["backlog"]["status"] == "pass"
    assert report["summary"]["all_passed"]


@pytest.mark.parametrize("queues,status", [((1, 0, 2), "pass"), ((0, 1, 2), "unknown"),
                                           ((0, 1, 0, 1, 2), "pass"), ((1, 0, 4), "unknown")])
def test_recovered_small_endpoint_transient_is_not_sustained_backlog(tmp_path, queues, status):
    count = len(queues)
    events = performance_events(times=tuple(100 + 5*i for i in range(count)), queues=queues,
                                overflow=(0,)*count, expired=(0,)*count,
                                progress=tuple(10 + 50*i for i in range(count)))
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["backlog"]["status"] == status
    assert performance["status"] == status
    assert performance["backlog"]["net_queue_growth"] == queues[-1] - queues[0]


@pytest.mark.parametrize("overflow,status", [((0, 1, 2), "fail"), ((0, 1, 1), "unknown")])
def test_backlog_needs_two_consecutive_growth_intervals_to_fail(tmp_path, overflow, status):
    performance = paired_report(tmp_path, performance_events(overflow=overflow))["performance"][0]
    assert performance["backlog"]["status"] == status
    assert not performance["passed"]


def test_occasional_tf_expiry_is_recorded_without_automatic_failure(tmp_path):
    performance = paired_report(tmp_path, performance_events(expired=(0, 1, 1)))["performance"][0]
    assert performance["backlog"]["status"] == "pass"
    assert performance["backlog"]["expired_delta_observed"] == 1


@pytest.mark.parametrize("change", ["gap", "stalled", "bad_accounting", "epoch_reset", "missing_tail"])
def test_incomplete_throughput_evidence_stays_unknown(tmp_path, change):
    events = performance_events(times=(100, 105, 115)) if change == "gap" else performance_events()
    mappings = [event for event in events if event["event"] == "mapping_status"]
    if change == "stalled":
        events = performance_events(progress=(10, 10, 10))
    elif change == "bad_accounting":
        mappings[-1]["payload"]["queue_counters"]["accepted"] += 1
    elif change == "epoch_reset":
        mappings[-1]["payload"]["queue_counters"]["resets"] = 1
    elif change == "missing_tail":
        events[-1]["wall_monotonic_s"] = 130
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["backlog"]["status"] == "unknown"
    assert not performance["passed"]


def test_control_telemetry_must_cover_end_of_trial(tmp_path):
    events = performance_events()
    events = [e for e in events if e["event"] != "navigation_status" or e["wall_monotonic_s"] == 100]
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["control_timing"]["status"] == "unknown"
    assert performance["backlog"]["status"] == "pass"


def test_missing_status_timestamp_is_not_silently_discarded(tmp_path):
    events = performance_events()
    for event in events:
        if event["event"] == "mapping_status" and event["wall_monotonic_s"] == 105:
            event.pop("wall_monotonic_s")
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "unknown"
    assert performance["backlog"]["status"] == "unknown"


def test_acquisition_end_excludes_postprocessing_and_later_telemetry(tmp_path):
    events = performance_events()
    events[-1]["wall_monotonic_s"] = 125
    events.insert(-1, {"event": "acquisition_finished", "wall_monotonic_s": 110})
    events.insert(-1, {"event": "navigation_status", "wall_monotonic_s": 120,
                       "payload": {"metrics": {"control_ms": {"count": 5, "p99": 250}}}})
    report = paired_report(tmp_path, events)
    performance = report["performance"][0]
    assert performance["window"]["end"] == 110
    assert performance["window"]["end_source"] == "acquisition_finished"
    assert not performance["window"]["legacy_fallback"]
    assert report["summary"]["all_passed"]


def test_old_records_explicitly_keep_trial_finished_fallback(tmp_path):
    events = performance_events()
    events[-1]["wall_monotonic_s"] = 111.6
    report = paired_report(tmp_path, events)
    performance = report["performance"][0]
    assert performance["window"]["end_source"] == "trial_finished"
    assert performance["window"]["legacy_fallback"]
    assert "postprocessing" in performance["window"]["fallback_reason"]
    assert performance["control_timing"]["status"] == "unknown"
    assert "旧末端回退" in compare.markdown(report)


def test_present_but_invalid_acquisition_end_is_not_silently_replaced(tmp_path):
    events = performance_events()
    events.insert(-1, {"event": "acquisition_finished", "wall_monotonic_s": None})
    performance = paired_report(tmp_path, events)["performance"][0]
    assert not performance["window"]["complete"]
    assert not performance["window"]["legacy_fallback"]
    assert performance["status"] == "unknown"


def post_drain_events(**kwargs):
    """New actual-schema observations; legacy fixtures intentionally stay old."""
    events = performance_events(**kwargs)
    for event in events:
        if event["event"] != "mapping_status":
            continue
        payload, stamp = event["payload"], event["wall_monotonic_s"]
        queue = payload["queue_size"]
        payload["queue_ms"] = {"count": payload["integrated_scans"], "p50": 50., "p95": 90.,
                               "p99": 100., "max": 110.}
        ages = {"count": 501 if queue else 0}
        if queue:
            ages.update(p50=30., p95=60., p99=65., max=70.)
        payload["queue_observation_window"] = {
            "window_s": 5., "sampling_phase": "after_drain", "capacity": 5, "tf_wait_timeout_s": .2,
            "observation_count": 501, "pending_observations": 501 if queue else 0,
            "empty_observations": 0 if queue else 501, "max_queue_size": queue, "latest_queue_size": queue,
            "latest_oldest_pending_age_ms": 50. if queue else None, "oldest_pending_age_ms": ages,
            "oldest_pending_distinct_count": 75 if queue else 0,
            "latest_oldest_pending": {"frame_id": "lidar", "stamp_ns": int(stamp * 1e9),
                                      "received_at_monotonic_s": stamp - .05} if queue else None,
            "max_observation_gap_ms": 10., "observed_span_s": 5.,
            "first_observed_monotonic_s": stamp - 5., "last_observed_monotonic_s": stamp,
        }
    return events


def test_new_dense_latency_evidence_accepts_stable_one_frame_pipeline(tmp_path):
    report = paired_report(tmp_path, post_drain_events(queues=(0, 1, 1)))
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "pass" and report["summary"]["all_passed"]
    assert backlog["observation_policy"] == "post_drain_latency_v1"
    assert backlog["net_queue_growth"] == 1 and not backlog["observed_queue_recovery"]
    assert backlog["queue_observations"]["reported_windows"] == 3
    assert "后drain" in compare.markdown(report)


def test_old_one_frame_growth_still_unknown_without_new_measurements(tmp_path):
    backlog = paired_report(tmp_path, performance_events(queues=(0, 1, 1)))["performance"][0]["backlog"]
    assert backlog["status"] == "unknown"
    assert backlog["observation_policy"] == "legacy_sparse_gauges"


def test_new_observed_empty_queue_needs_no_invented_age_samples(tmp_path):
    backlog = paired_report(tmp_path, post_drain_events())["performance"][0]["backlog"]
    assert backlog["status"] == "pass"
    for window in backlog["queue_observations"]["windows"]:
        assert window["observation"]["oldest_pending_age_ms"] == {"count": 0}
        assert window["observation"]["empty_observations"] > 0


def test_new_window_allows_bounded_producer_summary_computation_delay(tmp_path):
    events = post_drain_events(queues=(0, 1, 1))
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"]["process"]["monotonic"] += .01
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


@pytest.mark.parametrize("change", ["missing_window", "missing_waiting", "waiting_empty", "missing_age",
                                    "phase", "short_span", "large_gap", "stale", "count_mismatch",
                                    "one_identity", "deadline_age", "deadline_waiting", "capacity",
                                    "changed_config", "short_window", "bad_latest_identity"])
def test_new_missing_inconsistent_or_unbounded_queue_evidence_is_unknown(tmp_path, change):
    events = post_drain_events(queues=(1, 1, 1))
    payload = [e["payload"] for e in events if e["event"] == "mapping_status"][1]
    window = payload["queue_observation_window"]
    if change == "missing_window":
        payload.pop("queue_observation_window")
    elif change == "missing_waiting":
        payload.pop("queue_ms")
    elif change == "waiting_empty":
        payload["queue_ms"] = {"count": 0}
    elif change == "missing_age":
        window.pop("oldest_pending_age_ms")
    elif change == "phase":
        window["sampling_phase"] = "before_drain"
    elif change == "short_span":
        window["observed_span_s"] = 1.
        window["first_observed_monotonic_s"] = window["last_observed_monotonic_s"] - 1.
    elif change == "large_gap":
        window["max_observation_gap_ms"] = 250.
    elif change == "stale":
        payload["process"]["monotonic"] += .25
    elif change == "count_mismatch":
        window["empty_observations"] = 1
    elif change == "one_identity":
        window["oldest_pending_distinct_count"] = 1
    elif change == "deadline_age":
        window["oldest_pending_age_ms"]["max"] = 201.
    elif change == "deadline_waiting":
        payload["queue_ms"]["max"] = 201.
    elif change == "capacity":
        window["max_queue_size"] = 6
    elif change == "changed_config":
        window["capacity"] = 6
    elif change == "short_window":
        window["window_s"] = 1.
    elif change == "bad_latest_identity":
        window["latest_oldest_pending"]["received_at_monotonic_s"] -= .1
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["backlog"]["status"] == "unknown"
    assert performance["status"] == "unknown"


def test_new_fine_observations_do_not_replace_three_counter_snapshots(tmp_path):
    events = post_drain_events(times=(100, 105), queues=(1, 1))
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "unknown"


@pytest.mark.parametrize("overflow,status", [((0, 1, 2), "fail"), ((0, 1, 1), "unknown")])
def test_new_small_survivor_ages_do_not_hide_overflow(tmp_path, overflow, status):
    backlog = paired_report(tmp_path, post_drain_events(overflow=overflow))["performance"][0]["backlog"]
    assert backlog["status"] == status


@pytest.mark.parametrize("expired,status", [((0, 1, 1), "pass"), ((0, 1, 2), "unknown")])
def test_new_survivor_measurements_keep_isolated_and_repeated_tf_expiry_distinct(tmp_path, expired, status):
    backlog = paired_report(tmp_path, post_drain_events(expired=expired))["performance"][0]["backlog"]
    assert backlog["status"] == status
    assert backlog["expired_delta_observed"] == expired[-1]


def test_new_latency_branch_never_relaxes_computation_limits(tmp_path):
    events = post_drain_events(control=250., processing=500., period=40.)
    report = paired_report(tmp_path, events)
    performance = report["performance"][0]
    assert performance["backlog"]["status"] == "pass"
    assert performance["control_timing"]["status"] == performance["mapping_timing"]["status"] == "fail"
    assert not report["summary"]["all_passed"]


def test_new_contract_accepts_observations_produced_by_real_scan_queue(tmp_path):
    from dataclasses import asdict
    import numpy as np
    from perception.scan_queue import ScanQueue

    queue = ScanQueue()
    events = performance_events()
    mappings = [e for e in events if e["event"] == "mapping_status"]
    waits, sample = [], 0
    for tick in range(1501):
        now = 95. + .01 * tick
        if tick % 7 == 0:
            queue.enqueue(tick, frame_id="lidar", stamp_ns=tick + 1, received_at=now)

        def process(pending):
            if now - pending.received_at < .055:
                return False
            waits.append(1000 * (now - pending.received_at))
            return True

        queue.drain(now, process)
        queue.observe_pending(now)
        if tick in (500, 1000, 1500):
            payload = mappings[sample]["payload"]
            payload.update(queue_size=len(queue), integrated_scans=queue.counters.processed,
                           queue_counters=asdict(queue.counters), queue_observation_window=queue.observation_summary(now),
                           queue_ms={"count": len(waits), "p50": float(np.percentile(waits, 50)),
                                     "p95": float(np.percentile(waits, 95)), "p99": float(np.percentile(waits, 99)),
                                     "max": max(waits)})
            sample += 1
    report = paired_report(tmp_path, events)
    assert report["performance"][0]["backlog"]["status"] == "pass"
    assert report["summary"]["all_passed"]


def accounted_scan_events(*, new_schema=True, invalid=(0, 1, 3), **kwargs):
    """Malformed accepted scans are consumed, but never integrated into a map."""
    events = post_drain_events(**kwargs)
    mappings = [event["payload"] for event in events if event["event"] == "mapping_status"]
    for payload, count in zip(mappings, invalid):
        counters = payload["queue_counters"]
        payload["invalid_scans"] = count
        counters["accepted"] += count
        if new_schema:
            counters["invalid_consumed"] = count
        else:
            counters["processed"] += count
    return events


@pytest.mark.parametrize("new_schema", [False, True])
def test_scan_counter_versions_preserve_successful_integration_accounting(tmp_path, new_schema):
    report = paired_report(tmp_path, accounted_scan_events(new_schema=new_schema))
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "pass" and report["summary"]["all_passed"]
    assert backlog["counter_schemas"] == ["successful_integrations_v2" if new_schema else "consumed_scans_v1"]
    assert backlog["sample_counter_issues"] == []
    for interval in backlog["intervals"]:
        assert interval["accounting_balanced"]
        delta = interval["counter_deltas"]
        if new_schema:
            assert delta["processed"] == interval["integrated_delta"]
            assert delta["invalid_consumed"] == interval["invalid_scans_delta"]
        else:
            assert delta["processed"] == interval["integrated_delta"] + interval["invalid_scans_delta"]
            assert "invalid_consumed" not in delta


@pytest.mark.parametrize("new_schema", [False, True])
@pytest.mark.parametrize("key", ["invalid_scans", "integrated_scans", "queue_size", "accepted",
                                "processed", "expired", "overflow", "invalid", "duplicate", "resets"])
@pytest.mark.parametrize("change", ["missing", "negative"])
def test_missing_or_negative_scan_counters_never_become_zero(tmp_path, new_schema, key, change):
    events = accounted_scan_events(new_schema=new_schema)
    payload = [e["payload"] for e in events if e["event"] == "mapping_status"][1]
    container = payload if key in ("invalid_scans", "integrated_scans", "queue_size") else payload["queue_counters"]
    if change == "missing":
        container.pop(key)
    else:
        container[key] = -1
    report = paired_report(tmp_path, events)
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "unknown" and not report["summary"]["all_passed"]
    assert backlog["missing_samples"] == 1
    assert backlog["sample_counter_issues"][0]["reason"] == "missing_or_invalid_nonnegative_integer_counter"


@pytest.mark.parametrize("change", ["missing", "negative", "null", "bool"])
def test_new_invalid_consumed_must_be_present_and_a_nonnegative_integer(tmp_path, change):
    events = accounted_scan_events(invalid=(0, 0, 0))
    counters = [e["payload"]["queue_counters"] for e in events if e["event"] == "mapping_status"][1]
    if change == "missing":
        counters.pop("invalid_consumed")
    else:
        counters["invalid_consumed"] = {"negative": -1, "null": None, "bool": False}[change]
    report = paired_report(tmp_path, events)
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "unknown" and not report["summary"]["all_passed"]
    if change == "missing":
        assert len(backlog["counter_schemas"]) == 2
        assert any(not interval["same_counter_schema"] for interval in backlog["intervals"])
    else:
        assert backlog["missing_samples"] == 1


@pytest.mark.parametrize("new_schema", [False, True])
@pytest.mark.parametrize("change", ["processed_offset", "invalid_offset", "accepted_offset"])
def test_balanced_interval_deltas_do_not_hide_invalid_lifetime_or_processing_counts(tmp_path, new_schema, change):
    events = accounted_scan_events(new_schema=new_schema)
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        counters = payload["queue_counters"]
        if change == "processed_offset":
            counters["processed"] += 2
            counters["accepted"] += 2  # Keep queue conservation; integration relation must catch this.
        elif change == "invalid_offset":
            payload["invalid_scans"] += 2
        else:
            counters["accepted"] += 2  # Equal offsets cancel in interval-only checks.
    report = paired_report(tmp_path, events)
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "unknown" and not report["summary"]["all_passed"]
    assert backlog["missing_samples"] == 3
    assert all(issue["reason"] == "inconsistent_scan_counter_accounting"
               for issue in backlog["sample_counter_issues"])


def test_new_invalid_consumed_is_not_substituted_by_admission_invalid_count(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["invalid"] += 5
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


def test_new_invalid_consumed_must_match_node_invalid_scans(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["invalid_consumed"] += 2
        payload["queue_counters"]["accepted"] += 2
    backlog = paired_report(tmp_path, events)["performance"][0]["backlog"]
    assert backlog["status"] == "unknown"
    assert all(not issue["processing_consistent"] for issue in backlog["sample_counter_issues"])


def test_invalid_consumption_without_integrations_does_not_prove_throughput(tmp_path):
    events = accounted_scan_events(progress=(10, 10, 10), invalid=(0, 50, 100))
    backlog = paired_report(tmp_path, events)["performance"][0]["backlog"]
    assert backlog["sample_counter_issues"] == []
    assert backlog["status"] == "unknown"
    assert backlog["reason"] == "observed_window_does_not_establish_stable_throughput"


def test_new_isolated_expired_scan_is_conserved_without_counting_as_processed(tmp_path):
    backlog = paired_report(tmp_path, accounted_scan_events(expired=(0, 1, 1)))["performance"][0]["backlog"]
    assert backlog["status"] == "pass" and backlog["expired_delta_observed"] == 1
    for interval in backlog["intervals"]:
        assert interval["counter_deltas"]["processed"] == interval["integrated_delta"]


def test_preexisting_reset_keeps_interval_balance_without_inventing_dropped_count(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["resets"] = 1
        payload["queue_counters"]["accepted"] += 2  # Two pending scans discarded before this acquisition window.
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


def test_safety_stopped_trial_is_not_a_completed_comparable_navigation_result(tmp_path):
    py = write_run(tmp_path / "py", [result(status="safety_stopped", passed=False)], events=performance_events())
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    assert not report["comparisons"][0]["comparable"]
    assert not report["summary"]["all_passed"]
