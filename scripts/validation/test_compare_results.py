# 以真实日志字段结构构造测试数据，覆盖缺失基线、不可比指标、队列证据和固定三次均值验收。
"""Result aggregation tests using the actual acceptance summary/event schema."""

import importlib.util
import json
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("acceptance_compare", Path(__file__).with_name("compare_results.py"))
compare = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(compare)


# 构造单次导航摘要，显式保留 result、passed、RMSE 与仿真耗时，缺失间隙等字段保持 None。
def result(mode="python", repeat=1, rmse=0.2, duration=10.0, passed=True, status="completed"):
    return {"trial": f"{mode}_pid_straight_{repeat:02d}", "mode": mode, "controller": "pid",
            "scenario": "straight", "repeat": repeat, "fault": None, "result": status,
            "passed": passed, "cross_track_initial_m": {"count": 5, "rms": rmse},
            "duration_sim_s": duration, "minimum_footprint_clearance_m": None,
            "status": None, "mapping_status": None}


# 生成与验收日志一致的目录、manifest、总摘要和逐次事件文件，供聚合器从磁盘读取测试。
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


# 生成目标发布至试验结束的状态流；accepted 按处理、溢出、过期和在队数量守恒构造。
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


# 为同一重复号准备 Python 与 C++ 结果，性能遥测只放入 Python 侧，返回两侧汇总报告。
def paired_report(tmp_path, events):
    py = write_run(tmp_path / "py", [result()], events=events)
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    return compare.build_report([("python", py), ("cpp", cpp)])


# 切换到源时间 RMSE 后使用新的比值判定，同时保留原始摘要和旧墙钟指标，不覆盖历史观测。
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


# 请求源时间指标却缺少显式坐标系约定时必须不可比，不能因旧指标可通过就自动回退。
def test_selected_source_without_contract_is_unknown_not_wall_fallback(tmp_path):
    py = write_run(tmp_path / "py", [result()])
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    report = compare.build_report([("python", py), ("cpp", cpp)], rmse_metric="source_time_trapezoid")
    pair = report["comparisons"][0]
    assert pair["passed"] is None and not pair["comparable"]
    assert "explicit_frame_contract_required" in str(pair["reasons"])
    assert pair["rmse_alternatives"]["legacy_wall"]["passed"] is True


# 源位姿含 NaN 时输出可被严格 JSON 序列化的 unknown；默认旧路径仍按非有限 JSON 拒绝该日志。
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


# 误差与耗时恰好增加 10% 仍可通过；按已观测重复号计数并求均值，缺失间隙不补零。
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


# C++ 基线只有同时 completed 且 passed 才能作比较；失败、阻塞、未运行或中断均不给比值和通过票。
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


# 完全缺少基线和基线缺少单个指标分别保留原因；后者指标值保持 None，比较结果保持未知。
def test_missing_baseline_is_explicit_and_missing_metric_is_not_zero(tmp_path):
    py = write_run(tmp_path / "py", [result()])
    report = compare.build_report([("python", py)])
    assert report["comparisons"][0]["reasons"] == ["cpp_result_missing"]
    assert not report["summary"]["all_passed"]
    cpp = write_run(tmp_path / "cpp", [result("cpp", duration=None)])
    pair = compare.build_report([("python", py), ("cpp", cpp)])["comparisons"][0]
    assert not pair["comparable"] and pair["passed"] is None
    assert pair["metrics"]["duration_sim_s"]["cpp"] is None


# 两侧各自导航验收通过也可能不满足相对 10% 要求；此时属于可比但失败。
def test_completed_passed_trials_can_still_fail_relative_ten_percent_requirement(tmp_path):
    py = write_run(tmp_path / "py", [result(rmse=0.3)])
    cpp = write_run(tmp_path / "cpp", [result("cpp", rmse=0.2)])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    assert report["comparisons"][0]["comparable"]
    assert report["comparisons"][0]["passed"] is False
    assert report["summary"]["failed_pairs"] == 1
    assert not report["summary"]["all_passed"]


# 零基线用 1e-6 绝对容差并令比值为空；任何非零基线仍使用相对阈值，不能借此放宽。
@pytest.mark.parametrize("value,passed", [(0.0, True), (1e-6, True), (1.01e-6, False)])
def test_exact_zero_baseline_has_absolute_tolerance_and_no_invented_ratio(value, passed):
    compared = compare.ratio_comparison(value, 0.0)
    assert compared["comparable"] and compared["passed"] is passed
    assert compared["ratio"] is None
    assert compared["method"] == "zero_baseline_absolute_tolerance"
    assert compare.ratio_comparison(2e-8, 1e-8)["passed"] is False  # Nonzero denominator still uses 1.10.


# 推力总变差只累计各轴已观测相邻指令的绝对变化，不在序列两端补零。
def test_thruster_variation_uses_observed_changes_without_padding_zeros(tmp_path):
    commands = {"left": [0.2, -0.1, 0.5], "right": [0, 0.3, 0.2], "lateral": [0, 0]}
    events = [{"event": "thruster_command", "axis": axis, "normalized_command": value}
              for axis, values in commands.items() for value in values]
    run = write_run(tmp_path / "py", [result()], events=events)
    observed = compare.build_report([("python", run)])["trials"][0]["observations"]
    assert observed["thrusters"]["axes"]["left"]["total_variation"] == pytest.approx(0.9)
    assert observed["thrusters"]["total_variation"] == pytest.approx(1.3)


# 缺失任一推力通道时总变差保持未知，不能假定未观测通道始终为零。
def test_missing_thruster_channel_cannot_be_imputed_as_zero(tmp_path):
    events = [{"event": "thruster_command", "axis": "left", "normalized_command": value} for value in [0, 0.5]]
    run = write_run(tmp_path / "py", [result()], events=events)
    observed = compare.build_report([("python", run)])["trials"][0]["observations"]
    assert observed["thrusters"]["total_variation"] is None
    assert observed["thrusters"]["axes"]["lateral"]["total_variation"] is None


# 以同进程 CPU 秒数差除单调时间差求占用率，保留峰值内存与最终队列计数；缺少映射 CPU 不补零。
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


# 此样例重点验证 PID 切换后不跨进程差分 CPU，只用同一 PID 的有效相邻样本计算。
def test_process_cpu_is_not_differenced_across_pids_or_clock_resets():
    samples = [{"pid": 1, "cpu_seconds": 100, "monotonic": 100, "peak_rss_kib": 3},
               {"pid": 2, "cpu_seconds": 1, "monotonic": 101, "peak_rss_kib": 4},
               {"pid": 2, "cpu_seconds": 1.5, "monotonic": 102, "peak_rss_kib": 5}]
    observed = compare.process_metrics(samples)
    assert observed["mean_cpu_percent"] == 50
    assert observed["per_pid"][0]["mean_cpu_percent"] is None
    assert observed["peak_rss_kib"] == 5


# 重复试验默认报错；显式 latest 与 provided-list 采用各自选择规则，并记录选中及丢弃来源。
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


# 清单中缺失的计划试验仍进入缺失统计，故障试验单独计数，extra 来源里的普通试验不混入配对。
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


# 命令行仅写新报告目录；已有报告及原始运行目录内的输出目标被拒绝，原始摘要字节保持不变。
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


# 导航配对通过不能替代性能证据；没有遥测时性能为 unknown，总体不通过。
def test_missing_performance_cannot_report_all_passed(tmp_path):
    report = paired_report(tmp_path, [])
    assert report["summary"]["passed_pairs"] == 1
    assert not report["summary"]["all_passed"]
    assert report["summary"]["performance_unknown"] == 1
    assert report["performance"][0]["control_timing"]["status"] == "unknown"


# 控制 P99 必须严格小于 100 ms，等于阈值也失败；当前性能门槛只统计 Python 试验。
@pytest.mark.parametrize("p99,status", [(99.999, "pass"), (100.0, "fail"), (250.0, "fail")])
def test_control_p99_strict_limit_and_cpp_without_telemetry(tmp_path, p99, status):
    report = paired_report(tmp_path, performance_events(control=p99))
    assert report["performance"][0]["control_timing"]["status"] == status
    assert report["summary"]["all_passed"] == (status == "pass")
    assert report["summary"]["performance_trials"] == 1  # No C++ telemetry gate.


# 构造控制超时、映射计算超预算和连续溢出，即使导航比值合格也应报告性能失败。
def test_slow_control_and_mapping_fail_despite_passing_navigation_ratios(tmp_path):
    report = paired_report(tmp_path, performance_events(control=250, processing=500, period=40,
                                                       overflow=(0, 50, 100)))
    performance = report["performance"][0]
    assert report["comparisons"][0]["passed"]
    assert performance["status"] == "fail"
    assert all(performance[k]["status"] == "fail" for k in ("control_timing", "mapping_timing", "backlog"))
    assert not report["summary"]["all_passed"]


# 映射验收须同时具备实际积分计数、计算耗时和观测扫描周期，删除任一项都只能判未知。
@pytest.mark.parametrize("missing", ["processing_ms", "observed_scan_period_ms", "integrated_scans"])
def test_mapping_requires_integrated_computation_and_observed_period(tmp_path, missing):
    events = performance_events()
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"].pop(missing)
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "unknown"
    assert performance["status"] == "unknown"


# 计算预算按扫描周期中位数判断，不能用长间隔抬高的 P95 掩盖处理速度不足。
def test_mapping_budget_uses_median_scan_period_not_long_gap_p95(tmp_path):
    events = performance_events(processing=50, period=40)
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"]["observed_scan_period_ms"]["p95"] = 2000
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "fail"


# 仅两次队列快照只有一个完整区间，证据不足以确认持续积压，状态保持 unknown。
def test_two_nonempty_gauges_do_not_prove_sustained_backlog(tmp_path):
    performance = paired_report(tmp_path, performance_events(times=(100, 105), queues=(1, 1)))["performance"][0]
    assert performance["backlog"]["status"] == "unknown"
    assert performance["backlog"]["full_intervals"] == 1


# 队列可以持续非空，只要处理推进且计数守恒、队列有恢复，就不能仅凭非空判积压失败。
def test_balanced_progress_with_nonempty_queue_does_not_mean_backlog_failure(tmp_path):
    report = paired_report(tmp_path, performance_events(queues=(1, 2, 1)))
    assert report["performance"][0]["backlog"]["status"] == "pass"
    assert report["summary"]["all_passed"]


# 区分曾恢复的小幅末端瞬态与单向增长/较大增长，保留净队列变化并按证据判通过或未知。
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


# 连续两个区间溢出计数增长才判失败；只增长一次保持未知，不直接认定持续积压。
@pytest.mark.parametrize("overflow,status", [((0, 1, 2), "fail"), ((0, 1, 1), "unknown")])
def test_backlog_needs_two_consecutive_growth_intervals_to_fail(tmp_path, overflow, status):
    performance = paired_report(tmp_path, performance_events(overflow=overflow))["performance"][0]
    assert performance["backlog"]["status"] == status
    assert not performance["passed"]


# 孤立的一次 TF 等待过期需要记录，但在其余吞吐证据完整时不自动判失败。
def test_occasional_tf_expiry_is_recorded_without_automatic_failure(tmp_path):
    performance = paired_report(tmp_path, performance_events(expired=(0, 1, 1)))["performance"][0]
    assert performance["backlog"]["status"] == "pass"
    assert performance["backlog"]["expired_delta_observed"] == 1


# 观测间隔过长、处理停滞、计数不守恒、窗口内重置或尾部缺失都不足以证明稳定吞吐。
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


# 只有试验起点的控制遥测无法证明末端时效，即使映射吞吐通过，控制时序仍为未知。
def test_control_telemetry_must_cover_end_of_trial(tmp_path):
    events = performance_events()
    events = [e for e in events if e["event"] != "navigation_status" or e["wall_monotonic_s"] == 100]
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["control_timing"]["status"] == "unknown"
    assert performance["backlog"]["status"] == "pass"


# 中间状态缺少时间戳不能静默丢弃后继续判通过，应使映射时序和积压证据均未知。
def test_missing_status_timestamp_is_not_silently_discarded(tmp_path):
    events = performance_events()
    for event in events:
        if event["event"] == "mapping_status" and event["wall_monotonic_s"] == 105:
            event.pop("wall_monotonic_s")
    performance = paired_report(tmp_path, events)["performance"][0]
    assert performance["mapping_timing"]["status"] == "unknown"
    assert performance["backlog"]["status"] == "unknown"


# 优先以 acquisition_finished 截止采集窗口，后处理期间的高延迟遥测不污染试验内性能。
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


# 旧日志缺采集结束事件时显式回退 trial_finished，并报告末端回退原因及证据不足。
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


# 采集结束事件已存在但时间戳无效时，不允许用旧结束事件替换，窗口及性能保持未知。
def test_present_but_invalid_acquisition_end_is_not_silently_replaced(tmp_path):
    events = performance_events()
    events.insert(-1, {"event": "acquisition_finished", "wall_monotonic_s": None})
    performance = paired_report(tmp_path, events)["performance"][0]
    assert not performance["window"]["complete"]
    assert not performance["window"]["legacy_fallback"]
    assert performance["status"] == "unknown"


# 在旧遥测夹具上补充出队后的密集窗口、等待耗时与最老待处理帧年龄，构造新观测契约。
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


# 新密集等待和年龄证据可确认稳定的一帧流水线，即使净队列增长一且未观察到回空也能通过。
def test_new_dense_latency_evidence_accepts_stable_one_frame_pipeline(tmp_path):
    report = paired_report(tmp_path, post_drain_events(queues=(0, 1, 1)))
    backlog = report["performance"][0]["backlog"]
    assert backlog["status"] == "pass" and report["summary"]["all_passed"]
    assert backlog["observation_policy"] == "post_drain_latency_v1"
    assert backlog["net_queue_growth"] == 1 and not backlog["observed_queue_recovery"]
    assert backlog["queue_observations"]["reported_windows"] == 3
    assert "后drain" in compare.markdown(report)


# 旧稀疏快照同样呈现一帧增长时仍保持未知，不能享受新观测契约的放宽条件。
def test_old_one_frame_growth_still_unknown_without_new_measurements(tmp_path):
    backlog = paired_report(tmp_path, performance_events(queues=(0, 1, 1)))["performance"][0]["backlog"]
    assert backlog["status"] == "unknown"
    assert backlog["observation_policy"] == "legacy_sparse_gauges"


# 真实观察到空队列时年龄样本计数为零即可，不需要伪造不存在待处理帧的年龄。
def test_new_observed_empty_queue_needs_no_invented_age_samples(tmp_path):
    backlog = paired_report(tmp_path, post_drain_events())["performance"][0]["backlog"]
    assert backlog["status"] == "pass"
    for window in backlog["queue_observations"]["windows"]:
        assert window["observation"]["oldest_pending_age_ms"] == {"count": 0}
        assert window["observation"]["empty_observations"] > 0


# 允许状态生产端汇总计算带来 10 ms 的有限延迟，不把这种小偏差误判为陈旧观测。
def test_new_window_allows_bounded_producer_summary_computation_delay(tmp_path):
    events = post_drain_events(queues=(0, 1, 1))
    for event in events:
        if event["event"] == "mapping_status":
            event["payload"]["process"]["monotonic"] += .01
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


# 逐项破坏窗口完整性、阶段、覆盖、年龄上界、容量或身份一致性，新契约均须退回 unknown。
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


# 密集窗口中的大量年龄样本不能替代三次累计计数快照；只有两次快照仍不足。
def test_new_fine_observations_do_not_replace_three_counter_snapshots(tmp_path):
    events = post_drain_events(times=(100, 105), queues=(1, 1))
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "unknown"


# 幸存待处理帧年龄较小不能掩盖已经发生的溢出，连续两段增长仍失败，单段增长仍未知。
@pytest.mark.parametrize("overflow,status", [((0, 1, 2), "fail"), ((0, 1, 1), "unknown")])
def test_new_small_survivor_ages_do_not_hide_overflow(tmp_path, overflow, status):
    backlog = paired_report(tmp_path, post_drain_events(overflow=overflow))["performance"][0]["backlog"]
    assert backlog["status"] == status


# 新年龄观测仍区分孤立与连续 TF 过期：一次允许通过，连续增长不能证明稳定吞吐。
@pytest.mark.parametrize("expired,status", [((0, 1, 1), "pass"), ((0, 1, 2), "unknown")])
def test_new_survivor_measurements_keep_isolated_and_repeated_tf_expiry_distinct(tmp_path, expired, status):
    backlog = paired_report(tmp_path, post_drain_events(expired=expired))["performance"][0]["backlog"]
    assert backlog["status"] == status
    assert backlog["expired_delta_observed"] == expired[-1]


# 等待/积压分支通过不放宽控制和映射计算时限，计算超预算仍阻止总体通过。
def test_new_latency_branch_never_relaxes_computation_limits(tmp_path):
    events = post_drain_events(control=250., processing=500., period=40.)
    report = paired_report(tmp_path, events)
    performance = report["performance"][0]
    assert performance["backlog"]["status"] == "pass"
    assert performance["control_timing"]["status"] == performance["mapping_timing"]["status"] == "fail"
    assert not report["summary"]["all_passed"]


# 用真实 ScanQueue 入队、延迟处理和观测接口生成遥测，验证新契约能接受实际生产数据结构。
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


# 构造已接纳但因无效数据被消费的扫描；新旧版本分别用 invalid_consumed 与含无效项的 processed 表达。
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


# 同时验证两版计数守恒：新版 processed 等于成功积分，旧版 processed 包含成功积分及无效消费。
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


# 新旧版本任一必要计数缺失或为负都不能补零，应记录无效样本并使积压验收未知。
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


# 新版无效消费计数须为非负整数，None/布尔也无效；缺字段造成版本混用时同样不能通过。
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


# 所有快照加相同偏移会在区间差分中抵消，因此还须检查每个快照的生命周期守恒和处理关系。
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


# 入队拒绝计数 invalid 与入队后无效消费不同，增加前者不应破坏后者的积分守恒关系。
def test_new_invalid_consumed_is_not_substituted_by_admission_invalid_count(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["invalid"] += 5
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


# 即使同步提高 accepted 维持队列守恒，invalid_consumed 与节点 invalid_scans 不一致仍应判未知。
def test_new_invalid_consumed_must_match_node_invalid_scans(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["invalid_consumed"] += 2
        payload["queue_counters"]["accepted"] += 2
    backlog = paired_report(tmp_path, events)["performance"][0]["backlog"]
    assert backlog["status"] == "unknown"
    assert all(not issue["processing_consistent"] for issue in backlog["sample_counter_issues"])


# 持续消费无效扫描但成功积分计数不变时，计数可守恒，仍不能据此证明建图吞吐正常。
def test_invalid_consumption_without_integrations_does_not_prove_throughput(tmp_path):
    events = accounted_scan_events(progress=(10, 10, 10), invalid=(0, 50, 100))
    backlog = paired_report(tmp_path, events)["performance"][0]["backlog"]
    assert backlog["sample_counter_issues"] == []
    assert backlog["status"] == "unknown"
    assert backlog["reason"] == "observed_window_does_not_establish_stable_throughput"


# 一次过期可以通过并被单独计数，不得并入 processed 冒充成功积分。
def test_new_isolated_expired_scan_is_conserved_without_counting_as_processed(tmp_path):
    backlog = paired_report(tmp_path, accounted_scan_events(expired=(0, 1, 1)))["performance"][0]["backlog"]
    assert backlog["status"] == "pass" and backlog["expired_delta_observed"] == 1
    for interval in backlog["intervals"]:
        assert interval["counter_deltas"]["processed"] == interval["integrated_delta"]


# 采集前已发生的重置与遗留接纳数偏差不破坏窗口内守恒；不凭空推算一个丢弃计数。
def test_preexisting_reset_keeps_interval_balance_without_inventing_dropped_count(tmp_path):
    events = accounted_scan_events()
    for payload in [e["payload"] for e in events if e["event"] == "mapping_status"]:
        payload["queue_counters"]["resets"] = 1
        payload["queue_counters"]["accepted"] += 2  # Two pending scans discarded before this acquisition window.
    assert paired_report(tmp_path, events)["performance"][0]["backlog"]["status"] == "pass"


# 安全停止不同于成功完成，即使有完整性能遥测也不能进入可比导航配对。
def test_safety_stopped_trial_is_not_a_completed_comparable_navigation_result(tmp_path):
    py = write_run(tmp_path / "py", [result(status="safety_stopped", passed=False)], events=performance_events())
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    report = compare.build_report([("python", py), ("cpp", cpp)])
    assert not report["comparisons"][0]["comparable"]
    assert not report["summary"]["all_passed"]


# 为两种实现各构造固定第 1/2/3 次试验，并分别提供有效的源时间 RMSE 证据。
def scenario_fixture(python=(.12, .08, .10), cpp=(.10, .10, .10), durations=(12., 8., 10.)):
    """Three fixed repetitions with independently valid source-time evidence."""
    records = []
    for mode, values in (("python", python), ("cpp", cpp)):
        for repeat, value in enumerate(values, 1):
            records.append({"result": result(mode, repeat, rmse=value,
                duration=durations[repeat - 1] if mode == "python" else 10.),
                "source": {"manifest_mode": "live", "run_directory": mode},
                "observations": {"tracking": {"source_time_trapezoid": {
                    "status": "valid", "rmse_m": value, "reasons": []}}}})
    return refresh_scenario_fixture(records)


# 按试验键重新配对两侧记录，显式使用源时间 RMSE 及既定相对/零基线阈值。
def refresh_scenario_fixture(records):
    keys = {compare.comparison_key(record["result"]) for record in records}
    pairs = []
    for key in sorted(keys):
        modes = {r["result"]["mode"]: r for r in records if compare.comparison_key(r["result"]) == key}
        pairs.append(compare.compare_pair(key, modes.get("python"), modes.get("cpp"), 1.1, 1e-6,
                                          "source_time_trapezoid"))
    return {"trials": records, "comparisons": pairs,
            "method": {"rmse_metric": "source_time_trapezoid", "ratio_limit": 1.1, "zero_baseline_atol": 1e-6}}


# 三次 RMSE 取算术均值并报告样本标准差 ddof=1；均值通过仍保留逐次失败及最差比值，且不修改输入报告。
def test_scenario_mean_is_arithmetic_of_three_rmse_values_and_retains_worst_pair():
    report = scenario_fixture()
    before = json.dumps(report, sort_keys=True)
    observed = compare.scenario_mean_acceptance(report)
    group = observed["groups"][0]
    assert group["comparable"] and group["passed"]
    assert group["python"]["metrics"]["cross_track_rmse_m"] == {
        "values": [.12, .08, .10], "count": 3, "mean": pytest.approx(.1),
        "std_ddof1": pytest.approx(.02), "worst": .12}
    assert group["metrics"]["cross_track_rmse_m"]["ratio"] == pytest.approx(1.)
    assert group["metrics"]["duration_sim_s"]["ratio"] == 1.
    assert group["individual_comparisons"][0]["passed"] is False
    assert group["worst_comparable_repeat_ratios"]["cross_track_rmse_m"] == 1.2
    assert observed["summary"]["all_eligible_passed"]
    assert json.dumps(report, sort_keys=True) == before


# 固定三次必须齐全且证据有效；缺次、加次、基线失败、源时间未知或耗时缺失都不可挑最好几次替代。
@pytest.mark.parametrize("mutation, reason", [
    ("missing_repeat", "requires_exact_repeats_1_2_3"),
    ("extra_repeat", "requires_exact_repeats_1_2_3"),
    ("failed_baseline", "cpp_repeat_2_not_completed_and_passed"),
    ("source_unknown", "cpp_repeat_2_source_time_not_valid"),
    ("missing_duration", "cpp_repeat_2_invalid_duration_sim_s"),
])
def test_scenario_mean_never_selects_only_the_best_valid_repeats(mutation, reason):
    report = scenario_fixture()
    rows = report["trials"]
    record = rows[4]
    if mutation == "missing_repeat":
        rows.remove(record)
    elif mutation == "extra_repeat":
        import copy
        extra = copy.deepcopy(record)
        extra["result"].update(repeat=4, trial="cpp_pid_straight_04")
        rows.append(extra)
    elif mutation == "failed_baseline":
        record["result"]["passed"] = False
    elif mutation == "source_unknown":
        record["observations"]["tracking"]["source_time_trapezoid"]["status"] = "unknown"
    else:
        record["result"]["duration_sim_s"] = None
    observed = compare.scenario_mean_acceptance(refresh_scenario_fixture(rows))
    group = observed["groups"][0]
    assert not group["comparable"] and group["passed"] is None
    assert reason in str(group["reasons"])
    assert observed["summary"]["incomparable_groups"] == 1
    assert not observed["summary"]["all_eligible_passed"]


# 均值 RMSE 合格不能抵消耗时超限；零基线仍独立采用绝对容差并保持 ratio=None。
def test_scenario_mean_duration_failure_and_zero_baseline_atol_are_independent():
    failed_time = compare.scenario_mean_acceptance(scenario_fixture(durations=(12., 12., 12.)))["groups"][0]
    assert failed_time["metrics"]["cross_track_rmse_m"]["passed"]
    assert failed_time["passed"] is False
    for value, expected in ((1e-6, True), (1.01e-6, False)):
        group = compare.scenario_mean_acceptance(scenario_fixture((value,) * 3, (0.,) * 3))["groups"][0]
        assert group["metrics"]["cross_track_rmse_m"]["ratio"] is None
        assert group["passed"] is expected


# 即使旧墙钟指标被选为逐次指标，场景三次均值仍要求有效源时间证据。
def test_scenario_mean_requires_source_valid_even_when_wall_selected():
    report = scenario_fixture()
    report["method"]["rmse_metric"] = "legacy_wall"
    report["trials"][0]["observations"]["tracking"] = {}
    assert not compare.scenario_mean_acceptance(report)["groups"][0]["comparable"]


# 默认和显式 per_repeat 路径应输出等价 JSON/Markdown，并不引入仅属于场景均值的新字段。
def test_default_and_explicit_per_repeat_reports_and_cli_stay_compatible(tmp_path):
    py = write_run(tmp_path / "py", [result()], events=performance_events())
    cpp = write_run(tmp_path / "cpp", [result("cpp")])
    sources = [("python", py), ("cpp", cpp)]
    default = compare.build_report(sources)
    explicit = compare.build_report(sources, acceptance_aggregation="per_repeat")
    for report in (default, explicit):
        report.pop("generated_utc")
        assert "scenario_acceptance" not in report
        assert "acceptance_aggregation" not in report["method"]
    assert default == explicit
    for suffix, flag in (("default", []), ("explicit", ["--acceptance-aggregation", "per_repeat"])):
        assert compare.main(["--python", str(py), "--baseline", str(cpp),
                             "--output", str(tmp_path / suffix), *flag]) == 0
    reports = [json.loads((tmp_path / suffix / "report.json").read_text()) for suffix in ("default", "explicit")]
    for report in reports:
        report.pop("generated_utc")
    assert reports[0] == reports[1]
    assert (tmp_path / "default" / "report.md").read_bytes() == (tmp_path / "explicit" / "report.md").read_bytes()


# 场景均值通过仍不能盖过三次性能未知；报告标明算术均值、标准差及逐次结果，CLI 保持失败退出码。
def test_scenario_mean_report_labels_and_unknown_performance_remain_independent(tmp_path):
    import runpy
    events = runpy.run_path(str(Path(__file__).with_name("test_tracking_metrics.py")))["source_events"]()
    py = write_run(tmp_path / "py", [result(repeat=i) for i in (1, 2, 3)], events=events)
    cpp = write_run(tmp_path / "cpp", [result("cpp", repeat=i) for i in (1, 2, 3)], events=events)
    report = compare.build_report([("python", py), ("cpp", cpp)], acceptance_aggregation="scenario_mean",
        rmse_metric="source_time_trapezoid", frame_contract="otter_fake_localization")
    assert report["scenario_acceptance"]["summary"]["all_eligible_passed"]
    assert report["summary"]["performance_unknown"] == 3
    assert report["summary"]["all_passed"] is False
    output = compare.markdown(report)
    for label in ("scenario_mean", "source_time_trapezoid", "算术均值", "ddof=1", "最差", "逐次"):
        assert label in output
    with pytest.raises(ValueError, match="aggregation"):
        compare.build_report([("python", py)], acceptance_aggregation="best_two")
    assert compare.main(["--python", str(py), "--baseline", str(cpp),
        "--rmse-metric", "source_time_trapezoid", "--frame-contract", "otter_fake_localization",
        "--acceptance-aggregation", "scenario_mean", "--output", str(tmp_path / "mean")]) == 1
    written = json.loads((tmp_path / "mean" / "report.json").read_text())
    assert written["scenario_acceptance"] == report["scenario_acceptance"]


# 六组中两组可比通过、四组不可比应分别统计；不可比组仍保留三次数据和全部 18 条逐次比较。
def test_scenario_mean_two_eligible_four_incomparable_preserves_all_individuals():
    import copy
    rows = []
    for controller in ("pid", "lqr"):
        for scenario in ("straight", "turn", "single_obstacle"):
            group = copy.deepcopy(scenario_fixture()["trials"])
            for record in group:
                row = record["result"]
                if controller == "lqr" and row["mode"] == "cpp":
                    continue
                row.update(controller=controller, scenario=scenario,
                    trial=f"{row['mode']}_{controller}_{scenario}_{row['repeat']:02d}")
                if scenario == "single_obstacle" and row["mode"] == "cpp" and row["repeat"] in (2, 3):
                    row["passed"] = False
                rows.append(record)
    report = refresh_scenario_fixture(rows)
    output = compare.scenario_mean_acceptance(report)
    assert output["summary"] == {"groups": 6, "eligible_groups": 2, "passed_groups": 2,
        "failed_groups": 0, "incomparable_groups": 4,
        "all_eligible_passed": True, "all_groups_comparable": False}
    circle = next(group for group in output["groups"] if group["controller"] == "pid" and group["scenario"] == "single_obstacle")
    assert circle["cpp"]["metrics"]["cross_track_rmse_m"]["count"] == 3
    assert circle["individual_comparisons"][0]["comparable"]
    assert sum(len(group["individual_comparisons"]) for group in output["groups"]) == 18


# 负值、NaN、无穷、布尔和 None 都不是有效源时间 RMSE，输出 None/未知并保证严格 JSON 可序列化。
@pytest.mark.parametrize("invalid", [-1., float("nan"), float("inf"), True, None])
def test_scenario_mean_invalid_values_remain_unknown(invalid):
    report = scenario_fixture()
    report["trials"][0]["observations"]["tracking"]["source_time_trapezoid"]["rmse_m"] = invalid
    output = compare.scenario_mean_acceptance(report)
    assert output["groups"][0]["passed"] is None
    assert output["groups"][0]["python"]["metrics"]["cross_track_rmse_m"]["values"][0] is None
    json.dumps(output, allow_nan=False)
