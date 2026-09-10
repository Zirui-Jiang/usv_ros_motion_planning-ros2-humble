#!/usr/bin/env python3
"""Summarize observed ROS acceptance data without modifying raw runs.

Typical invocation (one completed run per implementation):
  python3 compare_results.py --python /path/python_run --baseline /path/cpp_run \
      --extra /path/fault_run --output /path/new_report

Repeat --python/--baseline to provide several runs. Conflicting trial IDs or
semantic keys require --duplicates latest or --duplicates provided-list.
Latest uses manifest.created_unix_s, falling back explicitly to summary mtime;
provided-list selects the last directory in the recorded source order.
--extra consumes fault cases only. Report output must be new and outside raw
run directories. Exit 0 means all requested observed/planned navigation pairs
passed, sources are finished, and extra fault cases passed; 1 means otherwise.
Exit 2 denotes invalid inputs. No ROS or NumPy dependency is needed.
"""

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys


AXES = ("left", "right", "lateral")
MAPPING_METRICS = ("processing_ms", "queue_ms", "serialization_and_publish_ms", "observed_scan_period_ms")


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def nested(value, *keys):
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def read_json(path):
    def nonfinite(token):
        raise ValueError(f"nonfinite JSON value {token} in {path}")
    with Path(path).open(encoding="utf-8-sig") as stream:
        return json.load(stream, parse_constant=nonfinite)


def metric_summary(values):
    values = [float(v) for v in values if number(v)]
    return {"count": len(values), "mean": statistics.fmean(values) if values else None,
            "min": min(values) if values else None, "max": max(values) if values else None}


def trial_key(row):
    """Faults have a separate key and never enter navigation ratio matching."""
    return (row.get("mode"), row.get("controller"), row.get("scenario"), row.get("repeat"), row.get("fault"))


def comparison_key(row):
    return (row.get("controller"), row.get("scenario"), row.get("repeat"))


def ratio_comparison(python_value, baseline_value, limit=1.10, atol=1e-6):
    result = {"python": python_value, "cpp": baseline_value, "ratio": None,
              "limit": limit, "zero_baseline_atol": atol, "comparable": False,
              "passed": None, "reason": ""}
    if not all(number(v) and v >= 0 for v in (python_value, baseline_value)):
        result["reason"] = "missing_or_invalid_metric"
        return result
    result["comparable"] = True
    if baseline_value == 0:
        # Never invent a ratio (including 0/0). Absolute error is the entire
        # acceptance rule for an exactly zero baseline.
        result.update(passed=abs(python_value) <= atol, method="zero_baseline_absolute_tolerance",
                      reason="zero_baseline_ratio_undefined")
    else:
        ratio = python_value / baseline_value
        result.update(ratio=ratio if math.isfinite(ratio) else None,
                      passed=python_value <= limit * baseline_value, method="python_divided_by_cpp")
        if not math.isfinite(ratio):
            result["reason"] = "ratio_overflow"
    return result


def process_metrics(samples):
    """CPU percent uses only same-PID, monotonic, non-reset intervals."""
    groups = defaultdict(list)
    invalid = 0
    for sample in samples:
        if not isinstance(sample, dict) or not isinstance(sample.get("pid"), int):
            invalid += 1
            continue
        groups[sample["pid"]].append(sample)
    processes = []
    for pid, observations in sorted(groups.items()):
        previous = None
        cpu_delta = wall_delta = 0.0
        percentages, peaks = [], []
        rejected = 0
        for sample in observations:
            peak = sample.get("peak_rss_kib")
            if number(peak) and peak >= 0:
                peaks.append(peak)
            cpu, wall = sample.get("cpu_seconds"), sample.get("monotonic")
            if not number(cpu) or cpu < 0 or not number(wall):
                rejected += 1
                previous = None
                continue
            if previous is not None:
                dcpu, dwall = cpu - previous[0], wall - previous[1]
                if dcpu == 0 and dwall == 0:
                    continue  # Final summary may repeat the last status event.
                if dcpu < 0 or dwall <= 0:
                    rejected += 1
                else:
                    cpu_delta += dcpu
                    wall_delta += dwall
                    percentages.append(100.0 * dcpu / dwall)
            previous = (cpu, wall)
        processes.append({"pid": pid, "samples": len(observations), "valid_intervals": len(percentages),
                          "rejected_intervals_or_samples": rejected,
                          "cpu_delta_seconds": cpu_delta if percentages else None,
                          "monotonic_delta_seconds": wall_delta if percentages else None,
                          "mean_cpu_percent": 100.0 * cpu_delta / wall_delta if percentages else None,
                          "max_interval_cpu_percent": max(percentages) if percentages else None,
                          "peak_rss_kib": max(peaks) if peaks else None})
    valid = [p for p in processes if p["valid_intervals"]]
    cpu = sum(p["cpu_delta_seconds"] for p in valid)
    wall = sum(p["monotonic_delta_seconds"] for p in valid)
    peaks = [p["peak_rss_kib"] for p in processes if p["peak_rss_kib"] is not None]
    return {"per_pid": processes, "invalid_process_samples": invalid,
            "mean_cpu_percent": 100 * cpu / wall if wall else None,
            "peak_rss_kib": max(peaks) if peaks else None,
            "valid_intervals": sum(p["valid_intervals"] for p in processes)}


def event_metrics(path, summary):
    commands = {axis: [] for axis in AXES}
    invalid_commands = defaultdict(int)
    statuses = {"navigation_status": [], "mapping_status": []}
    process_samples = {key: [] for key in statuses}
    event_count = 0
    issues = []
    if path.is_file():
        with path.open(encoding="utf-8-sig") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    event = json.loads(line, parse_constant=lambda token: (_ for _ in ()).throw(ValueError("nonfinite JSON token: " + token)))
                except json.JSONDecodeError as error:
                    raise ValueError(f"invalid JSONL {path}:{line_number}: {error}") from error
                if not isinstance(event, dict):
                    raise ValueError(f"event must be an object: {path}:{line_number}")
                event_count += 1
                kind = event.get("event")
                if kind == "thruster_command" and event.get("axis") in AXES:
                    axis, value = event["axis"], event.get("normalized_command")
                    if number(value):
                        commands[axis].append(float(value))
                    else:
                        invalid_commands[axis] += 1
                elif kind in statuses and isinstance(event.get("payload"), dict):
                    payload = event["payload"]
                    statuses[kind].append(payload)
                    if isinstance(payload.get("process"), dict):
                        process_samples[kind].append(payload["process"])
    else:
        issues.append("events_jsonl_missing")
    final_status = {}
    final_sources = {}
    for event_name, summary_name in (("navigation_status", "status"), ("mapping_status", "mapping_status")):
        payload = summary.get(summary_name)
        if isinstance(payload, dict):
            final_status[event_name], final_sources[event_name] = payload, "summary." + summary_name
            if isinstance(payload.get("process"), dict):
                process_samples[event_name].append(payload["process"])
        else:
            final_status[event_name] = statuses[event_name][-1] if statuses[event_name] else None
            final_sources[event_name] = "events.last_" + event_name if statuses[event_name] else None
    variation = {}
    for axis, values in commands.items():
        tv = sum(abs(b - a) for a, b in zip(values[:-1], values[1:])) if len(values) >= 2 and not invalid_commands[axis] else None
        variation[axis] = {"samples": len(values), "invalid_samples": invalid_commands[axis],
                           "observed_transitions": max(0, len(values) - 1), "total_variation": tv}
    total = sum(variation[a]["total_variation"] for a in AXES) if all(variation[a]["total_variation"] is not None for a in AXES) else None
    nav, mapping = final_status["navigation_status"], final_status["mapping_status"]
    flat = {"control_p99_ms": nested(nav, "metrics", "control_ms", "p99"),
            "planning_p99_ms": nested(nav, "metrics", "planning_ms", "p99")}
    for name in ("control", "planning"):
        values = [nested(s, "metrics", name + "_ms", "p99") for s in statuses["navigation_status"]]
        values.append(flat[name + "_p99_ms"])
        flat[name + "_max_reported_p99_ms"] = metric_summary(values)["max"]
    for name in MAPPING_METRICS:
        flat["mapping_" + name.removesuffix("_ms") + "_p99_ms"] = nested(mapping, name, "p99")
    cpu = {name: process_metrics(samples) for name, samples in process_samples.items()}
    for event_name, label in (("navigation_status", "navigation"), ("mapping_status", "mapping")):
        flat[label + "_cpu_percent"] = cpu[event_name]["mean_cpu_percent"]
        flat[label + "_peak_rss_kib"] = cpu[event_name]["peak_rss_kib"]
    flat["thruster_total_variation"] = total
    flat.update({"thruster_" + axis + "_total_variation": variation[axis]["total_variation"] for axis in AXES})
    return {"events_path": str(path.resolve()), "events_present": path.is_file(), "event_count": event_count,
            "issues": issues, "thrusters": {"axes": variation, "total_variation": total},
            "processes": cpu, "status": nav, "mapping_status": mapping,
            "status_sources": final_sources, "metrics": flat}


def load_run(directory, role, order):
    directory = Path(directory).resolve()
    if not directory.is_dir():
        raise ValueError(f"run directory does not exist: {directory}")
    aggregate_path = directory / "summary.json"
    manifest_path = directory / "manifest.json"
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    aggregate = read_json(aggregate_path) if aggregate_path.is_file() else {}
    if not isinstance(aggregate, dict) or not isinstance(manifest, dict):
        raise ValueError(f"summary and manifest must be objects: {directory}")
    if "results" in aggregate:
        rows = aggregate["results"]
        if not isinstance(rows, list):
            raise ValueError(f"results must be a list: {aggregate_path}")
    elif "trial" in aggregate:
        rows = [aggregate]
    else:
        rows = []
    observed = {}
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("trial"), str) or not row["trial"]:
            raise ValueError(f"invalid trial result in {aggregate_path}")
        if row["trial"] in (".", "..") or "/" in row["trial"] or "\\" in row["trial"]:
            raise ValueError(f"trial ID must be a single directory name: {aggregate_path}")
        if row["trial"] in observed:
            raise ValueError(f"duplicate trial inside one summary: {row['trial']} in {aggregate_path}")
        observed[row["trial"]] = (row, aggregate_path)
    # Interrupted runs may have a last child summary absent from aggregate.
    if "trial" not in aggregate:
        for child in sorted(directory.glob("*/summary.json")):
            row = read_json(child)
            if not isinstance(row, dict) or not isinstance(row.get("trial"), str):
                continue
            if not row["trial"] or row["trial"] in (".", "..") or "/" in row["trial"] or "\\" in row["trial"]:
                raise ValueError(f"trial ID must be a single directory name: {child}")
            if row["trial"] in observed and observed[row["trial"]][0] != row:
                raise ValueError(f"aggregate/child trial summaries disagree: {child}")
            observed[row["trial"]] = (row, child)
    created = manifest.get("created_unix_s")
    basis = "manifest.created_unix_s"
    if not number(created):
        basis = "summary_mtime_unix_s"
        paths = [item[1] for item in observed.values()]
        created = aggregate_path.stat().st_mtime if aggregate_path.is_file() else max((p.stat().st_mtime for p in paths), default=None)
    def allowed(row):
        return bool(row.get("fault")) if role == "extra" else row.get("mode") == role
    selected, ignored = [], []
    for row, file in observed.values():
        if not allowed(row):
            ignored.append(row["trial"])
            continue
        if row.get("mode") not in ("python", "cpp"):
            raise ValueError(f"unsupported mode in {file}: {row.get('mode')}")
        if not row.get("fault") and (not isinstance(row.get("controller"), str) or not isinstance(row.get("scenario"), str)
                                     or type(row.get("repeat")) is not int or row["repeat"] < 1):
            raise ValueError(f"navigation matching fields missing/invalid in {file}")
        events_path = file.parent / "events.jsonl" if file.parent != directory or "trial" in aggregate else directory / row["trial"] / "events.jsonl"
        selected.append({"result": row, "source": {"run_directory": str(directory), "summary_path": str(file),
                        "source_order": order, "created_unix_s": created, "latest_basis": basis,
                        "summary_sha256": hashlib.sha256(file.read_bytes()).hexdigest(),
                        "manifest_mode": manifest.get("mode"),
                        "git_commit": manifest.get(row["mode"] + "_git_commit")},
                        "observations": event_metrics(events_path, row)})
    planned = [p for p in manifest.get("matrix", []) if isinstance(p, dict) and allowed(p) and not p.get("fault")]
    metadata = {"directory": str(directory), "role": role, "source_order": order,
                "created_unix_s": created, "latest_basis": basis,
                "manifest_mode": manifest.get("mode"), "run_finished": aggregate.get("run_finished"),
                "interrupted": aggregate.get("interrupted"), "summary_present": aggregate_path.is_file(),
                "selected_trials": len(selected), "ignored_trials": ignored,
                "planned_navigation_trials": len(planned),
                "missing_planned_trials": [p for p in planned if trial_key(p) not in {trial_key(s['result']) for s in selected}]}
    return selected, planned, metadata


def select_duplicates(records, policy):
    chosen = []
    decisions = []
    for record in records:
        row = record["result"]
        conflicts = [r for r in chosen if r["result"]["trial"] == row["trial"] or trial_key(r["result"]) == trial_key(row)]
        if not conflicts:
            chosen.append(record)
            continue
        if policy == "error":
            raise ValueError(f"duplicate trial {row['trial']}; choose --duplicates latest or provided-list explicitly")
        candidates = [*conflicts, record]
        if policy == "latest":
            if any(c["source"]["created_unix_s"] is None for c in candidates):
                raise ValueError(f"cannot select latest without timestamps: {row['trial']}")
            winner = max(candidates, key=lambda c: (c["source"]["created_unix_s"], c["source"]["source_order"]))
        else:
            winner = max(candidates, key=lambda c: c["source"]["source_order"])
        decisions.append({"trial": row["trial"], "policy": policy, "selected": winner["source"],
                          "discarded": [c["source"] for c in candidates if c is not winner]})
        chosen = [c for c in chosen if c not in conflicts]
        chosen.append(winner)
    return chosen, decisions


def completed_pass(record):
    return (record is not None and record["result"].get("result") == "completed"
            and record["result"].get("passed") is True and record["source"].get("manifest_mode") != "dry_run")


def compare_pair(key, python_record, cpp_record, limit, atol):
    pair = {"controller": key[0], "scenario": key[1], "repeat": key[2],
            "comparable": False, "passed": None, "reasons": [], "metrics": {},
            "python_source": python_record["source"] if python_record else None,
            "cpp_source": cpp_record["source"] if cpp_record else None}
    for name, record in (("python", python_record), ("cpp", cpp_record)):
        if record is None:
            pair["reasons"].append(name + "_result_missing")
        elif not completed_pass(record):
            row = record["result"]
            pair["reasons"].append(f"{name}_not_completed_and_passed: result={row.get('result')}, passed={row.get('passed')}, manifest_mode={record['source'].get('manifest_mode')}, reason={row.get('reason') or row.get('reasons')}")
    if pair["reasons"]:
        return pair
    py, cpp = python_record["result"], cpp_record["result"]
    pair["metrics"] = {
        "cross_track_rmse_m": ratio_comparison(nested(py, "cross_track_initial_m", "rms"), nested(cpp, "cross_track_initial_m", "rms"), limit, atol),
        "duration_sim_s": ratio_comparison(py.get("duration_sim_s"), cpp.get("duration_sim_s"), limit, atol)}
    pair["comparable"] = all(m["comparable"] for m in pair["metrics"].values())
    if pair["comparable"]:
        pair["passed"] = all(m["passed"] for m in pair["metrics"].values())
    else:
        pair["reasons"] = [name + ": " + value["reason"] for name, value in pair["metrics"].items() if not value["comparable"]]
    return pair


def group_results(records, planned):
    groups = defaultdict(list)
    expected = defaultdict(set)
    for record in records:
        row = record["result"]
        groups[(row.get("mode"), row.get("controller"), row.get("scenario"), row.get("fault"))].append(record)
    for row in planned:
        key = (row.get("mode"), row.get("controller"), row.get("scenario"), None)
        expected[key].add(row.get("repeat"))
        groups.setdefault(key, [])
    output = []
    for key, members in sorted(groups.items(), key=lambda x: str(x[0])):
        completed = [r for r in members if r["result"].get("result") == "completed"]
        metrics = defaultdict(list)
        counters = defaultdict(list)
        for record in completed:
            row, observed = record["result"], record["observations"]
            metrics["cross_track_rmse_m"].append(nested(row, "cross_track_initial_m", "rms"))
            metrics["duration_sim_s"].append(row.get("duration_sim_s"))
            metrics["minimum_footprint_clearance_m"].append(row.get("minimum_footprint_clearance_m"))
            for name, value in observed["metrics"].items():
                metrics[name].append(value)
            mapping = observed["mapping_status"] or {}
            for name in ("integrated_scans", "scan_overruns", "invalid_scans", "invalid_beams", "queue_size"):
                counters[name].append(mapping.get(name))
            for name, value in (mapping.get("queue_counters") or {}).items():
                counters["queue_counters." + name].append(value)
        output.append({"mode": key[0], "controller": key[1], "scenario": key[2], "fault": key[3],
                       "observed_trials": len(members), "observed_repeats": sorted({r['result']['repeat'] for r in members if 'repeat' in r['result']}),
                       "expected_repeats": sorted(expected[key]), "completed_trials": len(completed),
                       "passed_trials": sum(completed_pass(r) for r in members),
                       "results": {name: sum(r['result'].get('result') == name for r in members)
                                   for name in sorted({r['result'].get('result', 'missing') for r in members})},
                       "metrics_population": "all completed observed trials, including failed acceptance trials; no imputation",
                       "metrics": {name: metric_summary(values) for name, values in metrics.items()},
                       "mapping_final_counters": {name: {**metric_summary(values),
                           "sum_across_observed_final_snapshots": sum(v for v in values if number(v)) if name != "queue_size" and any(number(v) for v in values) else None}
                           for name, values in counters.items()}})
    return output


def build_report(sources, duplicate_policy="error", limit=1.10, atol=1e-6):
    if duplicate_policy not in ("error", "latest", "provided-list"):
        raise ValueError("unknown duplicate policy")
    if not number(limit) or limit <= 0 or not number(atol) or atol < 0:
        raise ValueError("limit must be positive and atol nonnegative")
    records, planned, metadata = [], [], []
    for index, (role, directory) in enumerate(sources):
        found, expected, info = load_run(directory, role, index)
        records.extend(found); planned.extend(expected); metadata.append(info)
    records, decisions = select_duplicates(records, duplicate_policy)
    navigation = [r for r in records if not r["result"].get("fault")]
    by_pair = defaultdict(dict)
    for record in navigation:
        by_pair[comparison_key(record["result"])][record["result"]["mode"]] = record
    for row in planned:
        by_pair.setdefault(comparison_key(row), {})
    comparisons = [compare_pair(key, modes.get("python"), modes.get("cpp"), limit, atol)
                   for key, modes in sorted(by_pair.items(), key=lambda x: str(x[0]))]
    faults = [r for r in records if r["result"].get("fault")]
    sources_finished = all(m["run_finished"] is True and m["manifest_mode"] != "dry_run" for m in metadata)
    all_passed = bool(comparisons) and all(p["passed"] is True for p in comparisons) and all(completed_pass(f) for f in faults) and sources_finished
    return {"schema_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
            "selection": {"duplicate_policy": duplicate_policy, "source_order": "Python directories, baseline directories, extra directories; later source wins provided-list/timestamp ties", "decisions": decisions},
            "method": {"rmse_field": "cross_track_initial_m.rms", "duration_field": "duration_sim_s",
                       "ratio_limit": limit, "zero_baseline_atol": atol,
                       "zero_baseline_rule": "exactly zero cpp: ratio=null; pass iff abs(python)<=atol",
                       "p99_aggregation": "mean/max of reported per-trial P99, not a pooled sample percentile",
                       "cpu": "100*sum(nonnegative same-PID cpu_seconds deltas)/sum(positive same-PID process.monotonic deltas); not machine CPU and excludes planner child/Gazebo",
                       "thruster_variation": "sum(abs(command[i]-command[i-1])) per axis in full recorded JSONL order; no initial/final zero appended; total requires all axes and >=2 valid samples each",
                       "counter_scope": "final reported cumulative counters per completed trial, not sums of status event counters"},
            "sources": metadata, "trials": records, "groups": group_results(records, planned),
            "comparisons": comparisons, "fault_trials": [f["result"]["trial"] for f in faults],
            "summary": {"pairs": len(comparisons), "comparable_pairs": sum(p["comparable"] for p in comparisons),
                        "passed_pairs": sum(p["passed"] is True for p in comparisons),
                        "failed_pairs": sum(p["passed"] is False for p in comparisons),
                        "not_comparable_pairs": sum(not p["comparable"] for p in comparisons),
                        "fault_trials": len(faults), "fault_passed": sum(completed_pass(f) for f in faults),
                        "all_sources_finished": sources_finished, "all_passed": all_passed}}


def markdown(report):
    def fmt(value):
        return f"{value:.6g}" if number(value) else "缺失/不可比"
    def safe(value):
        return str(value).replace("|", "\\|").replace("\n", " ")
    summary = report["summary"]
    lines = ["# Python/C++ 导航结果对照", "",
             f"总体判定：{'通过' if summary['all_passed'] else '未通过或证据不完整'}。可比 {summary['comparable_pairs']}/{summary['pairs']} 对，通过 {summary['passed_pairs']} 对，失败 {summary['failed_pairs']} 对，不可比 {summary['not_comparable_pairs']} 对；故障用例通过 {summary['fault_passed']}/{summary['fault_trials']}。",
             "", f"仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ {report['method']['ratio_limit']:.2f}；零基线比例不定义，按绝对容差 {report['method']['zero_baseline_atol']:g} 判断。",
             "", "## 每次配对", "", "| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |", "| --- | --- | --- | --- | --- | --- |"]
    for pair in report["comparisons"]:
        values = []
        for name in ("cross_track_rmse_m", "duration_sim_s"):
            metric = pair["metrics"].get(name, {})
            values.append("零基线绝对容差：" + ("通过" if metric.get('passed') else "失败") if metric.get("method") == "zero_baseline_absolute_tolerance" else fmt(metric.get("ratio")))
        verdict = "通过" if pair["passed"] is True else "失败" if pair["passed"] is False else "不可比：" + "; ".join(pair["reasons"])
        lines.append(f"| {safe(pair['controller'])} | {safe(pair['scenario'])} | {pair['repeat']} | {values[0]} | {values[1]} | {safe(verdict)} |")
    lines += ["", "## 重复试验汇总", "", "统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。", "",
              "| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
    for group in report["groups"]:
        metrics = group["metrics"]
        def stat(name, field): return fmt(nested(metrics, name, field))
        label = "/".join(str(group[k]) for k in ("mode", "controller", "scenario")) + ("/" + group['fault'] if group['fault'] else "")
        lines.append(f"| {safe(label)} | {group['observed_trials']}/{group['completed_trials']}/{group['passed_trials']} | {stat('cross_track_rmse_m','mean')}/{stat('cross_track_rmse_m','max')} | {stat('duration_sim_s','mean')}/{stat('duration_sim_s','max')} | {stat('minimum_footprint_clearance_m','min')} | {stat('control_p99_ms','mean')}/{stat('control_p99_ms','max')}；{stat('planning_p99_ms','mean')}/{stat('planning_p99_ms','max')} | {stat('thruster_total_variation','mean')} |")
    lines += ["", "## 进程与建图观测", "", "CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。", "",
              "| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |", "| --- | --- | --- | --- | --- |"]
    mapping_notes = []
    for record in report["trials"]:
        row, observed = record["result"], record["observations"]
        m = observed["metrics"]
        tv = "/".join(fmt(m['thruster_' + a + '_total_variation']) for a in AXES)
        lines.append(f"| {safe(row['trial'])} | {fmt(m['navigation_cpu_percent'])}/{fmt(m['navigation_peak_rss_kib'])} | {fmt(m['mapping_cpu_percent'])}/{fmt(m['mapping_peak_rss_kib'])} | {fmt(m['mapping_processing_p99_ms'])}/{fmt(m['mapping_queue_p99_ms'])} | {tv} |")
        if isinstance(observed["mapping_status"], dict):
            counters = {key: observed['mapping_status'].get(key) for key in ('integrated_scans','scan_overruns','invalid_scans','invalid_beams','queue_size','queue_counters')}
            mapping_notes.append(f"- `{safe(row['trial'])}` 建图最后观测计数：`{json.dumps(counters, ensure_ascii=False)}`。")
    lines += ["", *mapping_notes]
    lines += ["", "## 来源与选择", "", f"重复策略：`{report['selection']['duplicate_policy']}`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。", ""]
    for source in report["sources"]:
        lines.append(f"- `{source['directory']}`：角色 {source['role']}，run_finished={source['run_finished']}，选中 {source['selected_trials']} 条，未观测计划 {len(source['missing_planned_trials'])} 条。")
    for decision in report["selection"]["decisions"]:
        lines.append(f"- 重复 `{decision['trial']}` 选择 `{decision['selected']['run_directory']}`；排除 " + ", ".join('`' + d['run_directory'] + '`' for d in decision['discarded']) + "。")
    lines += ["", "逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。"]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--python", type=Path, action="append", default=[], help="Python run directory; may repeat explicitly")
    parser.add_argument("--baseline", type=Path, action="append", default=[], help="C++ run directory; may repeat explicitly")
    parser.add_argument("--extra", type=Path, nargs="+", action="append", default=[], help="additional fault-only run directories")
    parser.add_argument("--duplicates", choices=("error", "latest", "provided-list"), default="error")
    parser.add_argument("--ratio-limit", type=float, default=1.10)
    parser.add_argument("--atol", type=float, default=1e-6)
    parser.add_argument("--output", type=Path, required=True, help="new report directory outside all raw input runs")
    args = parser.parse_args(argv)
    sources = [("python", p) for p in args.python] + [("cpp", p) for p in args.baseline] + [("extra", p) for batch in args.extra for p in batch]
    if not sources:
        parser.error("provide at least one --python, --baseline or --extra directory")
    output = args.output.resolve()
    try:
        if output.exists() or any(output == p.resolve() or p.resolve() in output.parents for _, p in sources):
            raise ValueError("output must be new and outside raw run directories")
        report = build_report(sources, args.duplicates, args.ratio_limit, args.atol)
        output.mkdir(parents=True, exist_ok=False)
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        (output / "report.md").write_text(markdown(report), encoding="utf-8")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps({"report": str(output / 'report.json'), **report['summary']}, ensure_ascii=False))
    return 0 if report["summary"]["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
