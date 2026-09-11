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
passed, Python navigation performance has sufficient passing evidence, sources
are finished, and extra fault cases passed; 1 means otherwise.
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
MAPPING_REPORT_PERIOD_S = 5.0


def verdict(status, reason, **evidence):
    return {"status": status, "passed": True if status == "pass" else False if status == "fail" else None,
            "reason": reason, **evidence}


def combined_status(items):
    states = [item["status"] for item in items]
    return "fail" if "fail" in states else "pass" if states and all(s == "pass" for s in states) else "unknown"


def post_drain_queue_evidence(records):
    """Validate the new producer contract without treating survivor ages as proof alone.

    These are sampled post-drain windows, not continuous queue maxima. The
    caller separately requires processing progress and balanced/drop counters.
    Missing or mixed old/new telemetry must not fall back to a permissive rule.
    """
    windows, seen, configurations = [], set(), set()
    for record in records:
        payload = record["payload"]
        mono = nested(payload, "process", "monotonic")
        identity = (nested(payload, "process", "pid"), mono)
        if number(mono) and identity in seen:
            continue
        if number(mono):
            seen.add(identity)
        observed = payload.get("queue_observation_window")
        waiting = payload.get("queue_ms")
        issues = []
        detail = {"source": record["source"], "process_monotonic_s": mono,
                  "observation": observed, "completed_scan_waiting_ms": waiting, "issues": issues}
        windows.append(detail)
        if not isinstance(observed, dict):
            issues.append("missing_queue_observation_window")
            continue

        def integer(value):
            return type(value) is int and value >= 0

        def distribution(value, count=None):
            if not isinstance(value, dict) or not integer(value.get("count")):
                return False
            if count is not None and value["count"] != count:
                return False
            if value["count"] == 0:
                return count == 0
            values = [value.get(k) for k in ("p50", "p95", "p99", "max")]
            return all(number(v) and v >= 0 for v in values) and values == sorted(values)

        capacity, deadline = observed.get("capacity"), observed.get("tf_wait_timeout_s")
        duration = observed.get("window_s")
        if (not integer(capacity) or capacity < 1 or not number(deadline) or deadline <= 0
                or not number(duration) or duration <= 0):
            issues.append("invalid_queue_configuration")
            continue
        configurations.add((capacity, deadline, duration))
        if not math.isclose(duration, MAPPING_REPORT_PERIOD_S, rel_tol=0, abs_tol=1e-6):
            issues.append("unsupported_queue_observation_window_duration")
        if observed.get("sampling_phase") != "after_drain":
            issues.append("unsupported_queue_observation_phase")
        count, pending, empty = (observed.get(k) for k in
                                 ("observation_count", "pending_observations", "empty_observations"))
        if (not all(integer(v) for v in (count, pending, empty)) or count < 2
                or pending + empty != count):
            issues.append("invalid_or_insufficient_observation_counts")
            continue
        first, last, span, gap = (observed.get(k) for k in
                                  ("first_observed_monotonic_s", "last_observed_monotonic_s",
                                   "observed_span_s", "max_observation_gap_ms"))
        if (not all(number(v) for v in (mono, first, last, span, gap)) or first > last
                or gap < 0 or not math.isclose(span, last - first, rel_tol=1e-9, abs_tol=1e-6)
                or span > duration + 1e-6
                # The producer records process.monotonic after computing its
                # summary. Permit only the same bounded publication age used
                # for the last observation, not an exact timestamp equality.
                or first < mono - duration - deadline - 1e-6 or last > mono + 1e-6
                or span > (count - 1) * gap / 1000 + 1e-6):
            issues.append("invalid_observation_time_bounds")
        elif (span < 0.8 * duration or gap > 1000 * deadline
              or mono - last > deadline or count < math.ceil(0.8 * duration / deadline) + 1):
            issues.append("insufficient_dense_observation_coverage")
        maximum, latest = observed.get("max_queue_size"), observed.get("latest_queue_size")
        if (not integer(maximum) or not integer(latest) or latest > maximum or maximum > capacity
                or (pending == 0 and maximum != 0) or (pending > 0 and maximum == 0)):
            issues.append("invalid_observed_queue_sizes")
        ages = observed.get("oldest_pending_age_ms")
        if not distribution(ages, pending):
            issues.append("missing_or_invalid_pending_age_distribution")
        elif pending and ages["max"] > 1000 * deadline:
            issues.append("observed_pending_age_exceeds_tf_deadline")
        distinct = observed.get("oldest_pending_distinct_count")
        if not integer(distinct) or distinct > pending or (pending > 0 and distinct < 1):
            issues.append("invalid_pending_identity_count")
        elif empty == 0 and distinct < 2:
            issues.append("nonempty_window_without_observed_identity_turnover")
        oldest, latest_age = observed.get("latest_oldest_pending"), observed.get("latest_oldest_pending_age_ms")
        if latest == 0:
            if oldest is not None or latest_age is not None:
                issues.append("empty_latest_queue_has_pending_identity")
        elif integer(latest) and latest > 0:
            received = nested(oldest, "received_at_monotonic_s")
            if (not isinstance(oldest, dict) or not isinstance(oldest.get("frame_id"), str)
                    or not oldest["frame_id"] or type(oldest.get("stamp_ns")) is not int
                    or oldest["stamp_ns"] <= 0 or not number(received) or not number(last)
                    or received > last or not number(latest_age) or latest_age < 0
                    or not math.isclose(latest_age, 1000 * (last - received), rel_tol=1e-9, abs_tol=1e-6)):
                issues.append("invalid_latest_pending_identity_or_age")
            elif (latest_age > 1000 * deadline or not distribution(ages, pending)
                  or latest_age > ages.get("max", -1) + 1e-6):
                issues.append("latest_pending_age_conflicts_with_window_distribution")
        # Waiting samples include completed scans only. Require their latency
        # too, but never use it to erase overflow/expiry or a missing live gauge.
        if not distribution(waiting) or not waiting.get("count"):
            issues.append("missing_completed_scan_waiting_distribution")
        elif waiting["max"] > 1000 * deadline:
            issues.append("completed_scan_waiting_exceeds_tf_deadline")
    details = {"reported_windows": len(windows), "windows": windows,
               "minimum_window_coverage_fraction": 0.8,
               "maximum_observation_gap": "configured TF waiting deadline",
               "latency_budget": "configured TF waiting deadline; wall time, not scan period",
               "sampling_scope": "post-drain observations only; maxima omit pre-drain peaks and scans already expired",
               "configuration_consistent": len(configurations) == 1}
    if not windows or len(configurations) != 1 or any(w["issues"] for w in windows):
        return verdict("unknown", "insufficient_post_drain_latency_or_turnover_evidence", **details)
    return verdict("pass", "dense_post_drain_windows_with_bounded_waiting_and_identity_turnover", **details)


def backlog_performance(records, window):
    """Finite observation-window evidence, never infer continuous fullness from two gauges."""
    samples, missing, seen, sample_issues, schemas = [], 0, set(), [], set()
    required = ("accepted", "processed", "overflow", "expired", "invalid", "duplicate", "resets")
    for record in records:
        payload, stamp = record["payload"], record["wall_monotonic_s"]
        pid = nested(payload, "process", "pid")
        identity = (pid, nested(payload, "process", "monotonic"))
        if number(identity[1]) and identity in seen:
            continue
        if number(identity[1]):
            seen.add(identity)
        counts = payload.get("queue_counters")
        queue, integrated = payload.get("queue_size"), payload.get("integrated_scans")
        invalid_scans = payload.get("invalid_scans")
        new_schema = isinstance(counts, dict) and "invalid_consumed" in counts
        schema = "successful_integrations_v2" if new_schema else "consumed_scans_v1"
        if isinstance(counts, dict):
            schemas.add(schema)
        keys = required + (("invalid_consumed",) if new_schema else ())
        if (type(pid) is not int or not number(stamp) or type(queue) is not int or queue < 0
                or type(integrated) is not int or integrated < 0
                or type(invalid_scans) is not int or invalid_scans < 0 or not isinstance(counts, dict)
                or any(type(counts.get(k)) is not int or counts[k] < 0 for k in keys)):
            missing += 1
            sample_issues.append({"source": record["source"], "schema": schema,
                                  "reason": "missing_or_invalid_nonnegative_integer_counter"})
            continue
        # Old processed counted every consumed callback (including invalid
        # scans); v2 counts only actual integrations and separates invalids.
        # Neither schema counts admission-time invalid/duplicate input as
        # accepted work. Missing measurements are never inferred as zero.
        processing_consistent = (counts["processed"] == integrated
                                 and counts["invalid_consumed"] == invalid_scans) if new_schema else (
                                     counts["processed"] == integrated + invalid_scans)
        consumed_invalid = counts["invalid_consumed"] if new_schema else 0
        # Reset clears pending entries without a dropped-on-reset counter.
        # Across such epochs only interval conservation is observable; do not
        # assert an unsupported lifetime balance or hide an in-window reset.
        lifetime_balanced = (counts["resets"] > 0 or counts["accepted"] - counts["processed"]
                             - consumed_invalid - counts["expired"] - counts["overflow"] == queue)
        if not processing_consistent or not lifetime_balanced:
            missing += 1
            sample_issues.append({"source": record["source"], "schema": schema,
                                  "reason": "inconsistent_scan_counter_accounting",
                                  "processing_consistent": processing_consistent,
                                  "lifetime_balanced_or_unobservable_after_reset": lifetime_balanced})
            continue
        samples.append({"wall_monotonic_s": stamp, "pid": pid, "queue_size": queue,
                        "integrated_scans": integrated, "queue_counters": counts,
                        "invalid_scans": invalid_scans, "counter_schema": schema,
                        "source": record["source"]})
    intervals, gaps, invalid = [], 0, 0
    overflow_run = 0
    sustained_overflow = False
    for previous, current in zip(samples[:-1], samples[1:]):
        duration = current["wall_monotonic_s"] - previous["wall_monotonic_s"]
        a, b = previous["queue_counters"], current["queue_counters"]
        same_epoch = previous["pid"] == current["pid"] and a["resets"] == b["resets"]
        same_schema = previous["counter_schema"] == current["counter_schema"]
        # The producer publishes every 5s. Short final-summary intervals are
        # retained as partial evidence; missing scheduled snapshots break runs.
        continuous = same_epoch and same_schema and 0 < duration <= 1.5 * MAPPING_REPORT_PERIOD_S
        full_interval = continuous and duration >= 0.8 * MAPPING_REPORT_PERIOD_S
        delta = {k: b[k] - a[k] for k in required}
        new_schema = same_schema and current["counter_schema"] == "successful_integrations_v2"
        if new_schema:
            delta["invalid_consumed"] = b["invalid_consumed"] - a["invalid_consumed"]
        integrated_delta = current["integrated_scans"] - previous["integrated_scans"]
        invalid_scans_delta = current["invalid_scans"] - previous["invalid_scans"]
        valid = (continuous and all(v >= 0 for v in delta.values())
                 and integrated_delta >= 0 and invalid_scans_delta >= 0)
        queue_delta = current["queue_size"] - previous["queue_size"]
        balanced = (valid and delta["accepted"] - delta["processed"] - delta["overflow"]
                    - delta["expired"] - (delta["invalid_consumed"] if new_schema else 0) == queue_delta)
        if not continuous:
            gaps += 1
        if not valid or not balanced:
            invalid += 1
        if full_interval and valid and delta["overflow"] > 0:
            overflow_run += 1
        else:
            overflow_run = 0
        sustained_overflow |= overflow_run >= 2
        intervals.append({"start": previous["wall_monotonic_s"], "end": current["wall_monotonic_s"],
                          "duration_s": duration, "full_metrics_interval": full_interval,
                          "same_epoch": same_epoch, "same_counter_schema": same_schema,
                          "counter_schema": current["counter_schema"] if same_schema else "mixed",
                          "valid": valid, "accounting_balanced": balanced,
                          "counter_deltas": delta, "integrated_delta": integrated_delta,
                          "invalid_scans_delta": invalid_scans_delta,
                          "queue_delta": queue_delta})
    full = [v for v in intervals if v["full_metrics_interval"] and v["valid"] and v["accounting_balanced"]]
    observed_start = samples[0]["wall_monotonic_s"] if samples else None
    observed_end = samples[-1]["wall_monotonic_s"] if samples else None
    boundary_covered = (window["complete"] and samples
                        and observed_start - window["start"] <= 1.5 * MAPPING_REPORT_PERIOD_S
                        and window["end"] - observed_end <= 1.5 * MAPPING_REPORT_PERIOD_S)
    evidence = dict(status_samples=len(samples), missing_samples=missing, full_intervals=len(full),
                    counter_schemas=sorted(schemas), sample_counter_issues=sample_issues,
                    observation_start=observed_start, observation_end=observed_end,
                    observation_span_s=observed_end-observed_start if samples else None,
                    expected_metrics_period_s=MAPPING_REPORT_PERIOD_S,
                    boundary_covered=bool(boundary_covered), intervals=intervals,
                    expired_delta_observed=sum(i["counter_deltas"]["expired"] for i in intervals if i["valid"]))
    net_growth = samples[-1]["queue_size"] - samples[0]["queue_size"] if samples else None
    recovery = any(i["queue_delta"] < 0 for i in intervals if i["valid"] and i["accounting_balanced"])
    evidence.update(net_queue_growth=net_growth, observed_queue_recovery=recovery,
                    transient_net_growth_limit_scans=2,
                    conclusion_scope="observed acquisition window only; not an indefinite stability guarantee")
    post_drain = any("queue_observation_window" in record["payload"] for record in records)
    evidence["observation_policy"] = "post_drain_latency_v1" if post_drain else "legacy_sparse_gauges"
    if sustained_overflow:
        return verdict("fail", "overflow_increased_in_at_least_two_consecutive_metrics_intervals", **evidence)
    if (not boundary_covered or missing or invalid or gaps or len(full) < 2 or len(schemas) != 1):
        return verdict("unknown", "insufficient_contiguous_queue_counter_evidence", **evidence)
    # Positive processing, balanced counters and no overflow support stability
    # over this window. A witnessed decline distinguishes a 1-2 scan endpoint
    # transient from a queue that only grows throughout all observed samples.
    # Occasional TF expiry is recorded independently and is not a failure.
    if (any(i["counter_deltas"]["overflow"] for i in intervals)
            or any(i["integrated_delta"] <= 0 for i in full)):
        return verdict("unknown", "observed_window_does_not_establish_stable_throughput", **evidence)
    if post_drain:
        observations = post_drain_queue_evidence(records)
        evidence["queue_observations"] = observations
        # A single TF miss is not a performance failure. Repeated expiry across
        # consecutive full windows cannot establish normal-TF throughput from
        # survivor-only post-drain ages, even if those ages look small.
        repeated_expiry = any(a["counter_deltas"]["expired"] > 0 and b["counter_deltas"]["expired"] > 0
                              for a, b in zip(intervals[:-1], intervals[1:])
                              if a["full_metrics_interval"] and b["full_metrics_interval"])
        evidence["repeated_tf_expiry"] = repeated_expiry
        if repeated_expiry or observations["status"] != "pass":
            return verdict("unknown", "post_drain_observations_do_not_establish_normal_tf_throughput", **evidence)
        return verdict("pass", "balanced_progress_no_overflow_and_bounded_observed_queue_latency", **evidence)
    if net_growth > 0 and (not recovery or net_growth > 2):
        return verdict("unknown", "observed_window_does_not_establish_stable_throughput", **evidence)
    return verdict("pass", "observed_progress_balanced_counters_no_overflow_and_recovery_or_no_net_growth", **evidence)


def performance_verdict(observed):
    """Separate real computation percentiles from throughput evidence."""
    evidence = observed["performance_observations"]
    start = evidence["goal_wall"]
    has_acquisition_end = evidence.get("acquisition_seen", False)
    end = evidence.get("acquisition_wall") if has_acquisition_end else evidence["finished_wall"]
    window = {"start": start, "end": end,
              "complete": number(start) and number(end) and end >= start,
              "end_source": "acquisition_finished" if has_acquisition_end else "trial_finished",
              "legacy_fallback": not has_acquisition_end,
              "fallback_reason": None if has_acquisition_end else "no acquisition_finished event; trial_finished can include postprocessing time",
              "definition": "first goal_published through acquisition_finished; legacy records fall back explicitly to trial_finished; reported rolling timing windows can include earlier samples"}

    def active(name):
        rows = evidence[name]
        if not window["complete"]:
            return rows  # Known violations remain visible; passing needs a window.
        return [r for r in rows if not number(r["wall_monotonic_s"])
                or start <= r["wall_monotonic_s"] <= end]

    def timing(name):
        rows = active("navigation_status" if name == "control" else "mapping_status")
        measurements, malformed = [], 0
        for row in rows:
            payload = row["payload"]
            metric = nested(payload, "metrics", "control_ms") if name == "control" else payload.get("processing_ms")
            count = nested(metric, "count")
            if count == 0:
                continue  # Startup snapshots before any computation are not samples.
            value = nested(metric, "p99" if name == "control" else "p95")
            period = 100.0 if name == "control" else nested(payload, "observed_scan_period_ms", "p50")
            period_count = None if name == "control" else nested(payload, "observed_scan_period_ms", "count")
            integrated = None if name == "control" else payload.get("integrated_scans")
            valid = type(count) is int and count > 0 and number(value) and value >= 0
            valid = valid and (not window["complete"] or number(row["wall_monotonic_s"]))
            if name != "control":
                valid = (valid and type(integrated) is int and integrated > 0
                         and type(period_count) is int and period_count > 0 and number(period) and period > 0)
            if not valid:
                malformed += 1
                continue
            measurements.append({"source": row["source"], "wall_monotonic_s": row["wall_monotonic_s"],
                                 "reported_sample_count": count, "value_ms": value,
                                 "limit_ms": period, "scan_period_samples": period_count,
                                 "integrated_scans": integrated,
                                 "passed": value < period if name == "control" else value <= period})
        last_observed = max((m["wall_monotonic_s"] for m in measurements if number(m["wall_monotonic_s"])), default=None)
        maximum_tail_gap = 1.5 if name == "control" else 1.5 * MAPPING_REPORT_PERIOD_S
        tail_covered = (window["complete"] and last_observed is not None
                        and end - last_observed <= maximum_tail_gap)
        details = dict(reported_windows=len(measurements), malformed_windows=malformed, measurements=measurements,
                       last_observed=last_observed, tail_covered=tail_covered,
                       maximum_tail_gap_s=maximum_tail_gap,
                       sample_count_scope="per reported rolling window; overlapping counts are never summed",
                       comparison="P99 < 100ms" if name == "control" else "processing P95 <= same snapshot observed scan-period P50; only integrated scans")
        if any(not m["passed"] for m in measurements):
            return verdict("fail", "reported_computation_percentile_exceeds_limit", **details)
        if not window["complete"] or not measurements or malformed or not tail_covered:
            return verdict("unknown", "missing_active_window_or_computation_percentile_evidence", **details)
        return verdict("pass", "all_observed_computation_windows_meet_limit", **details)

    control, mapping = timing("control"), timing("mapping")
    backlog = backlog_performance(active("mapping_status"), window)
    status = combined_status([control, mapping, backlog])
    return verdict(status, "Python normal navigation performance; C++ telemetry is not required",
                   window=window, control_timing=control, mapping_timing=mapping, backlog=backlog)


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
    performance = {"navigation_status": [], "mapping_status": [], "goal_wall": None, "finished_wall": None,
                   "acquisition_wall": None, "acquisition_seen": False}
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
                if kind == "goal_published" and performance["goal_wall"] is None:
                    performance["goal_wall"] = event.get("wall_monotonic_s")
                elif kind == "trial_finished":
                    performance["finished_wall"] = event.get("wall_monotonic_s")
                elif kind == "acquisition_finished":
                    performance["acquisition_wall"] = event.get("wall_monotonic_s")
                    performance["acquisition_seen"] = True
                if kind == "thruster_command" and event.get("axis") in AXES:
                    axis, value = event["axis"], event.get("normalized_command")
                    if number(value):
                        commands[axis].append(float(value))
                    else:
                        invalid_commands[axis] += 1
                elif kind in statuses and isinstance(event.get("payload"), dict):
                    payload = event["payload"]
                    statuses[kind].append(payload)
                    performance[kind].append({"payload": payload, "wall_monotonic_s": event.get("wall_monotonic_s"),
                                              "source": f"events.jsonl:{line_number}"})
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
            if not any(row["payload"] == payload for row in performance[event_name]):
                performance[event_name].append({"payload": payload,
                    "wall_monotonic_s": nested(payload, "process", "monotonic"),
                    "source": "summary." + summary_name})
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
            "status_sources": final_sources, "metrics": flat,
            "performance_observations": performance}


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
    performance = []
    for record in navigation:
        if record["result"]["mode"] != "python":
            continue
        measured = performance_verdict(record["observations"])
        if record["result"].get("result") != "completed" or record["source"].get("manifest_mode") == "dry_run":
            measured = verdict("unknown", "navigation_trial_not_actually_completed", observed_performance=measured)
        performance.append({"trial": record["result"]["trial"], "source": record["source"], **measured})
    sources_finished = all(m["run_finished"] is True and m["manifest_mode"] != "dry_run" for m in metadata)
    performance_passed = bool(performance) and all(p["status"] == "pass" for p in performance)
    all_passed = bool(comparisons) and all(p["passed"] is True for p in comparisons) and all(completed_pass(f) for f in faults) and sources_finished and performance_passed
    return {"schema_version": 2, "generated_utc": datetime.now(timezone.utc).isoformat(),
            "selection": {"duplicate_policy": duplicate_policy, "source_order": "Python directories, baseline directories, extra directories; later source wins provided-list/timestamp ties", "decisions": decisions},
            "method": {"rmse_field": "cross_track_initial_m.rms", "duration_field": "duration_sim_s",
                       "ratio_limit": limit, "zero_baseline_atol": atol,
                       "zero_baseline_rule": "exactly zero cpp: ratio=null; pass iff abs(python)<=atol",
                       "p99_aggregation": "mean/max of reported per-trial P99, not a pooled sample percentile",
                       "performance_scope": "Python non-fault navigation trials only; control P99<100ms, mapping P95<=observed scan-period P50, independently evidenced stable backlog; unknown is not passed",
                       "backlog_rule": "at least 3 same-epoch snapshots and 2 consecutive approximately-5s intervals, boundary coverage, balanced counters and integration progress; sustained overflow fails; legacy sparse gauges retain recovery rule; new post_drain_latency_v1 additionally requires dense 80%-covered 5s windows, bounded wall-time waiting and identity turnover, without requiring an empty queue; recurrent TF expiry is unknown and isolated expiry only recorded",
                       "cpu": "100*sum(nonnegative same-PID cpu_seconds deltas)/sum(positive same-PID process.monotonic deltas); not machine CPU and excludes planner child/Gazebo",
                       "thruster_variation": "sum(abs(command[i]-command[i-1])) per axis in full recorded JSONL order; no initial/final zero appended; total requires all axes and >=2 valid samples each",
                       "counter_scope": "final reported cumulative counters per completed trial, not sums of status event counters"},
            "sources": metadata, "trials": records, "groups": group_results(records, planned),
            "comparisons": comparisons, "fault_trials": [f["result"]["trial"] for f in faults],
            "performance": performance,
            "summary": {"pairs": len(comparisons), "comparable_pairs": sum(p["comparable"] for p in comparisons),
                        "passed_pairs": sum(p["passed"] is True for p in comparisons),
                        "failed_pairs": sum(p["passed"] is False for p in comparisons),
                        "not_comparable_pairs": sum(not p["comparable"] for p in comparisons),
                        "fault_trials": len(faults), "fault_passed": sum(completed_pass(f) for f in faults),
                        "performance_trials": len(performance),
                        "performance_passed": sum(p["status"] == "pass" for p in performance),
                        "performance_failed": sum(p["status"] == "fail" for p in performance),
                        "performance_unknown": sum(p["status"] == "unknown" for p in performance),
                        "all_performance_passed": performance_passed,
                        "all_sources_finished": sources_finished, "all_passed": all_passed}}


def markdown(report):
    def fmt(value):
        return f"{value:.6g}" if number(value) else "缺失/不可比"
    def safe(value):
        return str(value).replace("|", "\\|").replace("\n", " ")
    summary = report["summary"]
    lines = ["# Python/C++ 导航结果对照", "",
             f"总体判定：{'通过' if summary['all_passed'] else '未通过或证据不完整'}。可比 {summary['comparable_pairs']}/{summary['pairs']} 对，通过 {summary['passed_pairs']} 对，失败 {summary['failed_pairs']} 对，不可比 {summary['not_comparable_pairs']} 对；故障用例通过 {summary['fault_passed']}/{summary['fault_trials']}。",
             "", f"Python 正常导航性能：通过 {summary['performance_passed']}、失败 {summary['performance_failed']}、证据不足 {summary['performance_unknown']}；性能证据不足也不能总体通过。",
             "", f"仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ {report['method']['ratio_limit']:.2f}；零基线比例不定义，按绝对容差 {report['method']['zero_baseline_atol']:g} 判断。",
             "", "## 每次配对", "", "| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |", "| --- | --- | --- | --- | --- | --- |"]
    for pair in report["comparisons"]:
        values = []
        for name in ("cross_track_rmse_m", "duration_sim_s"):
            metric = pair["metrics"].get(name, {})
            values.append("零基线绝对容差：" + ("通过" if metric.get('passed') else "失败") if metric.get("method") == "zero_baseline_absolute_tolerance" else fmt(metric.get("ratio")))
        verdict = "通过" if pair["passed"] is True else "失败" if pair["passed"] is False else "不可比：" + "; ".join(pair["reasons"])
        lines.append(f"| {safe(pair['controller'])} | {safe(pair['scenario'])} | {pair['repeat']} | {values[0]} | {values[1]} | {safe(verdict)} |")
    labels = {"pass": "通过", "fail": "失败", "unknown": "待审/证据不足"}
    lines += ["", "## Python 正常导航性能验收", "",
              "窗口为第一次 goal_published 至 acquisition_finished；旧记录明确回退到 trial_finished，该末端可能包含统计落盘时间。状态中的耗时分位数来自节点计算历史，可能含启动样本，各窗口样本数不相加。控制 P99 必须小于100ms；有实际积分扫描的建图处理 P95 不得超过同一状态中扫描时间戳间隔的 P50。扫描周期仅作为预算，不充当算法耗时。C++ 无对应遥测不要求性能对等。", "",
              "积压至少需要3次同一进程/时钟周期的有效快照、2个连续约5s间隔，并覆盖试验窗口首尾。连续两个间隔 overflow 增长判失败；单次溢出、采样缺口、停滞或计数不自洽标待审。处理持续进展、队列计数平衡、无溢出增长时，有明确队列回落的末端1～2扫描瞬态净增不单独否决；持续非减且净增仍待审。两次 queue>0 不能证明持续积压，偶发 TF 过期单列记录。结论仅覆盖观测窗口，不保证无限时长稳定。", "",
              "新增 queue_observation_window 使用独立 post_drain_latency_v1 规则：每个5s后drain观测窗至少覆盖80%，最大观测间隔和末端新鲜度不超过配置的TF墙钟截止；已有积分进展和计数守恒之外，还要求完成扫描等待时间及有效pending年龄分布有界、持续非空时有最老身份轮换。实测空队列以empty_observations表示，不伪造年龄零样本。满足时允许固定单帧流水等待，无需一定看到队列清空；连续区间TF过期增长仅能待审，不能由剩余扫描年龄推断没有过期。旧记录仍沿用上述稀疏观测规则，缺失/混合新字段不退回宽松判定。后drain最大值不代表剔除过期项前或连续时域的峰值。", "",
              "计数按字段版本校验：新字段queue_counters.invalid_consumed存在时，processed必须等于integrated_scans，invalid_consumed必须等于invalid_scans，accepted减去processed、invalid_consumed、expired、overflow等于pending；旧字段缺失时，processed必须等于integrated_scans加invalid_scans，守恒式不重复扣除无效扫描。入队前拒绝的invalid/duplicate不计入accepted。无reset历史时另查累计守恒，reset曾清空的队列缺少丢弃计数时仅查同epoch区间守恒。缺失、负数、版本混合或计数不一致均不当作零或通过，具体原因保存在sample_counter_issues及intervals。", "",
              "| Python试验 | 总状态 | 控制耗时 | 建图耗时 | 积压 | 状态样本/完整间隔/跨度s |", "| --- | --- | --- | --- | --- | --- |"]
    for trial in report["performance"]:
        observed = trial.get("observed_performance", trial)
        control, mapping, backlog = (observed[key] for key in ("control_timing", "mapping_timing", "backlog"))
        label = trial['trial'] + ("（旧末端回退）" if observed['window']['legacy_fallback'] else "")
        lines.append(f"| {safe(label)} | {labels[trial['status']]} | {labels[control['status']]} | {labels[mapping['status']]} | {labels[backlog['status']]} | {backlog['status_samples']}/{backlog['full_intervals']}/{fmt(backlog['observation_span_s'])} |")
    lines += ["", "各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。"]
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
