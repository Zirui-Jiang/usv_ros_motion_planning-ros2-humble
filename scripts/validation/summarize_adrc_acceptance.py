#!/usr/bin/env python3
"""Summarize finished PID/LQR/ADRC ROS acceptance artifacts without rerunning ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


CSV_FIELDS = (
    "trial", "algorithm", "scenario", "fault", "fault_flag", "result", "passed", "arrived",
    "duration_sim_s", "initial_path_rmse_m", "initial_path_max_m",
    "minimum_footprint_clearance_m", "output_thruster_peak_abs", "output_thruster_mean_abs",
    "thruster_sample_count", "controller_requested", "controller_actual",
    "controller_verification_matches_claim", "config_sha256", "control_p99_ms", "control_max_ms",
    "control_overruns", "source_summary", "source_events",
)


def _number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return value
    return None


def _nested(value, *keys):
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _read_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def _manifest_for(run_root):
    for directory in (run_root, *run_root.parents):
        path = directory / "manifest.json"
        if path.is_file():
            return _read_json(path)
    return {}


def _discover(inputs):
    """Yield unique (summary row, trial directory, summary path, manifest) tuples."""
    found = {}
    for raw_root in inputs:
        root = Path(raw_root).resolve()
        summary_path = root / "summary.json"
        if not summary_path.is_file():
            raise ValueError(f"missing summary.json in input root: {root}")
        summary = _read_json(summary_path)
        results = summary.get("results")
        if isinstance(results, list):
            manifest = _manifest_for(root)
            for row in results:
                if not isinstance(row, dict):
                    continue
                trial_name = row.get("trial")
                trial_dir = root / str(trial_name) if trial_name else root
                trial_summary = trial_dir / "summary.json"
                identity = (trial_summary if trial_summary.is_file() else summary_path).resolve()
                found.setdefault(identity, (row, trial_dir, identity, manifest))
        else:
            found.setdefault(summary_path.resolve(), (summary, root, summary_path.resolve(), _manifest_for(root)))
    return list(found.values())


def _events(path):
    if not path.is_file():
        return []
    values = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid JSONL {path}:{line_number}: {error}") from error
        if isinstance(event, dict):
            values.append(event)
    return values


def _event_observations(events, claimed_controller):
    verification = next((event for event in events if event.get("event") == "controller_verified"), None)
    requested = verification.get("requested") if verification else None
    actual = verification.get("actual") if verification else None
    verified = bool(verification) and requested == actual == claimed_controller

    start = next((_number(event.get("wall_monotonic_s")) for event in events
                  if event.get("event") == "goal_published"), None)
    end = next((_number(event.get("wall_monotonic_s")) for event in events
                if event.get("event") == "acquisition_finished"), None)
    samples = []
    if start is not None and end is not None and end >= start:
        for event in events:
            stamp = _number(event.get("wall_monotonic_s"))
            command = _number(event.get("normalized_command"))
            if event.get("event") == "thruster_command" and stamp is not None and start <= stamp <= end and command is not None:
                samples.append(abs(command))
    return {
        "controller_requested": requested,
        "controller_actual": actual,
        "controller_verification_matches_claim": verified,
        "output_thruster_peak_abs": max(samples) if samples else None,
        "output_thruster_mean_abs": sum(samples) / len(samples) if samples else None,
        "thruster_sample_count": len(samples) if samples else None,
    }


def _config_sha(manifest):
    return manifest.get("controller_config_sha256") or manifest.get("config_sha256")


def _trial_row(summary, trial_dir, summary_path, manifest):
    events_path = trial_dir / "events.jsonl"
    controller = summary.get("controller")
    status = summary.get("status")
    fault = summary.get("fault")
    row = {
        "trial": summary.get("trial"),
        "algorithm": controller,
        "scenario": summary.get("scenario"),
        "fault": fault,
        "fault_flag": fault is not None,
        "result": summary.get("result"),
        "passed": summary.get("passed") if isinstance(summary.get("passed"), bool) else None,
        "arrived": summary.get("arrived") if isinstance(summary.get("arrived"), bool) else None,
        "duration_sim_s": _number(summary.get("duration_sim_s")),
        "initial_path_rmse_m": _number(_nested(summary, "cross_track_initial_m", "rms")),
        "initial_path_max_m": _number(_nested(summary, "cross_track_initial_m", "max")),
        "minimum_footprint_clearance_m": _number(summary.get("minimum_footprint_clearance_m")),
        "config_sha256": _config_sha(manifest),
        "control_p99_ms": _number(_nested(status, "metrics", "control_ms", "p99")),
        "control_max_ms": _number(_nested(status, "metrics", "control_ms", "max")),
        "control_overruns": _number(_nested(status, "counters", "control_overruns")),
        "source_summary": str(summary_path),
        "source_events": str(events_path) if events_path.is_file() else None,
    }
    row.update(_event_observations(_events(events_path), controller))
    return row


def _values(rows, field):
    return [value for row in rows if (value := _number(row.get(field))) is not None]


def _mean(values):
    return sum(values) / len(values) if values else None


def _groups(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row.get("algorithm"), row.get("scenario"))].append(row)
    output = []
    for (algorithm, scenario), group_rows in sorted(grouped.items(), key=lambda item: tuple(str(v) for v in item[0])):
        normal_rows = [row for row in group_rows if not row["fault_flag"]]
        p99 = _values(normal_rows, "control_p99_ms")
        control_max = _values(normal_rows, "control_max_ms")
        overruns = _values(normal_rows, "control_overruns")
        output.append({
            "algorithm": algorithm,
            "scenario": scenario,
            "total_trials": len(group_rows),
            "arrived_trials": sum(row.get("arrived") is True for row in group_rows),
            "passed_trials": sum(row.get("passed") is True for row in group_rows),
            "fault_trials": sum(row["fault_flag"] for row in group_rows),
            "normal_metric_trials": len(normal_rows),
            "mean_duration_sim_s": _mean(_values(normal_rows, "duration_sim_s")),
            "mean_initial_path_rmse_m": _mean(_values(normal_rows, "initial_path_rmse_m")),
            "max_initial_path_error_m": max(_values(normal_rows, "initial_path_max_m"), default=None),
            "minimum_clearance_m": min(_values(normal_rows, "minimum_footprint_clearance_m"), default=None),
            "mean_output_thruster_mean_abs": _mean(_values(normal_rows, "output_thruster_mean_abs")),
            "max_output_thruster_peak_abs": max(_values(normal_rows, "output_thruster_peak_abs"), default=None),
            "max_trial_reported_p99_ms": max(p99, default=None),
            "max_trial_reported_control_max_ms": max(control_max, default=None),
            "control_overruns_sum_for_provided_trials": sum(overruns) if overruns else None,
            "control_overruns_provided_trials": len(overruns),
        })
    return output


def build_comparison(inputs):
    trials = [_trial_row(*item) for item in _discover(inputs)]
    trials.sort(key=lambda row: (str(row.get("algorithm")), str(row.get("scenario")), str(row.get("trial"))))
    return {
        "schema_version": 1,
        "method": {
            "grouping": "algorithm/scenario",
            "fault_metric_policy": "fault rows are retained and counted but excluded from cross-trial metric aggregates",
            "missing_data_policy": "missing metrics remain null and are never replaced with zero",
            "controller_claim_policy": "a controller claim is verified only by a matching controller_verified requested/actual event",
            "thruster_window": "first goal_published through acquisition_finished, inclusive",
            "timing_semantics": {
                "aggregate": "max_trial_reported_p99_ms",
                "raw_per_cycle_timing_available": False,
                "note": "Each value is the final status.metrics.control_ms p99 for one reported window; raw per-cycle timing is unavailable, so trial p99 values are not averaged or pooled.",
            },
            "claim_limit": "descriptive evidence only; no performance-advantage claim is made",
        },
        "trials": trials,
        "groups": _groups(trials),
        "summary": {
            "total_trials": len(trials),
            "arrived_trials": sum(row.get("arrived") is True for row in trials),
            "passed_trials": sum(row.get("passed") is True for row in trials),
            "controller_verified_trials": sum(row["controller_verification_matches_claim"] for row in trials),
            "fault_trials": sum(row["fault_flag"] for row in trials),
        },
    }


def _format(value):
    if value is None:
        return "缺失"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _markdown(report):
    summary = report["summary"]
    lines = [
        "# PID/LQR/ADRC Gazebo 验收证据汇总", "",
        f"共保留 {summary['total_trials']} 次试验，到达 {summary['arrived_trials']} 次，通过 {summary['passed_trials']} 次，控制器身份核验一致 {summary['controller_verified_trials']} 次。",
        "本报告仅整理验收证据，不据此声明性能优势。故障试验保留为独立行，但不进入跨试验性能指标聚合。", "",
        "控制耗时只提供各 trial 最终上报窗口的 P99/最大值；没有逐周期原始时序。组级字段 `max_trial_reported_p99_ms` 取 trial P99 最大值，不计算 P99 的平均值，也不解释为合并样本 P99。", "",
        "## 分组", "",
        "| 算法/场景 | 到达/总数/通过 | 正常指标试验 | 初始路径 RMSE 均值/最大误差 m | 最小间隙 m | 推进器 mean-abs 均值/peak-abs 最大 | max trial-reported P99/控制最大 ms | overruns 合计（有值试验数） |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for group in report["groups"]:
        lines.append(
            f"| {group['algorithm']}/{group['scenario']} | {group['arrived_trials']}/{group['total_trials']}/{group['passed_trials']} | "
            f"{group['normal_metric_trials']} | {_format(group['mean_initial_path_rmse_m'])}/{_format(group['max_initial_path_error_m'])} | "
            f"{_format(group['minimum_clearance_m'])} | {_format(group['mean_output_thruster_mean_abs'])}/{_format(group['max_output_thruster_peak_abs'])} | "
            f"{_format(group['max_trial_reported_p99_ms'])}/{_format(group['max_trial_reported_control_max_ms'])} | "
            f"{_format(group['control_overruns_sum_for_provided_trials'])} ({group['control_overruns_provided_trials']}) |"
        )
    lines += ["", "## 每次试验", "",
              "| trial | 算法/场景 | 故障 | 结果/到达/通过 | 仿真时长 s | 初始 RMSE/最大 m | 间隙 m | 推进器 peak/mean abs | 控制器 requested/actual/一致 | 控制 P99/最大 ms | overruns | config SHA |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for row in report["trials"]:
        lines.append(
            f"| {row['trial']} | {row['algorithm']}/{row['scenario']} | {_format(row['fault'])} | {row['result']}/{_format(row['arrived'])}/{_format(row['passed'])} | "
            f"{_format(row['duration_sim_s'])} | {_format(row['initial_path_rmse_m'])}/{_format(row['initial_path_max_m'])} | {_format(row['minimum_footprint_clearance_m'])} | "
            f"{_format(row['output_thruster_peak_abs'])}/{_format(row['output_thruster_mean_abs'])} | {_format(row['controller_requested'])}/{_format(row['controller_actual'])}/{row['controller_verification_matches_claim']} | "
            f"{_format(row['control_p99_ms'])}/{_format(row['control_max_ms'])} | {_format(row['control_overruns'])} | {_format(row['config_sha256'])} |"
        )
    return "\n".join(lines) + "\n"


def write_outputs(report, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "comparison.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    with (output / "trials.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["trials"])
    (output / "comparison.md").write_text(_markdown(report), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, nargs="+", required=True, help="one or more finished acceptance run roots")
    parser.add_argument("--output", type=Path, required=True, help="fresh output directory")
    args = parser.parse_args(argv)
    try:
        write_outputs(build_comparison(args.input), args.output)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
