#!/usr/bin/env python3
# 从已完成试验的比较报告导出图表和 CSV，保留所选指标定义、原始来源与失败记录。
"""Export observed acceptance results after the live trials have stopped.

Input is compare_results.py's report.json, with its selected raw summaries and
events still available. No ROS imports, rerunning trials, smoothing, pooled
percentiles, inferred baselines, or changes to acceptance verdicts are involved.
Matplotlib is imported only when rendering, so --help needs the standard library.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys


SCENARIOS = ("straight", "turn", "single_obstacle")
CONTROLLERS = ("pid", "lqr")
TITLES = {"straight": "Straight", "turn": "90-degree initial heading change",
          "single_obstacle": "Single cylinder"}
COLORS = ("#0072B2", "#D55E00", "#009E73")
MARKERS = ("o", "s", "^")
STYLES = ("-", "--", "-.")


# 区分有效有限数值与缺失、NaN、无穷和布尔值，避免污染统计结果。
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 安全读取嵌套结果字段；缺少字段时保留 None，而不补造零值。
def nested(value, *keys):
    for key in keys:
        value = value.get(key) if isinstance(value, dict) else None
    return value


# 流式计算输入文件 SHA-256，核对绘图数据仍是报告选中的同一份来源。
def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


# 读取 JSON 并拒绝非有限数字标记，保持后续统计与序列化的输入契约。
def read_json(path):
    def reject(value):
        raise ValueError(f"nonfinite JSON token {value}: {path}")
    with path.open(encoding="utf-8-sig") as stream:
        return json.load(stream, parse_constant=reject)


# 按明确的路径前缀映射定位归档数据，不修改原始记录中的路径。
def map_path(value, mappings):
    value = str(value)
    for old, new in mappings:
        if value == old or value.startswith(old.rstrip("/") + "/"):
            value = new.rstrip("/\\") + value[len(old.rstrip("/")):]
            break
    return Path(value).expanduser().resolve()


# 仅把已完成且通过的记录识别为成功，保留其余记录用于解释失败。
def successful(row):
    return row.get("result") == "completed" and row.get("passed") is True


# 提取试验身份字段，让图表和导出数据可以追溯到具体重复试验。
def identity(row):
    return {key: row.get(key) for key in ("trial", "mode", "controller", "scenario", "repeat")}


# 按重复序号稳定分配颜色、标记与线型，便于跨图比较同一次试验。
def style(row):
    repeat = row.get("repeat", 1)
    index = (repeat - 1) % len(COLORS)
    return COLORS[index], MARKERS[index], STYLES[index]


# 核对来源哈希后载入轨迹与事件；观测间隔过大时分段显示，避免补画未观测运动。
def load_trial(record, mappings, gap, issues):
    row, source = record["result"], record["source"]
    summary_path = map_path(source["summary_path"], mappings)
    events_path = map_path(record["observations"]["events_path"], mappings)
    if not summary_path.is_file():
        raise ValueError(f"selected source summary is missing: {summary_path}")
    actual_hash = sha256(summary_path)
    if source.get("summary_sha256") and source["summary_sha256"] != actual_hash:
        raise ValueError(f"summary changed since comparison: {summary_path}")
    summary = read_json(summary_path)
    if "results" in summary:
        summary = next((item for item in summary["results"] if item.get("trial") == row["trial"]), None)
    if summary != row:
        raise ValueError(f"selected summary no longer matches comparison: {row['trial']}")
    trial = {"row": row, "record": record, "samples": [], "goals": [], "obstacles": [],
             "initial_path": [], "start_wall": None, "end_wall": None,
             "source": {**identity(row), "summary_path": str(summary_path),
                        "summary_sha256": actual_hash, "events_path": str(events_path),
                        "events_sha256": None, "missing_events": not events_path.is_file()}}
    if not events_path.is_file():
        issues.append({**identity(row), "issue": "missing_events; trajectory omitted"})
        return trial
    trial["source"]["events_sha256"] = sha256(events_path)
    events = []
    with events_path.open(encoding="utf-8-sig") as stream:
        for line, content in enumerate(stream, 1):
            try:
                event = json.loads(content)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid event at {events_path}:{line}: {error}") from error
            event["_line"] = line
            events.append(event)
    goal_events = [e for e in events if e.get("event") == "goal_published"]
    if goal_events:
        trial["start_wall"] = goal_events[0].get("wall_monotonic_s")
    ends = [e for e in events if e.get("event") == "acquisition_finished"]
    fallback = not ends
    if not ends:
        ends = [e for e in events if e.get("event") == "trial_finished"]
    if ends:
        trial["end_wall"] = ends[-1].get("wall_monotonic_s")
    trial["source"]["trajectory_end_fallback_to_trial_finished"] = fallback
    trial["source"]["trajectory_window_complete"] = all(finite(trial[k]) for k in ("start_wall", "end_wall"))
    if not trial["source"]["trajectory_window_complete"]:
        issues.append({**identity(row), "issue": "incomplete acquisition window; all available samples shown"})
    previous_time, segment = None, 0
    for event in events:
        kind, wall = event.get("event"), event.get("wall_monotonic_s")
        if kind == "obstacle_spawned" and isinstance(event.get("geometry"), dict):
            geometry = event["geometry"]
            if all(finite(geometry.get(k)) for k in ("x", "y", "radius")) and geometry["radius"] > 0:
                trial["obstacles"].append(geometry)
        if kind == "goal_published":
            goal = event.get("goal")
            if isinstance(goal, list) and len(goal) >= 2 and all(finite(v) for v in goal[:2]):
                trial["goals"].append(goal)
        if trial["source"]["trajectory_window_complete"] and finite(wall):
            if wall < trial["start_wall"] or wall > trial["end_wall"]:
                continue
        if kind == "reference_path" and not trial["initial_path"] and event.get("points"):
            trial["initial_path"] = event["points"]
        if kind != "sample":
            continue
        pose, stamp, velocity = event.get("pose"), event.get("sim_time_s"), event.get("body_velocity")
        if not (isinstance(pose, list) and len(pose) >= 3 and all(finite(v) for v in pose[:3]) and finite(stamp)):
            issues.append({**identity(row), "line": event["_line"], "issue": "invalid pose/time; trajectory segment broken"})
            previous_time, segment = None, segment + 1
            continue
        if previous_time is not None and (stamp < previous_time or stamp - previous_time > gap):
            segment += 1
        previous_time = stamp
        v = velocity if isinstance(velocity, list) and len(velocity) >= 3 else [None] * 3
        trial["samples"].append({**identity(row), "event_line": event["_line"], "segment": segment,
                                 "wall_monotonic_s": wall, "sim_time_s": stamp,
                                 "x_m": pose[0], "y_m": pose[1], "heading_rad": pose[2],
                                 "u_m_s": v[0], "v_m_s": v[1], "r_rad_s": v[2]})
    return trial


# 兼容旧报告的指标字段，同时明确返回当前采用的 RMSE 定义。
def rmse_metric_name(report):
    return report.get("method", {}).get("rmse_metric", "legacy_wall")


# 在坐标轴标出误差单位和指标定义，避免混淆两种 RMSE。
def rmse_axis_label(report):
    return "RMSE (m)\n" + rmse_metric_name(report)


# 同时导出所选指标、墙钟指标和源时间指标，并保留源时间有效性。
def rmse_export_values(trial, report):
    metric = rmse_metric_name(report)
    legacy = nested(trial["row"], "cross_track_initial_m", "rms")
    source = nested(trial["record"], "observations", "tracking", "source_time_trapezoid") or {}
    value = source.get("rmse_m") if source.get("status") == "valid" else None
    return {"selected_rmse_metric": metric, "selected_rmse_m": legacy if metric == "legacy_wall" else value,
            "legacy_wall_rmse_m": legacy, "source_time_trapezoid_rmse_m": value,
            "source_time_status": source.get("status", "unknown"),
            "source_time_reasons": "; ".join(source.get("reasons", [])),
            "source_sample_count": source.get("sample_count"), "source_span_s": source.get("source_span_s"),
            "source_max_gap_s": source.get("max_source_gap_s"),
            "source_start_coverage_gap_s": source.get("start_coverage_gap_s"),
            "source_end_coverage_gap_s": source.get("end_coverage_gap_s"),
            "source_error_p95_m": source.get("error_p95_m"), "source_error_max_m": source.get("error_max_m")}


# 按固定字段顺序写出 CSV，便于表格软件读取与后续复核。
def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


# 筛选规定的 PID/LQR 场景与重复试验，显式记录跳过、缺失和来源问题。
def prepare(report, mappings, gap):
    trials, issues, skipped, keys = [], [], [], set()
    for record in report["trials"]:
        row = record["result"]
        if row.get("fault") or row.get("scenario") not in SCENARIOS or row.get("controller") not in CONTROLLERS:
            skipped.append({**identity(row), "reason": "outside normal PID/LQR three-scenario figure scope"})
            continue
        key = tuple(row.get(k) for k in ("mode", "controller", "scenario", "repeat"))
        if key in keys:
            raise ValueError(f"report contains unresolved duplicate selection: {key}")
        keys.add(key)
        trials.append(load_trial(record, mappings, gap, issues))
    # Boundary-sensitive and grid-interior reruns are distinct experiments.
    # Do not silently overlay shifted goals/cylinders in a single comparison.
    for scenario in SCENARIOS:
        group = [trial for trial in trials if trial["row"]["scenario"] == scenario]
        goals = {tuple(goal[:2]) for trial in group for goal in trial["goals"]}
        cylinders = {tuple(obstacle[k] for k in ("x", "y", "radius"))
                     for trial in group for obstacle in trial["obstacles"]}
        if len(goals) > 1 or len(cylinders) > 1:
            raise ValueError(f"mixed scenario geometries in {scenario}; use separate reports/outputs for shifted reruns")
    timing = []
    for performance in report.get("performance", []):
        source = performance.get("source", {})
        matching = [trial for trial in trials if trial["row"]["mode"] == "python"
                    and trial["row"]["trial"] == performance["trial"]
                    and trial["record"]["source"].get("summary_path") == source.get("summary_path")]
        if not matching:
            continue
        trial = matching[0]
        observed = performance.get("observed_performance", performance)
        start = nested(observed, "window", "start")
        for kind, key in (("control", "control_timing"), ("mapping", "mapping_timing")):
            for measurement in observed.get(key, {}).get("measurements", []):
                wall = measurement.get("wall_monotonic_s")
                timing.append({**identity(trial["row"]), "kind": kind, "source": measurement.get("source"),
                               "wall_monotonic_s": wall,
                               "wall_since_goal_s": wall - start if finite(wall) and finite(start) else None,
                               "reported_percentile": "p99" if kind == "control" else "p95",
                               "value_ms": measurement.get("value_ms"), "budget_ms": measurement.get("limit_ms"),
                               "reported_sample_count": measurement.get("reported_sample_count"),
                               "scan_period_samples": measurement.get("scan_period_samples"),
                               "integrated_scans": measurement.get("integrated_scans"),
                               "window_passed": measurement.get("passed"),
                               "timing_status": nested(observed, key, "status"),
                               "overall_performance_status": performance.get("status")})
    return trials, timing, issues, skipped


# 选择同场景同控制器的试验，并按实现和重复序号稳定排序。
def selected(trials, scenario, controller):
    return sorted((t for t in trials if t["row"]["scenario"] == scenario and t["row"]["controller"] == controller),
                  key=lambda t: (t["row"]["mode"] != "cpp", t["row"]["repeat"]))


# 使用无界面绘图后端导出轨迹及指标图；只消费已有观测，不重跑仿真。
def figures(trials, timing, report, output, dpi):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    exported = []

    def save(fig, name, caption):
        fig.suptitle(caption, fontsize=10)
        for extension in ("png", "pdf"):
            path = output / f"{name}.{extension}"
            fig.savefig(path, dpi=dpi, facecolor="white", metadata={"Title": caption})
            exported.append({"path": path.name, "sha256": sha256(path),
                             "width_inches": float(fig.get_figwidth()), "height_inches": float(fig.get_figheight()),
                             "dpi": dpi if extension == "png" else None})
        plt.close(fig)

    with plt.rc_context({"font.family": "DejaVu Sans", "font.size": 8, "axes.titlesize": 9,
                         "axes.labelsize": 8, "legend.fontsize": 7, "pdf.fonttype": 42,
                         "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(3, 2, figsize=(10, 10), constrained_layout=True)
        for i, scenario in enumerate(SCENARIOS):
            for j, controller in enumerate(CONTROLLERS):
                ax, group = axes[i, j], selected(trials, scenario, controller)
                baseline_count, python_count, geometry_seen = 0, 0, set()
                for trial in group:
                    row, points = trial["row"], trial["samples"]
                    if row["mode"] == "cpp" and not successful(row):
                        continue
                    if not points:
                        continue
                    color, marker, linestyle = style(row)
                    label = f"Python #{row['repeat']}" if row["mode"] == "python" else f"C++ #{row['repeat']}"
                    if row["mode"] == "cpp":
                        color, baseline_count = "#777777", baseline_count + 1
                    else:
                        python_count += 1
                        if not successful(row):
                            label += " (not passed)"
                    for n, segment in enumerate(sorted({p["segment"] for p in points})):
                        part = [p for p in points if p["segment"] == segment]
                        ax.plot([p["x_m"] for p in part], [p["y_m"] for p in part], color=color,
                                linestyle=linestyle, linewidth=1.25, marker=marker, markersize=2.3,
                                markevery=max(1, len(part) // 12), label=label if n == 0 else None)
                    ax.scatter(points[0]["x_m"], points[0]["y_m"], marker="o", s=22,
                               facecolors="white", edgecolors=color, zorder=4)
                    for goal in trial["goals"]:
                        ax.scatter(goal[0], goal[1], marker="*", color="black", s=55, zorder=4)
                    for obstacle in trial["obstacles"]:
                        key = tuple(obstacle[k] for k in ("x", "y", "radius"))
                        if key not in geometry_seen:
                            geometry_seen.add(key)
                            ax.add_patch(Circle(key[:2], key[2], facecolor="#DDDDDD", edgecolor="#444444", hatch="///"))
                notes = []
                if baseline_count == 0:
                    notes.append("No valid completed C++ baseline")
                if python_count < 3:
                    notes.append(f"Python trajectories available: {python_count}/3")
                title = f"{TITLES[scenario]} | {controller.upper()}"
                ax.set(title=title + ("\n" + "; ".join(notes) if notes else ""), xlabel="Map x (m)", ylabel="Map y (m)")
                ax.set_aspect("equal", adjustable="box")
                ax.grid(alpha=.2)
                if ax.get_legend_handles_labels()[0]:
                    ax.legend(loc="upper right", ncol=2, fontsize=6, framealpha=.9)
            # Identical map limits for the two controllers in the same scenario.
            limits = [ax.get_xlim() + ax.get_ylim() for ax in axes[i] if ax.has_data()]
            if limits:
                xmin, xmax = min(v[0] for v in limits), max(v[1] for v in limits)
                ymin, ymax = min(v[2] for v in limits), max(v[3] for v in limits)
                # Pad the shorter geographic dimension, never stretch metres.
                # Without padding an almost straight track collapses its axes
                # into a thin strip and causes titles/legends to overlap data.
                width, height = max(xmax - xmin, 1.0), max(ymax - ymin, 1.0)
                panel_ratio = 1.45
                width = max(width, height * panel_ratio)
                height = max(height, width / panel_ratio)
                cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
                for ax in axes[i]:
                    ax.set_xlim(cx - width / 2, cx + width / 2)
                    ax.set_ylim(cy - height / 2, cy + height / 2)
        save(fig, "trajectories", "Observed vessel-centre tracks; open circles: starts; stars: requested goals\n"
             "C++ shown only when completed and passed; raw spawned cylinder geometry; no smoothing")

        fig, axes = plt.subplots(2, 3, figsize=(12, 6.2), constrained_layout=True)
        for i, (field, ylabel) in enumerate((("rmse", rmse_axis_label(report)), ("arrival", "Arrival duration (simulation s)"))):
            for j, scenario in enumerate(SCENARIOS):
                ax = axes[i, j]
                missing = 0
                for trial in trials:
                    row = trial["row"]
                    if row["scenario"] != scenario or row["mode"] == "cpp" and not successful(row):
                        continue
                    value = rmse_export_values(trial, report)["selected_rmse_m"] if field == "rmse" else row.get("duration_sim_s")
                    if field == "arrival" and not (row.get("arrived") is True and row.get("result") == "completed"):
                        value = None
                    if not finite(value):
                        missing += 1
                        continue
                    color, marker, _ = style(row)
                    x = CONTROLLERS.index(row["controller"]) * 3 + row["repeat"] - 1
                    cpp = row["mode"] == "cpp"
                    ax.scatter(x + (-.12 if cpp else .12), value, color="#777777" if cpp else color,
                               marker="x" if cpp or not successful(row) else marker, s=30)
                no_lqr_baseline = not any(t["row"]["controller"] == "lqr" and t["row"]["scenario"] == scenario
                                         and t["row"]["mode"] == "cpp" and successful(t["row"]) for t in trials)
                title = TITLES[scenario] + ("\nLQR: no valid C++ baseline" if no_lqr_baseline else "")
                ax.set(title=title, ylabel=ylabel, xticks=range(6),
                       xticklabels=[f"{c.upper()} {r}" for c in CONTROLLERS for r in (1, 2, 3)])
                ax.tick_params(axis="x", rotation=40)
                ax.set_ylim(bottom=0)
                ax.grid(axis="y", alpha=.2)
                if missing:
                    ax.text(.02, .02, f"Missing/non-arrival values: {missing}", transform=ax.transAxes, fontsize=7)
        save(fig, "navigation_metrics", "Individual trial values: coloured Python, grey crosses valid C++\n"
             "No averaging or confidence intervals; unfinished durations are not arrival times")

        ratio_rows = []
        fig, axes = plt.subplots(2, 3, figsize=(12, 5.8), constrained_layout=True)
        for i, metric in enumerate(("cross_track_rmse_m", "duration_sim_s")):
            for j, scenario in enumerate(SCENARIOS):
                ax, unavailable = axes[i, j], 0
                for pair in report.get("comparisons", []):
                    if pair.get("scenario") != scenario or pair.get("controller") not in CONTROLLERS:
                        continue
                    value = pair.get("metrics", {}).get(metric, {})
                    ratio_rows.append({"controller": pair["controller"], "scenario": scenario, "repeat": pair["repeat"],
                                       "metric": metric, "selected_rmse_metric": rmse_metric_name(report), "pair_comparable": pair.get("comparable"),
                                       "pair_passed": pair.get("passed"), **value,
                                       "reasons": "; ".join(pair.get("reasons", []))})
                    ratio = value.get("ratio")
                    if not pair.get("comparable") or not finite(ratio):
                        unavailable += 1
                        continue
                    index = CONTROLLERS.index(pair["controller"]) * 3 + pair["repeat"] - 1
                    ax.scatter(index, ratio, color=COLORS[(pair["repeat"] - 1) % 3],
                               marker="o" if value.get("passed") is True else "x", s=33)
                limit = report.get("method", {}).get("ratio_limit", 1.10)
                ax.axhline(limit, color="black", linestyle="--", linewidth=1, label=f"Limit {limit:g}")
                ax.axhline(1.0, color="#888888", linestyle=":", linewidth=.8)
                ax.set(title=TITLES[scenario], ylabel=("RMSE " + rmse_metric_name(report) if i == 0 else "Duration") + "\nPython / C++",
                       xticks=range(6), xticklabels=[f"{c.upper()} {r}" for c in CONTROLLERS for r in (1, 2, 3)])
                ax.tick_params(axis="x", rotation=40)
                ax.set_ylim(bottom=0)
                ax.grid(axis="y", alpha=.2)
                ax.legend(loc="upper right")
                ax.text(.02, .02, f"Unavailable/zero-base ratios: {unavailable}", transform=ax.transAxes, fontsize=7)
            upper = max(ax.get_ylim()[1] for ax in axes[i])
            for ax in axes[i]:
                ax.set_ylim(0, upper)
        save(fig, "pairwise_ratios", "Precomputed acceptance ratios: only comparable completed/passed trial pairs\n"
             "Missing baselines and zero-baseline absolute-tolerance cases are not plotted as zero")

        for kind, name, ylabel in (("control", "control_timing", "Reported control P99 (ms)"),
                                   ("mapping", "mapping_timing", "Reported scan processing P95 (ms)")):
            fig, axes = plt.subplots(3, 2, figsize=(11, 9), constrained_layout=True)
            maxima = []
            for i, scenario in enumerate(SCENARIOS):
                for j, controller in enumerate(CONTROLLERS):
                    ax = axes[i, j]
                    group = [r for r in timing if r["kind"] == kind and r["scenario"] == scenario and r["controller"] == controller]
                    for repeat in (1, 2, 3):
                        rows = [r for r in group if r["repeat"] == repeat and finite(r["wall_since_goal_s"]) and finite(r["value_ms"])]
                        if rows:
                            ax.scatter([r["wall_since_goal_s"] for r in rows], [r["value_ms"] for r in rows],
                                       color=COLORS[repeat - 1], marker=MARKERS[repeat - 1], s=13, label=f"Python #{repeat}")
                            maxima.extend(r["value_ms"] for r in rows)
                            budgets = [r for r in rows if finite(r["budget_ms"])]
                            ax.scatter([r["wall_since_goal_s"] for r in budgets], [r["budget_ms"] for r in budgets],
                                       color="#444444", marker="_", s=23, label="Observed scan-period P50 budget" if kind == "mapping" and repeat == 1 else None)
                            maxima.extend(r["budget_ms"] for r in budgets)
                    if kind == "control":
                        ax.axhline(100, color="black", linestyle="--", linewidth=1, label="100 ms budget (strict)")
                        maxima.append(100)
                    if not group:
                        ax.text(.5, .5, "No usable computation timing evidence", transform=ax.transAxes, ha="center")
                    statuses = sorted({r["timing_status"] for r in group if r.get("timing_status")})
                    ax.set(title=f"{TITLES[scenario]} | {controller.upper()} | {','.join(statuses) or 'unknown'}",
                           xlabel="Wall time since goal publication (s)", ylabel=ylabel)
                    ax.grid(alpha=.2)
                    if ax.get_legend_handles_labels()[0]:
                        ax.legend(loc="best")
            if maxima:
                for ax in axes.flat:
                    ax.set_ylim(0, max(maxima) * 1.08)
            save(fig, name, "Actual node-reported computation percentiles versus budgets; one marker per reported window\n"
                 "Overlapping history counts are not pooled; message intervals are not computation durations")
    return exported, ratio_rows, matplotlib.__version__


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path, help="compare_results.py report.json")
    parser.add_argument("--output", required=True, type=Path, help="new directory outside raw results")
    parser.add_argument("--path-map", action="append", default=[], metavar="OLD=NEW", help="relocate archived source paths; first matching prefix wins")
    parser.add_argument("--max-track-gap-s", type=float, default=.35, help="break trajectories across larger simulation sample gaps (default .35s)")
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args(argv)
    try:
        if not finite(args.max_track_gap_s) or args.max_track_gap_s <= 0 or not 72 <= args.dpi <= 1200:
            raise ValueError("positive sample gap and dpi in [72,1200] are required")
        mappings = []
        for mapping in args.path_map:
            old, separator, new = mapping.partition("=")
            if not separator or not old or not new:
                raise ValueError("--path-map must be OLD=NEW with two nonempty paths")
            mappings.append((old.rstrip("/"), new))
        report_path, output = args.report.resolve(), args.output.resolve()
        report = read_json(report_path)
        if not isinstance(report.get("trials"), list):
            raise ValueError("input must contain compare_results.py selected trials")
        if output.exists():
            raise ValueError(f"output must be a new directory: {output}")
        for record in report["trials"]:
            raw = map_path(record["source"]["run_directory"], mappings)
            if output == raw or raw in output.parents:
                raise ValueError(f"output cannot be inside raw results: {raw}")
        trials, timing, issues, skipped = prepare(report, mappings, args.max_track_gap_s)
        output.mkdir(parents=True)
        exported, ratios, version = figures(trials, timing, report, output, args.dpi)
        common = ["trial", "mode", "controller", "scenario", "repeat"]
        write_csv(output / "trajectory_samples.csv", [p for t in trials for p in t["samples"]],
                  common + ["event_line", "segment", "wall_monotonic_s", "sim_time_s", "x_m", "y_m", "heading_rad", "u_m_s", "v_m_s", "r_rad_s"])
        write_csv(output / "timing_windows.csv", timing, common + ["kind", "source", "wall_monotonic_s", "wall_since_goal_s",
                  "reported_percentile", "value_ms", "budget_ms", "reported_sample_count", "scan_period_samples", "integrated_scans",
                  "window_passed", "timing_status", "overall_performance_status"])
        metrics = []
        for trial in trials:
            row = trial["row"]
            metrics.append({**identity(row), **rmse_export_values(trial, report), "result": row.get("result"), "passed": row.get("passed"), "arrived": row.get("arrived"),
                            "cross_track_initial_rmse_m": nested(row, "cross_track_initial_m", "rms"),
                            "cross_track_initial_count": nested(row, "cross_track_initial_m", "count"),
                            "observed_duration_sim_s": row.get("duration_sim_s"),
                            "arrival_duration_sim_s": row.get("duration_sim_s") if row.get("arrived") is True and row.get("result") == "completed" else None,
                            "minimum_footprint_clearance_m": row.get("minimum_footprint_clearance_m"),
                            "baseline_eligible": successful(row) if row["mode"] == "cpp" else None,
                            "summary_path": trial["source"]["summary_path"], "summary_sha256": trial["source"]["summary_sha256"]})
        write_csv(output / "trial_metrics.csv", metrics, common + ["result", "passed", "arrived", "cross_track_initial_rmse_m",
                  "cross_track_initial_count", "observed_duration_sim_s", "arrival_duration_sim_s", "minimum_footprint_clearance_m",
                  "baseline_eligible", "summary_path", "summary_sha256",
                  "selected_rmse_metric", "selected_rmse_m", "legacy_wall_rmse_m", "source_time_trapezoid_rmse_m",
                  "source_time_status", "source_time_reasons", "source_sample_count", "source_span_s", "source_max_gap_s",
                  "source_start_coverage_gap_s", "source_end_coverage_gap_s", "source_error_p95_m", "source_error_max_m"])
        write_csv(output / "pairwise_ratios.csv", ratios, ["controller", "scenario", "repeat", "metric", "python", "cpp", "ratio", "limit",
                  "zero_baseline_atol", "method", "comparable", "passed", "pair_comparable", "pair_passed", "reason", "reasons", "selected_rmse_metric"])
        manifest = {"generated_utc": datetime.now(timezone.utc).isoformat(), "script": str(Path(__file__).resolve()),
                    "script_sha256": sha256(Path(__file__)), "report": str(report_path), "report_sha256": sha256(report_path),
                    "matplotlib_version": version, "python_version": sys.version, "path_mappings": mappings,
                    "max_track_gap_sim_s": args.max_track_gap_s, "summary_verdict_unchanged": report.get("summary"),
                    "rmse_metric": rmse_metric_name(report), "tracking_protocol": report.get("method", {}).get("tracking_protocol"),
                    "frame_contract": report.get("method", {}).get("frame_contract"),
                    "sources": [t["source"] for t in trials], "excluded_scope": skipped, "issues": issues, "figures": exported,
                    "definitions": ["No raw samples, summaries, or acceptance verdicts are rewritten.",
                                    "Python observed trials are retained, including failed trials; valid C++ means completed and passed.",
                                    "Trajectory segments break on invalid samples, simulation clock rollback or configured sample gaps; duplicate samples remain.",
                                    "Coordinates are observed vessel centres, not full-footprint clearance or a swept-area proof.",
                                    "Cylinder shapes come only from actual obstacle_spawned events; no inferred world or costmap geometry.",
                                    "Percentile markers are reported rolling/history windows, never pooled percentiles or message intervals.",
                                    "Empty CSV cells mean unavailable, never imputed zero; unfinished durations are not arrival durations.",
                                    "Ratios and verdicts are read from the input comparison; unavailable/zero-baseline ratios remain absent."]}
        manifest["tables"] = [{"path": p.name, "sha256": sha256(p)} for p in sorted(output.glob("*.csv"))]
        (output / "figure_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"Exported {len(exported)} figures and 4 CSV tables to {output}; acceptance verdicts unchanged.")
        return 0
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        print(f"plot acceptance: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
