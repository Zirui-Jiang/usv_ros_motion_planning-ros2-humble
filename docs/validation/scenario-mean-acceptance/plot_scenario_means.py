#!/usr/bin/env python3
"""Export an explicit scenario_mean report; never select or re-score trials.

Run only when no performance acquisition is active. Error bars are sample SD,
not confidence intervals. Ineligible groups remain descriptive/incomparable.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


SCENARIOS = ("straight", "turn", "single_obstacle")
CONTROLLERS = ("pid", "lqr")
COLORS = {"python": "#0072B2", "cpp": "#777777"}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new directory")
    parser.add_argument("--title", default="Fixed three-repeat scenario means")
    args = parser.parse_args(argv)
    report_path = args.report.resolve()
    report = json.loads(report_path.read_text(encoding="utf-8"))
    aggregate = report.get("scenario_acceptance", {})
    if aggregate.get("aggregation") != "scenario_mean" or aggregate.get("required_repeats") != [1, 2, 3]:
        parser.error("requires an explicit scenario_mean report with fixed repeats 1,2,3")
    output = args.output.resolve()
    if output.exists():
        parser.error("output must be a new directory; existing exports are never overwritten")
    for source in report.get("sources", []):
        raw = Path(source["directory"]).resolve()
        if output == raw or raw in output.parents:
            parser.error("output cannot be inside a raw input directory")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    groups = {(group["controller"], group["scenario"]): group for group in aggregate["groups"]}
    metrics = (("cross_track_rmse_m", "RMSE (m)"), ("duration_sim_s", "Arrival time (s)"))
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.6))
    fig.subplots_adjust(left=.075, right=.98, bottom=.10, top=.80, hspace=.32, wspace=.23)
    csv_rows = []
    for row_index, controller in enumerate(CONTROLLERS):
        for column_index, (metric, ylabel) in enumerate(metrics):
            ax = axes[row_index, column_index]
            upper, lower = 0., 0.
            for scenario_index, scenario in enumerate(SCENARIOS):
                group = groups.get((controller, scenario))
                if group is None:
                    ax.text(scenario_index, .96, "not observed", ha="center", va="top", transform=ax.get_xaxis_transform())
                    continue
                if group["comparable"]:
                    comparison = group["metrics"][metric]
                    ratio = comparison.get("ratio")
                    ratio_text = f"{ratio:.3f}x" if finite(ratio) else "zero-baseline atol"
                    note = ratio_text + (" / PASS" if comparison["passed"] else " / FAIL")
                else:
                    note = "incomparable"
                ax.text(scenario_index, .96, note, ha="center", va="top", fontsize=9,
                        transform=ax.get_xaxis_transform())
                for mode, offset in (("python", -.14), ("cpp", .14)):
                    values = group[mode]["metrics"][metric]
                    mean, sd = values.get("mean"), values.get("std_ddof1")
                    center = scenario_index + offset
                    if finite(mean):
                        spread = sd if finite(sd) else 0.
                        upper, lower = max(upper, mean + spread), min(lower, mean - spread)
                        ax.errorbar(center, mean, yerr=sd if finite(sd) else None, color=COLORS[mode],
                                    fmt="_", markersize=17, linewidth=1.7, capsize=5, zorder=3)
                    for repeat, value in enumerate(values["values"], 1):
                        fixed = group[mode]["fixed_repeats"][repeat - 1]
                        passed = fixed["completed_and_passed"]
                        if finite(value):
                            upper = max(upper, value)
                            ax.scatter(center + (repeat - 2) * .035, value, marker="o" if passed else "x",
                                       s=27, color=COLORS[mode], alpha=.85, zorder=4)
                        record = {"controller": controller, "scenario": scenario, "mode": mode,
                            "aggregation": "scenario_mean", "rmse_metric": aggregate["rmse_metric"],
                            "metric": metric, "repeat": repeat, "value": value,
                            "trial_completed_and_passed": passed, "source_time_status": fixed["source_time_status"],
                            "count": values["count"], "mean": mean, "std_ddof1": sd, "worst": values["worst"],
                            "group_comparable": group["comparable"], "group_passed": group["passed"],
                            "group_reasons": "; ".join(group["reasons"])}
                        if metric == "cross_track_rmse_m":
                            record["legacy_wall_rmse_m"] = group[mode]["metrics"]["legacy_wall_rmse_m"]["values"][repeat - 1]
                        csv_rows.append(record)
            span = max(upper - lower, .01)
            ax.set_ylim(lower - span * .04, upper + span * .25)
            ax.set_xlim(-.55, 2.55)
            ax.set_xticks(range(3), ["Straight", "Turn", "Single obstacle"])
            ax.set_title(f"{controller.upper()} / {ylabel}", loc="left", fontsize=12)
            ax.set_ylabel(ylabel)
            ax.grid(axis="y", alpha=.2)
            for spine in ("top", "right"):
                ax.spines[spine].set_visible(False)
    summary = aggregate["summary"]
    fig.suptitle(args.title + "\n" + aggregate["rmse_metric"] +
        f"; eligible groups {summary['passed_groups']}/{summary['eligible_groups']} pass; {summary['incomparable_groups']} incomparable\n" +
        "Mean +/- sample SD (ddof=1), with all fixed repeats; SD is not a confidence interval.\n" +
        "Failed baseline values are descriptive only. Functional/performance gates and legacy results remain separate.", fontsize=11)
    legend = [Line2D([], [], marker="o", linestyle="none", color=COLORS[mode], label=mode.upper()) for mode in ("python", "cpp")]
    legend.append(Line2D([], [], marker="x", linestyle="none", color="#777777", label="Functional acceptance failed"))
    fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(.5, .01), ncol=3, frameon=False)
    output.mkdir(parents=True)
    fig.savefig(output / "scenario_means.png", dpi=300)
    fig.savefig(output / "scenario_means.pdf")
    plt.close(fig)
    fields = list(dict.fromkeys(key for record in csv_rows for key in record))
    with (output / "scenario_means.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(csv_rows)
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = {"source_report": str(report_path), "source_report_sha256": digest(report_path),
                "script_sha256": digest(Path(__file__)), "aggregation": "scenario_mean",
                "rmse_metric": aggregate["rmse_metric"], "required_repeats": [1, 2, 3],
                "error_bars": "sample standard deviation ddof=1, not confidence interval",
                "failed_or_missing_baseline": "incomparable; observed failed values remain descriptive; no imputation",
                "csv_rows": len(csv_rows), "visual_inspection": "not performed by script",
                "outputs": {name: digest(output / name) for name in ("scenario_means.png", "scenario_means.pdf", "scenario_means.csv")}}
    (output / "scenario_mean_figure_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
