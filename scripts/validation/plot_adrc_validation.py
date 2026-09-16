#!/usr/bin/env python3
"""Plot recorded ADRC numerical responses and all completed comparison runs."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


COLORS = {"adrc": "#007C91", "pid": "#BF5B17", "lqr": "#7560A8"}
SCENARIOS = ("straight", "turn", "single_obstacle")


def _save(figure, output, name):
    figure.savefig(output / (name + ".png"), dpi=200)
    figure.savefig(output / (name + ".svg"))
    plt.close(figure)


def numerical(directory, output):
    with (directory / "samples.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    speed = [r for r in rows if r["scenario"] == "speed_1mps"]
    axes[0].plot([float(r["time_s"]) for r in speed], [float(r["state_u_mps"]) for r in speed],
                 color=COLORS["adrc"], label="Measured surge speed")
    axes[0].axhline(1., color="#59616B", linestyle="--", label="Reference: 1 m/s")
    axes[0].set(title="Numerical speed response", xlabel="Time (s)", ylabel="Surge speed (m/s)")
    for name, color, reference in (("heading_plus_30deg", COLORS["adrc"], 30.),
                                   ("heading_minus_30deg", COLORS["pid"], -30.)):
        selected = [r for r in rows if r["scenario"] == name]
        axes[1].plot([float(r["time_s"]) for r in selected],
                     [float(r["state_heading_rad"]) * 180 / 3.141592653589793 for r in selected],
                     color=color, label=f"Measured heading ({reference:+g} deg step)")
        axes[1].axhline(reference, color=color, linestyle="--", alpha=.65)
    axes[1].set(title="Numerical heading response", xlabel="Time (s)", ylabel="Heading (deg)")
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc="best")
        ax.spines[["top", "right"]].set_visible(False)
    _save(fig, output, "numeric_responses")


def trajectories(matrix, output):
    manifest = json.loads((matrix / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((matrix / "summary.json").read_text(encoding="utf-8"))
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5), constrained_layout=True)
    fig.suptitle("Recorded Gazebo tracks — every completed acquisition, no smoothing", fontsize=12)
    for scene_name, ax in zip(SCENARIOS, axes):
        scene = next(s for s in manifest["config"]["scenarios"] if s["name"] == scene_name)
        labels = set()
        for trial in summary["results"]:
            if trial.get("scenario") != scene_name or trial.get("fault"):
                continue
            poses = []
            events_path = matrix / trial["trial"] / "events.jsonl"
            for line in events_path.read_text(encoding="utf-8").splitlines():
                event = json.loads(line)
                if event.get("event") == "sample" and event.get("pose"):
                    poses.append(event["pose"])
            if not poses:
                continue
            algorithm = trial["controller"]
            label = algorithm.upper() if algorithm not in labels else None
            labels.add(algorithm)
            ax.plot([p[0] for p in poses], [p[1] for p in poses], color=COLORS[algorithm],
                    linewidth=1.4, alpha=.7, linestyle="-" if trial.get("passed") else ":", label=label)
            if not trial.get("passed"):
                ax.scatter(*poses[-1][:2], color=COLORS[algorithm], marker="x", s=35)
        start, goal = manifest["config"]["start"], scene["goal"]
        ax.scatter(*start[:2], color="#27313D", marker="o", s=24, label="Start")
        ax.scatter(*goal[:2], color="#27313D", marker="*", s=95, label="Goal")
        for obstacle in scene["obstacles"]:
            ax.add_patch(Circle((obstacle["x"], obstacle["y"]), obstacle["radius"],
                                facecolor="#AAB1B8", edgecolor="#53606B", alpha=.85))
        ax.set(title=scene_name.replace("_", " ").title(), xlabel="Map x (m)", ylabel="Map y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.2)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(fontsize=8)
    _save(fig, output, "gazebo_tracks")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--numeric", type=Path, required=True)
    parser.add_argument("--matrix", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    numerical(args.numeric, args.output)
    if args.matrix:
        trajectories(args.matrix, args.output)


if __name__ == "__main__":
    main()
