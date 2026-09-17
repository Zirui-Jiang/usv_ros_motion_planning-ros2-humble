#!/usr/bin/env python3
"""Render mission trial logs into standalone figures; never invent missing data."""
import argparse
import json
import math
from pathlib import Path


def plot_trial(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    metrics = json.loads((directory/"metrics.json").read_text(encoding="utf-8"))
    rows = [json.loads(line) for line in (directory/"events.jsonl").read_text(encoding="utf-8").splitlines()]
    samples = [r for r in rows if r["event"] == "mission_sample"]
    if not samples:
        return {"trial": directory.name, "plots": [], "reason": "no measured mission samples"}
    commands = [r for r in rows if r["event"] == "thruster_command"]
    target = metrics["config"]["target"]
    figure, axes = plt.subplots(3, 1, figsize=(9, 11), constrained_layout=True)
    phase_colors = {"ORBITING": "#167b9b", "AVOIDING": "#c75e2c", "STATION_KEEP": "#186a3b",
                    "STANDBY": "#6c3483", "RETURN_CENTER": "#555555",
                    "BRAKING_ESCAPE": "#777777", "BACKING_ESCAPE": "#b33240",
                    "ALIGN_ESCAPE": "#756bb1", "ALIGN_ENTRY": "#9e9ac8",
                    "WAITING_CLEARANCE": "#8c564b"}
    trajectory = axes[0]
    trajectory.plot([s["pose"][0] for s in samples], [s["pose"][1] for s in samples], color="0.75", lw=1)
    for phase in dict.fromkeys(s["phase"] for s in samples):
        points = [s for s in samples if s["phase"] == phase]
        trajectory.scatter([s["pose"][0] for s in points], [s["pose"][1] for s in points],
                           s=4, label=phase or "unknown", color=phase_colors.get(phase, "#927733"))
    trajectory.scatter([target[0]], [target[1]], marker="x", s=70, color="black", label="mission centre")
    if metrics["config"]["mode"] == "orbit":
        radius = metrics["config"]["orbit_radius"]
        trajectory.add_patch(plt.Circle(target[:2], radius, fill=False, linestyle="--", color="black", lw=1))
    for row in rows:
        if row["event"] == "controlled_obstacle_spawned" and row.get("success"):
            obstacle = row["obstacle"]
            trajectory.add_patch(plt.Circle((obstacle["x"], obstacle["y"]), obstacle["radius"], color="#c75e2c", alpha=0.5))
    trajectory.set(xlabel="World x (m)", ylabel="World y (m)", aspect="equal")
    # Keep the phase key outside the measured route and controlled obstacle.
    trajectory.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=7)
    t0 = samples[0]["sim_time_s"]
    times = [s["sim_time_s"]-t0 for s in samples]
    axes[1].plot(times, [s.get("radial_error_m", s["position_error_m"]) for s in samples], label="radial error" if metrics["config"]["mode"] == "orbit" else "position error")
    axes[1].axhline(0.5, color="0.3", linestyle="--", lw=1,
                    label="normal-orbit threshold (0.5 m)" if metrics["config"]["mode"] == "orbit"
                    else "position threshold (0.5 m)")
    axes[1].set(xlabel="Simulation time since mission samples (s)", ylabel="Position / radial error (m)")
    if metrics["config"]["mode"] == "station_keep":
        heading = axes[1].twinx()
        heading.plot(times, [s["heading_error_deg"] for s in samples], color="#c75e2c", label="heading error", alpha=0.8)
        heading.axhline(5.0, color="#c75e2c", linestyle=":", lw=1)
        heading.set_ylabel("Heading error (deg)")
        heading.legend(loc="upper right", fontsize=8)
    for window in metrics.get("windows", {}).values():
        axes[1].axvspan(window["start_sim_s"]-t0, window["end_sim_s"]-t0, color="#186a3b", alpha=0.07)
    for row in rows:
        if row["event"] == "lateral_velocity_impulse_0_4mps_requested":
            axes[1].axvline(row["sim_time_s"]-t0, color="#c75e2c", linestyle="-.", linewidth=1,
                            label="observed lateral velocity impulse request")
    axes[1].legend(loc="upper left", fontsize=8)
    # Original command records carry wall time; retain that honest time base.
    if commands:
        wall0 = commands[0]["wall_monotonic_s"]
        for axis in ("left", "right", "lateral"):
            selected = [r for r in commands if r["axis"] == axis and r["normalized_command"] is not None]
            axes[2].plot([r["wall_monotonic_s"]-wall0 for r in selected],
                         [r["normalized_command"] for r in selected], label=axis, linewidth=0.9)
    axes[2].set(xlabel="Wall time since first observed command (s)", ylabel="Normalized thrust command", ylim=(-1.05, 1.05))
    axes[2].legend(loc="best", fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.2)
    figure.suptitle(f"{metrics['trial']} — {metrics['result'].upper()}", fontsize=13)
    outputs = []
    for extension in ("png", "pdf"):
        path = directory/("mission_plots."+extension)
        figure.savefig(path, dpi=180)
        outputs.append(str(path))
    plt.close(figure)
    progress = metrics.get("independent_orbit_progress", {})
    recorded = progress.get("net_progress_samples", [])
    if recorded:
        figure, axis = plt.subplots(figsize=(9, 3.5), constrained_layout=True)
        axis.plot([row["sim_time_s"]-t0 for row in recorded],
                  [row["independent_net_angle_rad"]/(2*math.pi) for row in recorded], label="independent signed odometry laps")
        required = progress["required_angle_rad"]/(2*math.pi)
        allowance = progress["boundary_angle_tolerance_rad"]/(2*math.pi)
        axis.axhline(required, color="black", linestyle="--", label="requested laps")
        axis.axhspan(required-allowance, required, color="0.7", alpha=0.3, label="bounded phase sampling uncertainty")
        axis.set(xlabel="Simulation time since mission samples (s)", ylabel="Signed net laps",
                 title=f"{metrics['trial']} — independent odometry verification")
        axis.grid(alpha=0.2)
        axis.legend(loc="best", fontsize=8)
        for extension in ("png", "pdf"):
            path = directory/("independent_orbit_progress."+extension)
            figure.savefig(path, dpi=180)
            outputs.append(str(path))
        plt.close(figure)
    return {"trial": directory.name, "plots": outputs}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="run root or individual trial directory")
    args = parser.parse_args(argv)
    directories = [args.run] if (args.run/"metrics.json").is_file() else sorted(p.parent for p in args.run.glob("*/metrics.json"))
    results = [plot_trial(directory) for directory in directories]
    print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
