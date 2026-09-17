"""Plot only captured maps and stored offline trajectories; no synthetic results."""
from pathlib import Path
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Circle, Polygon, Patch
from matplotlib.lines import Line2D
import numpy as np

ROOT = Path(__file__).resolve().parent


def rectangle(row):
    c, s = math.cos(row[4]), math.sin(row[4])
    corners = np.asarray([[-1.1, -.6], [1.1, -.6], [1.1, .6], [-1.1, .6]])
    return corners @ np.asarray([[c, s], [-s, c]]) + row[2:4]


def main():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.5))
    geometry = json.loads((ROOT / "footprint_map_diagnostic.json").read_text(encoding="utf-8"))["results"]
    specifications = [
        ("switch1_failure_001", "offline_combinations_immediate_map.json", 0,
         "A  Soft-cost rejection; hard geometry is clear", "#17608b", 0),
        ("obstacle_raw_002", "los_replay.json", 0,
         "B  A physically intersecting rollout", "#ac3039", 4),
    ]
    cmap = ListedColormap(["#ffffff", "#efefef", "#efc8a2", "#424242", "#c1cbd2"])
    for ax, (folder, report_name, index, title, color, geometry_index) in zip(axes, specifications):
        gap = geometry[geometry_index]["predicted_minimum_physical_cylinder_gap_m"]
        source = ROOT / folder
        report = json.loads((source / report_name).read_text(encoding="utf-8"))
        row = report["comparisons"][index]
        trajectory = np.asarray(row["trajectory"], dtype=float)
        with np.load(next(source.glob("costmap_input_*.npz")), allow_pickle=False) as snapshot:
            raw = snapshot["data"]
            res, ox, oy = (float(snapshot[k]) for k in ("resolution", "origin_x", "origin_y"))
            assert str(snapshot["cost_kind"]) == "raw"
            x0, x1 = int((21.5 - ox) // res), int((36.0 - ox) // res) + 1
            y0, y1 = int((-13. - oy) // res), int((-5.8 - oy) // res) + 1
            crop = raw[y0:y1, x0:x1]
            classes = np.zeros_like(crop)
            classes[(crop > 0) & (crop < 127)] = 1
            classes[(crop >= 127) & (crop < 254)] = 2
            classes[crop == 254] = 3
            classes[crop == 255] = 4
            ax.imshow(classes, origin="lower", interpolation="nearest", cmap=cmap,
                      norm=BoundaryNorm(np.arange(-.5, 5.5), cmap.N),
                      extent=(ox + x0 * res, ox + x1 * res, oy + y0 * res, oy + y1 * res))
            map_stamp = float(snapshot["stamp"])
        ax.add_patch(Circle((29, -10), 1, fill=False, edgecolor="black", linewidth=1.5, zorder=5))
        ax.plot(trajectory[:, 2], trajectory[:, 3], color=color, linewidth=2, zorder=6)
        for sample in sorted(set([0, 5, 10, 18, 26])):
            if sample < len(trajectory):
                ax.add_patch(Polygon(rectangle(trajectory[sample]), closed=True,
                                     facecolor=color, alpha=.11, edgecolor="none", zorder=4))
                ax.add_patch(Polygon(rectangle(trajectory[sample]), closed=True,
                                     fill=False, edgecolor=color, linewidth=.85, zorder=7))
        ax.scatter(*trajectory[0, 2:4], marker="o", s=22, color=color, zorder=8)
        if folder == "switch1_failure_001":
            bad = row["first_bad_sample"]
            ax.scatter(*bad["xy"], facecolor="white", edgecolor=color, s=45, zorder=9)
            ax.annotate("t = 2.5 s; raw cost = 162", xy=bad["xy"], xytext=(23.1, -6.3),
                        fontsize=8.4, arrowprops={"arrowstyle": "->", "color": color}, color=color)
            settings = "LOS distance = 1 m; switch = 1 m; yaw limit = 0.3 rad/s"
        else:
            settings = "LOS distance = 5 m; switch = 5 m; yaw limit = 0.3 rad/s"
        ax.set_title(title, loc="left", fontsize=10.5, fontweight="bold", pad=12)
        ax.text(.02, .025, f"Predicted swept hull-to-cylinder gap: {gap:+.3f} m\n"
                f"Captured raw-map stamp: {map_stamp:g} s\n{settings}",
                transform=ax.transAxes, fontsize=8, va="bottom",
                bbox={"facecolor": "white", "alpha": .94, "edgecolor": "#dddddd", "pad": 4})
        ax.set(xlim=(21.5, 36), ylim=(-13, -5.8), xlabel="World x (m)", ylabel="World y (m)")
        ax.set_aspect("equal")
        ax.tick_params(labelsize=8)
    handles = [Patch(facecolor="#efefef", label="Soft raw cost 1–126"),
               Patch(facecolor="#efc8a2", label="Soft raw cost 127–253"),
               Patch(facecolor="#424242", label="Hard raw cost 254 (full cell area)"),
               Patch(facecolor="#c1cbd2", label="Unknown raw cost 255 (blocked)"),
               Line2D([0], [0], color="black", label="Spawned cylinder, radius 1 m"),
               Patch(facecolor="none", edgecolor="#17608b", label="Sampled 2.2 × 1.2 m hull rectangles")]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=8.1,
               bbox_to_anchor=(.5, .075))
    fig.suptitle("Captured-map diagnosis: soft costs and full-hull collision", fontsize=13, y=.965)
    fig.text(.5, .023, "Measured Gazebo maps; stored offline LOS rollouts. Single-snapshot geometry checks are not closed-loop acceptance results.",
             ha="center", fontsize=8.5)
    fig.subplots_adjust(top=.84, bottom=.23, left=.055, right=.985, wspace=.16)
    for suffix in ("png", "pdf"):
        fig.savefig(ROOT / ("footprint_cost_semantics." + suffix), dpi=300, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
