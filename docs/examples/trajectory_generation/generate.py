"""Illustrate the current LOS reference; this is not a vessel tracking trial."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
for relative in ("src/core/shared/common", "src/core/decision_module/trajectory_planner"):
    sys.path.insert(0, str(ROOT / relative))

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner import LosTrajectoryPlanner


def main():
    shared_config = ROOT / "src/core/shared/usv_navigation/config"
    config_file = shared_config / "trajectory_planner/trajectory_planner_params.yaml"
    model_file = shared_config / "robots/otter/dynamics_otter.yaml"
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))["trajectory_planner"]["los"]
    model = USVModel.from_mapping(yaml.safe_load(model_file.read_text(encoding="utf-8"))["dynamics"])
    # Hand-specified geometric route, no obstacles and no A* call in this example.
    path = np.array([[0., 0., 0.], [8., 0., np.pi/2], [8., 12., np.pi/2]])
    state = VesselState(x=0., y=0., psi=0., u=.2, v=0., r=0.,
                        stamp=100., frame_id="otter/map")
    result = LosTrajectoryPlanner(model, config).plan(path, state)
    if result.status != AlgorithmStatus.SUCCESS:
        raise RuntimeError(result.message)
    values = result.trajectory.points
    assert values.shape[1] == 8 and np.isfinite(values).all()
    assert np.all(np.diff(values[:, 0]) > 0)
    assert np.all(np.abs(values[:, 5]*values[:, 6]) <= config["yaw_rate_max"]+1e-9)
    np.savetxt(OUT / "trajectory.csv", values, delimiter=",", comments="",
               header="t_s,s_m,x_m,y_m,yaw_rad,kappa_inv_m,speed_mps,accel_mps2")
    source = ROOT / "src/core/decision_module/trajectory_planner/trajectory_planner/los_trajectory_planner.py"
    metadata = {
        "purpose": "Illustrative LOS reference from a hand-specified route; not A*, closed-loop, or sea-trial data.",
        "state": asdict(state), "route": path.tolist(), "los_config": config,
        "model": asdict(model), "source": source.relative_to(ROOT).as_posix(),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "rows": len(values), "trajectory_stamp": result.trajectory.stamp,
        "selected_rows": values[[0, 1, 2, 10, 20, len(values)-1]].tolist(),
    }
    (OUT / "example.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")

    cjk = Path("C:/Windows/Fonts/msyh.ttc")
    if cjk.exists():
        font = FontProperties(fname=str(cjk))
        plt.rcParams["font.family"] = font.get_name()
    plt.rcParams.update({"font.size": 10, "axes.unicode_minus": False,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.facecolor": "white"})
    fig = plt.figure(figsize=(11.5, 6.3), layout="constrained")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1.])
    spatial = fig.add_subplot(gs[:, 0])
    speed = fig.add_subplot(gs[0, 1])
    heading = fig.add_subplot(gs[1, 1], sharex=speed)
    color = "#087e8b"
    spatial.plot(path[:, 0], path[:, 1], "--", color="#737d87", lw=1.8,
                 marker="s", ms=5, label="给定几何路径（本例手工指定）")
    spatial.plot(values[:, 2], values[:, 3], color=color, lw=2.2,
                 label="LOS 生成的期望轨迹")
    spatial.scatter(values[:, 2], values[:, 3], color=color, s=15, zorder=3)
    selected = [0, 10, 20, len(values)-1]
    for index in selected:
        x, y, yaw = values[index, [2, 3, 4]]
        spatial.annotate(f"{values[index, 0]:g} s", (x, y), xytext=(-32, 8),
                         textcoords="offset points", fontsize=9)
        spatial.quiver(x, y, np.cos(yaw), np.sin(yaw), angles="xy", scale_units="xy",
                       scale=1.5, width=.006, color=color, zorder=4)
    spatial.annotate("最终目标", path[-1, :2], xytext=(-60, 5), textcoords="offset points")
    spatial.set(xlabel="x：东向位置 / m", ylabel="y：北向位置 / m",
                title="① 路径 → 带时间的期望轨迹", xlim=(-2, 11), ylim=(-1, 14))
    spatial.set_aspect("equal", adjustable="box")
    spatial.legend(loc="upper left", fontsize=9)
    spatial.grid(alpha=.18)

    speed.plot(values[:, 0], values[:, 6], color=color, lw=2, marker=".", ms=4)
    speed.scatter([0], [state.u], color="#c04d30", marker="x", s=45,
                  label="示例输入 u=0.2 m/s（手工设定初态）", zorder=4)
    speed.set(title="② 每个时刻对应期望速度", ylabel="参考速度 / (m/s)",
              ylim=(0, 1.12), xlim=(0, config["trajectory_time"]))
    speed.legend(loc="lower left", fontsize=8.5)
    speed.grid(alpha=.18)
    heading.plot(values[:, 0], values[:, 4], color="#6458a6", lw=2, marker=".", ms=4)
    heading.set(title="③ 艏向逐步改变（限制转向率）", xlabel="相对轨迹起始时刻 / s",
                ylabel="参考 yaw / rad")
    heading.grid(alpha=.18)
    fig.suptitle("现有 LOS 算法的单轮参考生成示例\n20 s 时域 · 每 0.5 s 保存一点 · 图中没有实船跟踪结果", fontsize=14)
    fig.savefig(OUT / "trajectory_generation.png", dpi=160)
    plt.close(fig)
    print(json.dumps({"rows": len(values), "selected_rows": metadata["selected_rows"],
                      "output": str(OUT / "trajectory_generation.png")}, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
