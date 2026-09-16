#!/usr/bin/env python3
# 演示一次纯数值 PID/LQR 调用：构造状态与轨迹，打印归一化推进器命令；不发送实机报文。
"""One numerical PID/LQR/ADRC step; JSON is a demo, not an autopilot protocol."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("pid", "lqr", "adrc"), default="pid")
    parser.add_argument("--controller-config", type=Path, help="project controller YAML; ADRC uses the repository Otter YAML when available")
    args = parser.parse_args()
    here = Path(__file__).resolve()
    workspace = here.parents[1]
    if (workspace / "src/controller/controller").is_dir():
        package_roots = [workspace / "src" / name for name in ("common", "trajectory_planner", "controller")]
    else:
        repository = here.parents[2]
        package_roots = [repository / relative for relative in (
            "src/core/shared/common", "src/core/decision_module/trajectory_planner", "src/core/control_module/controller")]
    for package_root in package_roots:
        sys.path.insert(0, str(package_root))

    from common import USVModel, VesselState
    from controller import create_controller
    from trajectory_planner import Trajectory

    # 示例仅计算一个 0.1 秒控制步，不积分真实船舶运动。
    dt = 0.1
    # Current PID calibration has nonzero gains; PID constructor defaults are zero.
    config = {
        "pid": {"goal_distance_tolerance": 1.5, "goal_speed_tolerance": 0.25,
                "lookahead_time": 2.6, "k_p_speed": 2.0, "k_i_speed": 0.5,
                "k_d_speed": 0.0, "k_p_course": 2.0, "k_i_course": 0.0, "k_d_course": 0.0},
        "lqr": {"goal_distance_tolerance": 1.0, "goal_speed_tolerance": 0.25,
                "lookahead_time": 2.0, "discrete_dt": dt, "solve_eps": 0.1,
                "max_iterations": 5000, "solve_timeout": 0.08,
                "Q_matrix_diag": [1, 1, 100, 10, 10, 10], "R_matrix_diag": [20, 20]},
        "adrc": {},  # 原 0914 增益、物理力/矩限幅；适配层默认值与专用 YAML 一致。
    }[args.controller]
    config_path = args.controller_config
    if config_path is None and args.controller == "adrc":
        candidate = here.parents[2] / "src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
        if candidate.is_file():
            config_path = candidate
    config_source = "built-in demo parameters" if args.controller != "adrc" else "ADRC source defaults (standalone export fallback)"
    if config_path is not None:
        import yaml
        config_path = config_path.resolve()
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["controller"][args.controller + "_controller"]
        config_source = str(config_path)
    controller = create_controller(args.controller, USVModel(), config)
    if args.controller == "adrc":
        dt = controller.config["sample_time"]
    state = VesselState(x=0.0, y=0.1, psi=0.05, u=0.3, v=0.0, r=0.0,
                        stamp=100.0, frame_id="otter/map")
    # Columns: relative t (s), arc length (m), x/y (m), yaw (rad), curvature
    # (1/m), reference speed (m/s), reference acceleration (m/s^2).
    # 八列依次为相对时间、弧长、x、y、艏向、曲率、参考速度与参考加速度。
    trajectory = Trajectory([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0],
        [5.0, 4.0, 4.0, 0.0, 0.0, 0.0, 0.8, 0.0],
        [10.0, 8.0, 8.0, 0.0, 0.0, 0.0, 0.8, 0.0],
    ], stamp=100.0, frame_id="otter/map", goal=(8.0, 0.0))
    command = controller.compute(state, trajectory, dt)
    record = {"demo_only": True, "controller": args.controller, "dt_s": dt,
              "state": asdict(state), "command": asdict(command),
              "config_source": config_source, "effective_config": controller.config}
    if args.controller == "adrc":
        record["physical_output"] = {"surge_force_N": controller.speed_loop.surge_force,
                                      "yaw_moment_Nm": controller.heading_loop.yaw_moment}
    print(json.dumps(record, ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if command.is_valid() and command.status.value == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
