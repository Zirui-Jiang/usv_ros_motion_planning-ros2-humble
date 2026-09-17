"""在源码树直接运行一个 Hybrid A* 圆弧规划，不需要 ROS。"""
import argparse
from pathlib import Path
import sys

import numpy as np

package_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(package_root))
sys.path.insert(0, str(package_root.parent / "path_processor"))
sys.path.insert(0, str(package_root.parent.parent / "shared" / "common"))

from common.models import GridSnapshot
from path_planner import PathPlannerCore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", type=float, default=4., help="最小回转半径，米")
    parser.add_argument("--output", type=Path, help="可选保存 NumPy .npz 文件")
    args = parser.parse_args()
    radius = args.radius
    if not np.isfinite(radius) or not 0.1 <= radius <= 100.:
        parser.error("示例半径须在 [0.1, 100] 米内")
    resolution = .25
    size = int(np.ceil((2*radius+10)/resolution))
    grid = GridSnapshot(np.zeros((size, size), dtype=np.uint8), resolution, -5., -5.)
    planner = PathPlannerCore({"planner_name": "hybrid_astar",
                               "minimum_turning_radius": radius,
                               "analytic_expansion_max_length": max(20., 2*np.pi*radius),
                               "timeout": 5.})
    result = planner.make_plan((0., 0., 0.), (radius, radius, np.pi/2), grid)
    print(f"状态: {result.status.value}; {result.message}")
    print(f"搜索展开: {result.expanded}; 规划用时: {result.elapsed:.4f} s")
    if not result.success:
        return 1
    length = float(np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1).sum())
    print(f"路径点数: {len(result.path)}; 采样折线长度: {length:.4f} m")
    print(f"参考四分之一圆弧长: {np.pi*radius/2:.4f} m")
    print(f"终点 [x, y, yaw]: {result.path[-1].tolist()}")
    if args.output:
        with args.output.open("wb") as stream:
            np.savez_compressed(stream, path=result.path, data=grid.data,
                                resolution=grid.resolution, origin_x=grid.origin_x,
                                origin_y=grid.origin_y, minimum_turning_radius=radius)
        print(f"已保存: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
