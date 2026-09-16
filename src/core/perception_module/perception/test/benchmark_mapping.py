# 可复现建图内核基准：固定长射线输入，报告毫秒耗时及周期预算，不包含 ROS 序列化或 TF 查询。
"""Reproducible kernel latency benchmark, excluding ROS serialization/TF.

Run from the repository root with perception on PYTHONPATH, for example:
python src/core/perception/test/benchmark_mapping.py --backend python
"""

import argparse
import json
import math
import platform
import time

import numpy as np

from perception.mapping import GridMapper


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", default="python")
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--beams", type=int, default=2000)
    parser.add_argument("--frequency", type=float, default=15.0)
    parser.add_argument("--range-max", type=float, default=30.0)
    args = parser.parse_args()
    grid = GridMapper(backend=args.backend)
    # Long valid hits and no-returns exercise the longest normal rays. The
    # distribution is fixed and its maximum range is reported with results.
    ranges = np.full(args.beams, args.range_max - 0.1)
    ranges[::7] = np.inf
    scan_args = dict(angle_min=-math.pi, angle_increment=2 * math.pi / args.beams,
                     range_min=0.1, range_max=args.range_max,
                     pose_x=0.0, pose_y=0.0, pose_yaw=0.0)
    grid.integrate_scan(ranges, **scan_args)
    timings = []
    period = 1.0 / args.frequency
    for _ in range(args.samples):
        start = time.perf_counter()
        grid.integrate_scan(ranges, **scan_args)
        elapsed = time.perf_counter() - start
        timings.append(elapsed * 1000)
        if elapsed < period:
            time.sleep(period - elapsed)
    print(json.dumps(dict(backend=grid.backend, python=platform.python_version(),
                          machine=platform.machine(), grid_shape=list(grid.data.shape),
                          resolution=grid.config.resolution, beams=args.beams,
                          frequency_hz=args.frequency, range_max_m=args.range_max,
                          samples=args.samples, p50_ms=float(np.percentile(timings, 50)),
                          p95_ms=float(np.percentile(timings, 95)),
                          maximum_ms=max(timings), budget_ms=1000 / args.frequency), indent=2))


if __name__ == "__main__":
    main()
