# python-grid-interior-001：提前停止矩阵的 OS 旁路记录

本报告仅记录实际进程性能观察，不能视为 18 例完整矩阵通过证明。矩阵执行者报告 PID 圆柱第 2 次出现负扫掠间隙并请求停止；冻结的事件目录最终包含 9 个 PID 航行窗口及一个没有完整航行窗口的 LQR 启动记录。安全间隙、碰撞定义和退出健康以原验收日志另行判断，不用性能数据覆盖失败记录。

采样器 `scripts/validation/process_sampler.py` 每秒只读 `/proc`，唯一目标为 harness PID 97594（starttime=7157953 ticks），sampler PID 97996；没有发信号、附加调试器或修改 WSL 源码/install。随目标退出自动结束。

## 实际覆盖

- 251 个快照；单调时间从 `71611.055212315` 到 `71861.057390735`，包络 **250.002178 s**，采样器总运行 **251.016245 s**。
- PID 直线第 1 次航行结束于 `71611.009848127`，比首采样早 **45.364188 ms**，因此完全不纳入该例航行的 OS 性能统计。按主任务要求没有补跑或回填。
- PID 直线第 2–3 次、转弯 3 次、单圆柱 3 次的事件航行窗口被采样包络涵盖，共 **8 个 PID 窗口**。其中失败试验仍保留，不能把窗口覆盖解释为试验成功。
- LQR 启动记录没有完整 goal→acquisition_finished 窗口，不纳入下表；更不能声称剩余 LQR 试验得到验证。

## 航行窗口性能

下表排除启动、清理以及跨航行窗口边界的 CPU 差分，共 181 个进程快照、173 个完整 CPU 区间/主要角色。100% CPU 表示占满一个逻辑核。角色 CPU 均值按实际观察的 CPU 时间/进程区间时长计算，P95 为 1 Hz 区间利用率分位数。

| 角色 | CPU 均值 | CPU P95 | 观察 CPU 最大值 | RSS 观察峰值 / MiB |
| --- | ---: | ---: | ---: | ---: |
| 导航主进程 | 4.21% | 6.00% | 8.00% | 69.81 |
| 独立规划 worker | 2.88% | 5.40% | 25.01% | 45.10 |
| LiDAR 建图 | 34.69% | 41.00% | 44.03% | 68.12 |
| C++ costmap 支撑 | 0.88% | 2.00% | 2.00% | 36.16 |
| Gazebo | 46.14% | 51.59% | 55.03% | 542.94 |
| 验收 harness | 5.26% | 7.00% | 7.01% | 80.82 |

导航与规划 worker 同刻 CPU 之和：均值 **7.10%**、P95 **11.01%**、观察最大值 **30.01%**；二者同刻 RSS 和峰值 **114.06 MiB**。资源跟踪辅助进程另约 11 MiB。RSS 相加可能重复计算共享页，不是 USS/PSS。

1 Hz 快照无法捕获所有瞬时峰值、短命进程或新进程首次观察前的 CPU。CPU 包含 ROS 回调和进程其他工作，不能当作纯算法计时或 WCET。该次为 PID 窗口，不能直接与其他矩阵混合控制器窗口的均值作算法优劣比较。

采样开销均值 **1.748 ms**、P95 **2.346 ms**、最大 **4.871 ms**；记录的采样开销约占观察墙钟 **0.175%**。相邻快照实际间隔 **0.986081–1.014454 s**；census 读取错误计数为 0。UTC 只作定位，持续时间使用单调时钟。

## 旁路文件与复算

WSL 目录：`/home/usv/usv_migration/results/`。

| 文件 | SHA256 |
| --- | --- |
| `python-grid-interior-001-processes.jsonl` | `f5ffa6eefae2b9b9fb5ac55a9bf3f7a63f1f2cad199730bd950de0facdf251e8` |
| `python-grid-interior-001-processes-summarize.py` | `5f38fcca964ebbdc0a27854c0c8face6bd89d4af52fdd3efe5cc6c2d0f15ff39` |
| 运行时 sampler 源脚本 | `ac6a30158f2e51d4b54f9f030a543418b3d3a1d45c2e0187c0077b3fc59657dc` |

汇总结果为 `python-grid-interior-001-processes-summary.json`，内含原始文件及逐 trial 事件 SHA256、覆盖分类、所有观察/航行窗口/故障窗口的 CPU、RSS、VM 统计。原 JSONL 保留逐进程 PID/PPID、命令、角色、utime/stime 和单调时间，不删除失败时段。

```bash
python3 /home/usv/usv_migration/results/python-grid-interior-001-processes-summarize.py \
  /home/usv/usv_migration/results/python-grid-interior-001-processes.jsonl \
  /home/usv/usv_migration/results/python-grid-interior-001 \
  /home/usv/usv_migration/results/python-grid-interior-001-processes-summary-recomputed.json
```

复算要求新输出文件，拒绝覆盖已有报告。本次提前停止后 M5 删除与干净构建已暂停，没有以该性能结果放行清理。
