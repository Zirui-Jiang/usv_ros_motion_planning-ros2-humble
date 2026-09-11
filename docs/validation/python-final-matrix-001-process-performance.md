# python-final-matrix-001：进程 CPU/RSS 旁路补采

本报告仅对应 WSL `/home/usv/usv_migration/results/python-final-matrix-001` 的实际补采区间，不代表未采样的早期试验或后续矩阵。采样器只读取 `/proc`，每秒观察唯一验收 harness 及其后代，没有发送信号、附加调试器、启动 Gazebo 或修改 WSL 源码/install。导航判据与进程退出健康由该矩阵原始报告另行判断。

## 来源与覆盖

- 源脚本：`scripts/validation/process_sampler.py`。运行时 SHA256：`ac6a30158f2e51d4b54f9f030a543418b3d3a1d45c2e0187c0077b3fc59657dc`。
- harness PID：90044，进程启动标识：7019896 ticks；sampler PID：91977。这些是历史身份，不能用于后续进程控制。
- 采样开始 UTC：`2026-09-11T01:51:07.891180+00:00`。观察包含 346 个快照，首末快照的单调时间包络为 **345.005510 s**；采样器总运行时间为 346.030479 s，随后因原 harness 退出自动结束。UTC 只用于定位，全部持续时间按单调时钟计算。
- LQR 的直线、转弯、单圆柱各 3 次，以及 PID 单圆柱第 3 次：完整航行窗口被采样时间包络涵盖。
- PID 单圆柱第 2 次：仅涵盖末尾 **2.600936 s**，3 个快照。PID 直线 3 次、转弯 3 次及单圆柱第 1 次没有性能采样，不能补算。
- cancel、goal_change 故障窗口被采样时间包络涵盖。odom_dropout 存在原始进程采样，但原事件文件没有标准 `acquisition_finished` 标记，因此不将它并入下表的完整航行窗口统计。

“窗口被包络涵盖”不等于捕获了窗口内每个瞬时活动。快照为 1 Hz，首次看到一个进程时没有前一 CPU 样本，存在时间短于采样间隔的进程也可能未被捕获。

## 实际覆盖航行窗口的统计

采用每例 `goal_published` 至 `acquisition_finished` 的单调时间范围，排除跨越窗口边界的 CPU 差分区间，也不混入启动、清理和故障试验。下列每个主要角色有 191 个进程快照、180 个完整 CPU 差分区间，涉及 11 个导航进程实例；其中包括上述 PID 单圆柱第 2 次的部分窗口，不能将这些数字解释为 11 次完整性能实验。

CPU 的 **100% 表示占满一个逻辑核**，不是整机百分比。本机报告 12 个逻辑核。均值为观察到的 CPU 时间除以对应进程区间总时长；P95 是 1 秒区间利用率的线性插值分位数。

| 角色 | CPU 均值 | CPU P95 | 观察到的 CPU 最大值 | RSS 范围 / MiB |
| --- | ---: | ---: | ---: | ---: |
| 导航主进程 | 8.94% | 14.00% | 19.99% | 66.94–70.40 |
| 独立规划 worker | 2.66% | 5.00% | 18.00% | 30.64–45.17 |
| LiDAR 建图 | 31.33% | 37.99% | 62.03% | 64.68–68.18 |
| C++ costmap 支撑 | 1.67% | 9.01% | 10.00% | 35.30–36.11 |
| Gazebo 服务端 | 47.20% | 91.95% | 94.11% | 535.44–543.34 |
| 验收 harness | 6.04% | 17.00% | 19.00% | 81.09–81.85 |

导航主进程与规划 worker 在同一快照的 CPU 之和：均值 **11.60%**、P95 **18.00%**、观察最大值 **31.00%**。二者同刻 RSS 之和的观察峰值为 **115.18 MiB**。资源跟踪辅助进程另有约 10.88–11.00 MiB RSS；此辅助进程未计入上述二者之和。

这里的 CPU 包含进程内部 ROS 通信、回调与序列化等开销，不是纯算法函数计时。RSS 是常驻页快照，不是 PSS/USS；多个进程的 RSS 相加可能重复计算共享页。VM 为虚拟地址空间，原始 JSONL/汇总 JSON 均保留该字段，不将其解释为实际物理内存用量。上述最大值是采样观察值，不是瞬时峰值、WCET 或容量保证。

## 采样开销及完整性

- 单次采样墙钟开销：均值 **1.679 ms**、P95 **2.283 ms**、最大 **6.059 ms**。包括 `/proc` 遍历、分类、快照序列化与 flush；另记录采样器启动发现耗时。
- 已记录采样开销合计占采样器运行墙钟的约 **0.168%**；这不是另行测量的采样器 CPU 利用率。
- 相邻快照间隔为 **0.954954–1.046505 s**，没有将实际间隔强行视作精确 1 s。
- 本次 `/proc` census 读取错误计数为 0。进程身份使用 PID 与 starttime ticks，避免将 PID 复用前后的 CPU 计数拼接；新出现或中断观察的进程不反推缺失 CPU。

## 原始旁路与复算

全部文件位于 WSL `/home/usv/usv_migration/results/`，保留原始内容，没有写入正在验收的源码/install：

| 文件 | 文件名 | SHA256 |
| --- | --- | --- |
| 原始 JSONL | `python-final-matrix-001-processes.jsonl` | `064db6955db29ba5fda01d48c908dff63cb6dcae6ad9c976bfbb17ec08b46749` |
| 实际汇总脚本 | `python-final-matrix-001-processes-summarize.py` | `5f38fcca964ebbdc0a27854c0c8face6bd89d4af52fdd3efe5cc6c2d0f15ff39` |
| 汇总结果 | `python-final-matrix-001-processes-summary.json` | 输入文件及逐 trial 事件 SHA256 已保存在 JSON 内 |

汇总 JSON 同时保存全部观察区间、普通航行窗口、故障窗口的各角色统计和逐 trial 覆盖分类；原始 cmd/role、PID/PPID、utime/stime、RSS/VM、单调时间和逐次采样开销保存在 JSONL。

在 WSL 复算时使用一个尚不存在的输出文件，保留第一次结果：

```bash
python3 /home/usv/usv_migration/results/python-final-matrix-001-processes-summarize.py \
  /home/usv/usv_migration/results/python-final-matrix-001-processes.jsonl \
  /home/usv/usv_migration/results/python-final-matrix-001 \
  /home/usv/usv_migration/results/python-final-matrix-001-processes-summary-recomputed.json
```

已检查采样器语法、唯一 harness 匹配、后代范围及规划 worker/资源跟踪器分类，并实际完成上述只读采样。没有用未采样时段的数据推算完整矩阵 CPU/RSS。
