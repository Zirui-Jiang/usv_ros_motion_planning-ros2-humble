# python-grid-endpoint-final-matrix-001：正式矩阵 OS 旁路性能记录

本报告对应固定数值提交 `3afb25f84232e560c62599a721dc0bfbe1a21657`，运行目录为 WSL `/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001`。采样器每秒只读 `/proc`，按进程树区分导航、规划 worker、建图、costmap、Gazebo 等角色；没有发送信号、附加调试器、启动其他仿真或改写被测 WSL 源码/install。中途会话中断后恢复的是原有采样进程，没有重启或填补数据。

18 个导航窗口全部得到实际采样覆盖。执行代理确认 18 导航与 3 故障检查均通过，harness 返回 0，运行前后源文件、安装和 Git 指纹一致；这不等于配对跟踪指标或退出健康全部通过。PID 转弯第 2 次的 gzserver（PID 114737）与 LQR 直航第 1 次的 gzserver（PID 115809）均在 SIGINT 后出现 boost self-join 错误并退出 `-6`；其余对应节点正常退出。20 个含 Gazebo 的 trial 中，18 个各有 8 个正常退出进程；所有定位与 joint_state_publisher 正常退出。独立 odom_dropout 没有节点直接返回码记录，退出健康保留 unknown。完整功能和退出审计见[正式矩阵归档](python-grid-endpoint-final-matrix-001/run_audit.json)。

## 进程身份、覆盖和中断恢复

- 等待器在单调时间 `79254.219277534` READY，每 0.2 s 查找与本次 `--output` 精确相同的唯一 harness。命中 PID `113855`、启动标识 `7948542` ticks；sampler/wrapper PID 为 `113740`、启动标识 `7925419` ticks。这些历史身份不能用于后续进程控制。
- 首样本单调时间 `79485.467206417`，首个 `goal_published` 为 `79496.181248689`，提前 **10.714042 s**。harness 开始至首样本约 47 ms 未被采样，不反推这段 CPU。
- 共 **623 个快照**，索引连续为 0–622；末样本 `80107.468300718`，首末包络 **622.001094 s**。采样器运行 **623.002800 s**，在 `80108.470005859` 检测到原 harness 退出或身份不再匹配后自然结束，原工具 session 7370 返回 0，没有达到 1800 s 上限。
- 会话恢复核查时刻为 `79981.433853788`，原 sampler 和 harness 的 PID/starttime 均保持一致且仍运行；已有 496 个完整样本，最新时间 `79980.468311881`。当时及最终结果均没有超过 2 s 的样本间隔。会话中断的准确时刻未被独立仪器记录，receipt 中保留 null；可以证明采样序列连续，不能虚构 UI 中断起止时间。
- PID/LQR 各直线、转弯、圆柱 3 次，**18/18 个 goal→acquisition_finished 窗口**均处于样本包络内。窗口合计 **381.927903 s**，包含 **382 个快照**；排除跨窗口边界的 CPU 差分后，各主要角色得到 **364 个完整区间**，约 364 s 进程观察暴露时间。
- goal_change、cancel 的标准事件窗口也被包络涵盖。odom_dropout 有实际 `/proc` 记录，但缺少 `acquisition_finished`，不为其构造标准导航窗口。其 `trial_finished` 为 `80107.543755817`，比末样本晚 **0.075455 s**，因此不声称该故障清理尾部完整覆盖。

窗口被包络涵盖不表示连续捕获了所有亚秒活动或首次观察前的 CPU。逐 trial 时间、事件/summary 哈希、恢复快照和数据限制见[采样 receipt](python-grid-endpoint-final-matrix-001-process-performance-receipt.json)。

## 普通航行窗口性能

以下仅合并 18 个普通导航窗口，排除启动、清理及故障试验。**100% CPU 表示一个逻辑核满载**，本机报告 12 个逻辑核。角色均值为实际 CPU 时间差之和除以对应进程观察区间之和；P95 为实际 1 Hz 区间利用率的线性插值分位数。进程工作包含 ROS 通信、回调与序列化，不是纯算法函数耗时。

| 角色 | CPU 均值 | CPU P95 | 观察 CPU 最大值 | RSS 观察峰值 / MiB |
| --- | ---: | ---: | ---: | ---: |
| 导航主进程 | 7.50% | 16.01% | 27.32% | 70.03 |
| 独立规划 worker | 2.96% | 6.00% | 27.99% | 46.18 |
| LiDAR 建图 | 34.54% | 45.01% | 65.03% | 68.23 |
| C++ costmap 支撑 | 0.86% | 2.00% | 2.00% | 36.02 |
| Gazebo 服务端 | 44.50% | 54.02% | 67.03% | 534.60 |
| 验收 harness | 5.34% | 7.00% | 8.98% | 82.18 |

导航主进程与规划 worker 同刻 CPU 之和：快照均值 **10.46%**、P95 **20.01%**、观察最大值 **42.98%**；同刻 RSS 和的观察峰值 **115.63 MiB**。资源跟踪辅助进程未计入二者之和。RSS 和可能重复计算共享页，不是 PSS/USS；VM 保留在原始与汇总 JSON 中，不作为实际物理内存。原始 worker RSS 最低快照为 7 MiB，包含启动过程，不当作稳态内存。

本表混合两个控制器和三类场景，只描述这次固定矩阵的资源区间，不能单独证明相对 C++ 基线的效率提升。1 Hz 可能漏掉短命进程与瞬时峰值；观察最大值不是 WCET 或容量保证，也不能替代节点内部的控制/扫描耗时与队列判据。

## 采样开销与缺失项

- 单次采样开销均值 **1.897 ms**、P95 **2.557 ms**、最大 **12.992 ms**；累计 **1.181920 s**，约占采样器运行墙钟 **0.190%**。这是 `/proc` 遍历、分类、序列化和 flush 的墙钟开销，不是另外测量的采样器 CPU 利用率。
- 实际相邻样本间隔 **0.937289–1.058597 s**，没有超过 2 s 的间隔；没有假定每次恰为 1 s。持续时间采用单调时钟，UTC 仅作定位，避免将 WSL 时钟调整误认为性能变化。
- census 记录 **3 次读取错误**，分别位于 sample 350 / `79835.469082824`、sample 503 / `79988.468303853`、sample 598 / `80083.468198442`。前者该快照只观察到 harness；中者仍有主要运行角色；后者观察到 harness、ros_launch、costmap、navigation 和 resource_tracker。完整角色表保留在 receipt。脚本没有记录失败 PID 或错误类别，不能断言错误成因，不对遗漏 CPU/RSS 作插值补齐。
- CPU 差分要求相同 PID 与 starttime 且相邻观察有效。新进程首样本、缺失进程或观察中断不会用其他 trial 的数据估计。本报告只统计实际覆盖时间。

## 原始文件与复算

以下文件均保存在 `/home/usv/usv_migration/results/`。原始 JSONL 为 3,850,741 字节，保留在 WSL；精简数值、所有来源哈希及逐 trial 覆盖存入仓库 receipt。

| 文件 | SHA256 |
| --- | --- |
| `python-grid-endpoint-final-matrix-001-processes.jsonl` | `96fd186528436548d1f137da8881a070e7b5ea9d316ba20923c0cdeac5bb22f5` |
| `python-grid-endpoint-final-matrix-001-processes-summary.json` | `9a3e3393413f084f8d899a41a443b15d8b2921ea8f795a9e84b24a96c108de94` |
| `python-grid-endpoint-final-matrix-001-processes-summarize.py` | `5f38fcca964ebbdc0a27854c0c8face6bd89d4af52fdd3efe5cc6c2d0f15ff39` |
| `python-grid-endpoint-final-matrix-001-sampler-wait.py` | `e47bfe25b7b7117b8149e602518d15995f6fe703ab9ddd6e748c2daa3e998541` |
| `python-grid-endpoint-final-matrix-001-sampler-lifecycle.jsonl` | `96533edbc2798c178bfcc9ec4c60ca5c302846df3b6da34dd42fb15e69478705` |
| `python-grid-endpoint-final-matrix-001-sampler-recovery-001.json` | `a0b5f548629a3d8bc0a0ccd8d45ce3aaeaac2256dd59bd8e1f98a6a51e23b6f2` |
| 运行时 `scripts/validation/process_sampler.py` | `ac6a30158f2e51d4b54f9f030a543418b3d3a1d45c2e0187c0077b3fc59657dc` |

```bash
python3 /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001-processes-summarize.py \
  /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001-processes.jsonl \
  /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001 \
  /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001-processes-summary-recomputed.json
```

复算要求新的输出路径，拒绝覆盖已有报告；全观察、普通导航和标准故障窗口分别汇总。原始 JSONL 保留 PID/PPID、cmd/role、starttime、utime/stime、RSS/VM、时间和采样开销。本报告不改变旧失败、正式配对判定或 M5 清理门槛。
