# python-release-grid-001：正式矩阵 OS 旁路性能记录

本报告对应固定数值提交 `e38183218fce7044e686a0f697ed13f456f49039` 的正式运行，结果目录为 WSL `/home/usv/usv_migration/results/python-release-grid-001`。采样器每秒只读 `/proc`，区分导航主进程、独立规划 worker、建图、costmap、Gazebo 等角色。没有向被测进程发信号、附加调试器、启动其他仿真或写入被测 WSL 源码/install。

18 个导航窗口全部得到实际采样覆盖。功能判据和退出健康独立记录：执行代理确认 18 导航与 3 故障检查均通过，但 PID 圆柱第 1 次的 gzserver 退出 `-6`、第 3 次的 fake_localization 退出 `-11`；本报告不据 CPU/RSS 数据宣称全生命周期通过。其余退出细节由正式矩阵的原始 launch 日志和退出审计归档负责。

## 进程身份和实际覆盖

- 等待器于单调时间 `74803.657544437` READY，精确匹配本次 `--output`，每 0.2 s 查找唯一 harness。命中 PID `103328`，启动标识 `7484526` ticks；sampler/wrapper PID 为 `103284`。以上是本次历史身份，不可用于后续进程控制。
- 首样本为 `74845.341280677`，首个 `goal_published` 为 `74883.552537790`，采样提前 **38.211257 s**。harness 自身开始至首样本约 81 ms 没有采样，不反推这段 CPU；全部导航 goal 都在采样开始之后。
- 共 **699 个快照**，末样本为 `75543.342657851`，首末包络 **698.001377 s**。采样器总运行 **699.016883 s**，检测到原 harness 退出后自然结束，返回码 0；并非达到 1800 s 上限。
- PID/LQR 各直线、转弯、圆柱 3 次，共 **18/18 个 goal→acquisition_finished 窗口**均被包络涵盖。事件窗口合计 **384.998759 s**，内有 386 个快照。排除跨窗口边界的 CPU 差分后，各主要角色得到 **368 个完整区间**，约 368 s 观察暴露时间。
- goal_change 和 cancel 的标准事件窗口均被涵盖。odom_dropout 有实际 `/proc` 记录，但没有 `acquisition_finished`，因此不强行构造同名标准窗口，也不把它并入导航表；其 `trial_finished` 在末样本后约 0.542 s 才写入，不能声称该故障 trial 的整个清理尾部均有快照。

“窗口被涵盖”指事件起止处于首末样本包络内，不代表连续捕获了亚秒工作或进程首次观察前的 CPU。完整逐 trial 时间、覆盖分类及事件/summary 哈希见 [采样 receipt](python-release-grid-001-process-performance-receipt.json)。

## 航行窗口性能

下表仅合并 18 个普通导航窗口，排除启动、清理及故障试验。**100% CPU 表示占满一个逻辑核**，本机报告 12 个逻辑核。角色均值为实际 CPU 时间差之和除以对应进程观察区间之和；P95 是 1 Hz 区间利用率的线性插值分位数。包含 ROS 通信、回调和序列化等进程工作，不是纯算法函数计时。

| 角色 | CPU 均值 | CPU P95 | 观察 CPU 最大值 | RSS 范围 / MiB |
| --- | ---: | ---: | ---: | ---: |
| 导航主进程 | 7.36% | 15.02% | 22.03% | 66.37–70.19 |
| 独立规划 worker | 2.88% | 6.00% | 19.99% | 30.59–46.16 |
| LiDAR 建图 | 34.59% | 44.00% | 53.03% | 64.76–68.13 |
| C++ costmap 支撑 | 0.86% | 2.00% | 2.00% | 35.28–36.28 |
| Gazebo 服务端 | 44.27% | 54.00% | 63.04% | 525.47–534.84 |
| 验收 harness | 5.46% | 7.00% | 8.99% | 73.93–82.66 |

导航主进程与规划 worker 同刻 CPU 之和：快照均值 **10.24%**、P95 **19.02%**、观察最大值 **30.02%**；同刻 RSS 和的观察峰值 **114.00 MiB**。资源跟踪辅助进程另有约 10.88–11.00 MiB RSS，未计入上述二者之和。RSS 之和可能重复计算共享页，不是 PSS/USS；VM 保存在原始和汇总 JSON 中，不当作实际物理内存。

本表混合两个控制器与三种场景，仅描述这次固定矩阵的资源区间，不能据此证明单个算法相对基线的效率提升。1 Hz 无法捕获所有瞬时峰值或短命进程；最大值是观察值，不是 WCET 或容量保证。

## 采样开销与数据限制

- 单次采样开销均值 **1.963 ms**、P95 **2.734 ms**、最大 **5.897 ms**；累计 1.372448 s，约占采样器运行墙钟 **0.196%**。这包括 `/proc` 遍历、分类、序列化和 flush，不是另外测量的采样器 CPU 利用率。
- 实际相邻样本间隔 **0.983468–1.016553 s**，没有假定每次精确 1 s。UTC 可能受 WSL 时钟调整影响，持续时间全部使用单调时钟。
- census 共记录 **1 次读取错误**，发生于 sample 661 / `75506.342435955`；该快照仍含 harness、导航、worker、建图、costmap、Gazebo 等主要角色。脚本未记录失败 PID/错误类别，不能断言其来源或成因，也不补造其数据。
- PID 与 starttime ticks 共同识别进程。新进程首样本、观察中断或未捕获的进程，不从其他进程/其他试验推算缺失 CPU。

## 原始文件及复算

以下文件均保存在 `/home/usv/usv_migration/results/`，原始 JSONL 约 4.39 MB，没有复制进仓库；精简数值与全部来源哈希保存在上述 receipt。

| 文件 | SHA256 |
| --- | --- |
| `python-release-grid-001-processes.jsonl` | `0727fd3a311c9d903a3fe3b120f1b10b958c8389795aaed37d5ec8aeb965cdbc` |
| `python-release-grid-001-processes-summary.json` | `98e759ef27e7f6783ff4fc5be7d40817ec313b8444d9e9a89830b8cd0fa76068` |
| `python-release-grid-001-processes-summarize.py` | `5f38fcca964ebbdc0a27854c0c8face6bd89d4af52fdd3efe5cc6c2d0f15ff39` |
| `python-release-grid-001-sampler-wait.py` | `c620adcdb42a40445bf4c05c527e26fa4710219b92d37cb52270243b1579d3f3` |
| `python-release-grid-001-sampler-lifecycle.jsonl` | `47d721519562beda884e7019ec992acb73b8b0b76a8ddbcd05b2ffde7dcf34be` |
| 运行时 `scripts/validation/process_sampler.py` | `ac6a30158f2e51d4b54f9f030a543418b3d3a1d45c2e0187c0077b3fc59657dc` |

```bash
python3 /home/usv/usv_migration/results/python-release-grid-001-processes-summarize.py \
  /home/usv/usv_migration/results/python-release-grid-001-processes.jsonl \
  /home/usv/usv_migration/results/python-release-grid-001 \
  /home/usv/usv_migration/results/python-release-grid-001-processes-summary-recomputed.json
```

复算要求新的输出路径，拒绝覆盖已有报告。全观察区间、普通导航和标准故障窗口分别汇总，逐进程 cmd/role、PID/PPID、utime/stime、RSS/VM 与采样开销保留在原始 JSONL。本报告不改变旧失败记录或 M5 清理门槛。
