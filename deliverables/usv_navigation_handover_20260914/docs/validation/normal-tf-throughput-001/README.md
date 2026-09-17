# 正常 TF 条件下的 30 秒建图吞吐补充验证

这是一项**源节点受控实验**，条件是每条扫描发布前，实际 `MappingNode.tf_buffer` 已能查询该 frame/stamp 的 TF。它补充正常 TF 可用时的吞吐证据，不替代正式 `python-release-grid-001/python_lqr_straight_01` 的 `unknown`，也不表示 Gazebo/定位链路在所有负载下能及时提供 TF。

## 条件和测量方法

使用独立 domain 189，不启动 Gazebo，不改变 WSL 正式源码/install。控制代理释放仿真后执行，主任务及其他代理在测量期间暂停 CPU 密集测试、编译和绘图。默认地图保持 1024×1024、0.5 m，建图发布 2 Hz、队列容量 5、TF 等待期限 0.2 s，使用 Python 内核。

在 30 s 单调墙钟计划中发布 450 条独立扫描，15 Hz、每条 2000 束、`range_max=25 m`。全部束采用确定性 24 m 命中，构成可复现的长射线合成负载；它使用真实传感器的束数/频率/量程规格，但不是某个实际场景的录包，也不能代表所有可能扫描几何的最坏执行时间。

TF 经 ROS `TransformBroadcaster` 发到实际 listener；driver 在每次扫描发布之前调用该 mapper buffer 的 `can_transform` 核对对应的精确时间。扫描经 ROS 话题交给原 `_on_scan` 与 `_drain`，不直接调用积分函数，不改变队列/TF 截止逻辑。

测试子类仅在原回调返回后复制测量值：每次成功积分立即保存 processing/queue timing，逐次保存原 after_drain 的同一检查点及计数。**全过程 P95 使用全部 450 条原始计时，不使用原节点 deque 的最后 256 条代替。** 原 5 秒滚动观测仍按冻结的 `compare_results.backlog_performance` 评估；两种统计各自标明范围。

## 原始尝试 001：保留为失败

`/home/usv/usv_migration/results/normal-tf-throughput-001` 使用系统 ROS 时钟。WSL 校时期间时钟实际回退，节点按设计清队列并重置地图：最终 accepted=450、processed=449、integrated=449、resets=1，expired/overflow=0，末队列=0。等待全部 450 积分超时，脚本返回 1。

这不是 450 条成功的正常 TF 吞吐实验；不能删除漏掉的一条、将剩余 449 条称作完整 450 条，或把系统时钟回退解释成算法处理超时。[001 摘要](001-summary.json) 与完整原始事件保留。

## 受控时钟尝试 002：通过

仅调整测试 driver：`use_sim_time=true`，用单调墙钟生成连续 `/clock`，隔离 WSL 系统时钟校正。扫描仍按真实墙钟 15 Hz 发送，地图按该同步 ROS 时钟 2 Hz 发布；生产算法、容量和截止阈值不变。

| 指标 | 实际结果 |
| --- | ---: |
| 计划扫描 / 接收 accepted / 成功积分 | 450 / 450 / 450 |
| 逐条 TF 预可查 | 450 / 450 |
| 成功计时条数 / 不同扫描 stamp 数 | 450 / 450 |
| 处理耗时 P50 / P95 / P99 / 最大值 | 21.267 / 23.288 / 26.260 / 30.756 ms |
| 队列等待 P50 / P95 / 最大值 | 5.326 / 10.217 / 10.740 ms |
| 发布计划迟到 P95 / 最大值 | 1.411 / 1.996 ms |
| 实际发布间隔 P50 / P95 / 最大值 | 66.881 / 67.287 / 68.395 ms |
| 30 秒内地图发布数 | 60 |
| expired / overflow / invalid / reset | 0 / 0 / 0 / 0 |
| 稳态 after_drain 队列最大值 / 最终队列 | 0 / 0 |
| 冻结 backlog 判据 | pass：6 个状态快照、5 个完整区间 |

处理 P95=23.288 ms 满足预定 **66.7 ms** 上限。所有处理尝试均成功，没有 `WAITING`。after_drain 的 0 仅指处理完成后的观测，不宣称扫描从未入队；逐扫描等待时间明确保留。

450 条全部完成且队列清空后，才另发扫描 0、224、449 的重复 stamp。重复阶段把束距离改为 0.5 m，使错误二次积分可改变地图；实际 duplicate 增加 3，积分仍为 450，地图 SHA 不变：`9c68f6e28386005e7f5efc4553338da4c338c5aa77ae13a4b62124e7daa826d1`。重复检测没有挤占稳态 15 Hz 输入计划。

源组件、基准脚本与冻结判据文件的前后 SHA 一致，运行返回 0。源 HEAD 记录为 `877743d25b7f88ce82b226b7f334de52778ac9d5`；本项按摘要中的逐组件 SHA 固定被测源文件，不将它称为最终干净安装验收。

## CPU、RSS 与适用边界

mapper 与 ROS driver、TF 发布/listener、观测子类和订阅在同一进程 PID 111426 内。因此 CPU/RSS 包含整个受控驱动及测量工作，不能当作生产 mapping 节点独占值，也不能与正式多进程采样表直接比较。

30.000939 s CPU 观察范围内，累计 CPU 为 12.668527 s，均值 **42.227% 单核**。30 个 1 Hz 进程样本的 RSS 为 **64.46–72.74 MiB**，VM 观察最大值 318.75 MiB。RSS 是快照及进程高水位记录，不是 PSS/USS；没有做纯算法 CPU 分离或 WCET 保证。原事件保存各次进程 CPU 累计量、RSS/VM、单调时间与全部扫描/队列记录。

## 文件、复算与复现

- [002 摘要](002-summary.json)：包括全部 450 条统计、条件检查、冻结 backlog 详细结果、源码 SHA 与逐项 pass。
- [002 全部 450 条扫描计时 CSV](002-all-450-scan-timings.csv)：含每条 index/stamp、TF 预验证、发布、接收时间和 processing/queue timing，可独立重算分位数。
- 原始 WSL `normal-tf-throughput-001/events.jsonl` SHA：`5cdbdc8f024fd8ec0ee0889c9e6746a084204b8c3c902729004d7a7108d2f597`。
- 原始 WSL `normal-tf-throughput-002/events.jsonl` SHA：`75336d260ede62c3fc94fc73f33f9582ef5a7b1e114a9a5d4364ef68fdc97dae`。
- 001/002 原始 summary SHA 分别为 `14d685eab4a175dfbcc0069f1abffba5e5690abbe1f49d91283fb97e70616192`、`3a80286afb3410d8dd874279fcfedb552d6f723430e2c699543d7d2420182e5a`。
- 002 已执行脚本保存为 `/home/usv/usv_migration/results/normal-tf-throughput-002/executed-script.py`，复制时验证与运行前/后的 SHA 同为 `012609063f7ee3a5128dada9c4e4293750f0ea85db1d9f5c9b7bf89cfbe3f1fd`。

本项不加入常规全仓 pytest 的耗时流程。未来在没有其他性能负载时复现，使用新的输出目录：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/mapping_throughput.py' \
  --source-repo '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble' \
  --output /home/usv/usv_migration/results/normal-tf-throughput-recheck-001 \
  --domain-id 189
```

原正式 LQR straight01 的连续 TF 过期与 `unknown` 保持原样。受控正常 TF 实验证明的是上述已知条件下可达的吞吐，不能回写历史 live 缺失证据。
