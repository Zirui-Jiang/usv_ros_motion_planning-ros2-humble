# ScanQueue 200 ms 边界：只读审查与可区分证据

日期：2026-09-11。本文保留第一阶段对原 `python-final-matrix-001/python_pid_turn_03` 的只读诊断，并在末节记录随后授权实施的最小修复与实际回归。以下原始证据和代码行号对应修复前快照；原 200 ms 阈值及历史 unknown 判定不变，没有在正式矩阵中注入 TF 故障。

## 实际记录

原始事件文件：`/home/usv/usv_migration/results/python-final-matrix-001/python_pid_turn_03/events.jsonl`。收到相关 mapping_status 的单调时间为 `70348.258207891 s`；其中 5 秒 after_drain 窗口报告：

| 字段 | 原始值 |
| --- | ---: |
| oldest_pending_age_ms.max | 200.04234999942128 ms |
| 超出 200 ms 的差值 | 42.34999942128 μs |
| queue_ms.max，已完成扫描等待时间 | 103.04887399252038 ms |
| observation_count / pending_observations / empty_observations | 434 / 212 / 222 |
| max_queue_size / oldest_pending_distinct_count | 2 / 36 |
| accepted / processed / expired / overflow | 52 / 50 / 1 / 0 |
| max_observation_gap_ms | 53.97585598984733 ms |

该记录证明某次 after_drain 观测仍看到了年龄超过 200 ms 的 pending 扫描；它没有证明该扫描随后被积分。该状态中保存的已完成扫描等待最大值约为 103.05 ms，不能将 pending 最大值当作完成积分时的等待时间。过期计数为 1，但缺少最大年龄对应的扫描身份及其时序，不能将两者强行关联。

单调时间约 70348 s 处，Python float 的一个 ULP 为 `1.4551915228366852e-11 s`，远小于 42.35 μs。因此不能把该差值归为普通浮点舍入误差，也不能静默四舍五入到阈值以内。

## 现有调用顺序

- `src/core/perception/perception/scan_queue.py:71–78`：调用 process 前读取单调时间并检查过期；process 返回 False 时直接 break，没有再次读取时间。
- `src/core/perception/perception/mapping_node.py:121–126`：TF 查询设置 timeout=0，查询异常返回 False。非阻塞查询仍有 Python/C++ 调用、异常处理和调度耗时，timeout=0 不表示调用耗时为零。
- `mapping_node.py:115–116`：drain 返回后重新读取单调时间，再观察 pending 队列。
- `scan_queue.py:94–97`：pending age 用这次观测时间减去接收时间，数值没有裁剪到 200 ms；观测函数不会代替实际 drain 删除扫描。
- 成功路径在 `mapping_node.py:127–138` 取得 TF 后直接积分；虽然记录了新的 start 时间，仍没有在地图修改前用该时间复查 TF 等待期限。
- 队列轮询周期为 10 ms（`mapping_node.py:79–80`）。当前检查结果只在具体执行时点成立，不能由周期定时器推出任意墙钟瞬间的严格队列驻留上界。

记接收、首次检查、TF 返回、after_drain 观测时刻分别为 r、c、f、o，则：

`pending_age_at_observation = (c - r) + (f - c) + (o - f)`。

因此 `c - r < 200 ms` 与 `o - r > 200 ms` 可以同时成立。原记录没有 c、f 及最大年龄对应的 o/身份，不能区分“TF 查询期间跨过期限”与“返回后到观测之间跨过期限”。这里的 after_drain 数值是实际观测，不应以“观测误差”为由抹掉。

## 已执行的合成时间诊断

诊断直接调用真实 ScanQueue，使用可控单调时钟和无 ROS 的回调；不是真实 TF/Gazebo 重现，也不用于估计原实验中某条路径的发生概率。

| 合成输入路径 | 首次检查 | 回调返回 | 观测时间 | 实际 ScanQueue 结果 |
| --- | ---: | ---: | ---: | --- |
| TF 未就绪，耗时发生在回调内 | 199.990 ms | 200.04235 ms | 200.04235 ms | processed=0、expired=0，pending 最大年龄 200.04235 ms；下一轮在 210 ms 检查时 expired=1 |
| TF 未就绪，仅观测前出现延迟 | 199.990 ms | 199.990 ms | 200.04235 ms | 与上一行相同 |
| TF 成功，但回调内跨过期限后模拟积分 | 199.990 ms | 200.04235 ms | 200.04235 ms | processed=1、expired=0；模拟地图修改发生在 200.04235 ms |

前两例证明同一 pending-age 输出可以来自不同的时间分配，现有摘要不能区分。第三例证明当前回调契约允许成功路径在首次检查后跨界，属于独立可达的代码边界缺口；它并不证明原 200.04235 ms 样本走了成功积分路径。

可复查的已执行诊断材料：

- `/home/usv/usv_migration/results/scan-queue-deadline-diagnosis-001.py`
- `/home/usv/usv_migration/results/scan-queue-deadline-diagnosis-001.json`

JSON 保存诊断脚本、三个受审阅源文件与原事件的 SHA256，完整实际状态及三例合成输出。脚本只创建新结果文件；原结果仍在时会拒绝覆盖。

## 第一阶段提出的改进建议

1. **明确并落实积分准入期限。** 在 TF 查询返回后、任何地图修改前，再用新单调时间检查 `now - received_at >= 0.2`。超期扫描必须不积分，且仅计一次 expired。当前 bool 回调把“等待”和“消费完成”压在两个值里；可拆分 TF 获取与积分，或显式区分 WAITING / PROCESSED / EXPIRED，避免将超期消费误计为 processed。
2. **失败路径也复查。** 当 TF 查询返回未就绪时，在 break 前复查期限；若已经到期则丢弃并继续检查下一条，否则保持等待。保留逐束建图行为、容量和正常成功扫描的证据规则。使用已有确定性时钟模式补充“查询失败期间跨界”“查询成功期间跨界”“恰好到达期限”测试。
3. **把决策时间与观测时间同时保留。** 记录最后期限检查时间、TF 查询起止时间、积分准入时间、after_drain 观测时间，并为窗口最大年龄保存对应时间与扫描身份；分别计算决策年龄和观测延迟。不要在指标层删扫描、裁剪年龄、增加隐含 epsilon，或把观测前的旧时间伪标成 after_drain 时间。

即使补上复查，在普通 Linux/Python 调度下，检查后的数十微秒内仍可能被抢占；应准确陈述“某个单调时间点的准入/过期决定”，不能声称任意瞬间都没有超期驻留。现行 `compare_results.py:123–124` 对 pending 最大年龄以及 `149–150` 对完成等待时间的严格检查继续保留，当前样本仍为 **unknown**。若未来要调整验收语义，必须明示指标定义和证据，不能仅为消除 42 μs 超界放宽现有阈值。

## 第二阶段：已实施的最小修复与回归

收到执行授权后，仅修改建图队列、ROS 薄节点及其相关测试；没有调整容量 5、期限 0.2 s、静态 TF 行为、逐束证据规则或历史结果。

- 增加明确的 ScanResult：WAITING、PROCESSED、EXPIRED、INVALID；保留旧 bool 回调约定以兼容既有纯队列调用。
- TF 查询失败返回后再次检查时间，跨界扫描在当前 drain 中记 expired 并移除；成功取得 TF 后，在任何地图修改之前以新单调时间判断是否允许积分。
- MappingNode 在 drain 后读取一次 checkpoint，以它实际剔除过期项，再使用同一个 checkpoint 做 after_drain 观测。没有裁剪年龄、回填旧时间或在观测函数内部隐藏删除操作。
- 无效但已被消费的扫描使用 invalid_consumed 单独计数；processed 现在只表示实际成功积分。对于没有 reset 丢弃的区间，平衡式为 `accepted = processed + invalid_consumed + expired + overflow + pending`，同时 `processed = integrated_scans`、`invalid_consumed = invalid_scans`。invalid 仍表示未被 accepted 的无效输入。历史 schema 的兼容处理由验收器维护者单独实现。

先运行新增回归验证旧实现：**7 failed、14 passed、0 skipped**，包括成功 TF 在 200 ms/200.04235 ms 返回后仍积分、失败跨界仍滞留及无效输入误计 processed。修复后运行整个 perception 测试目录：**41 passed、0 failed、0 skipped**。

绿色回归实际执行了 `test_exact_tf_duplicate_expiry_clock_rollback_and_map_publication`，包括真实 rclpy 发布/订阅、准确时刻 TF、重复扫描、过期扫描、clock 回退后静态 TF 保留与地图发布。使用隔离 `ROS_DOMAIN_ID=185`、`ROS_LOCALHOST_ONLY=1`，直接导入 Windows 源目录；没有启动 Gazebo，也没有修改旧 WSL install。

原始日志和 JUnit：`/home/usv/usv_migration/results/mapping-deadline-fix-001/{red,green}.{log,xml}`。代码、测试与日志 SHA256 见同目录文档 `mapping-deadline-fix-receipt.json`。运行命令为：

```bash
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH='/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/perception:'"${PYTHONPATH:-}"
cd '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble'
python3 -m pytest -q -p no:cacheprovider src/core/perception/test
```

第一阶段合成诊断的输出对应修复前源码；复现旧失败应使用其 JSON 中记录的源码 SHA256 对应版本，不能直接用新实现重跑旧断言。旧真实 200.04235 ms 记录仍保持 unknown，本次绿色回归不回写旧实验结论。
