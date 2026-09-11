# 栅格末点兼容修复：PID 直航三次候选

这三次为预先固定的候选试验，源码和安装均为 `3afb25f84232e560c62599a721dc0bfbe1a21657`。默认 PID lookahead_time=2.6 s、LOS lookahead_distance=1 m / switch_dist=5 m、obstacle_factor=0.25；没有 controller-config 文件或在线参数覆盖。栅格末点修复恢复引导路径的格子中心终点，实际任务仍根据请求目标的位置与速度判定完成。

三次均满足功能门槛，源时间配对和独立运行性能均为 3/3 通过；并列保留的 legacy_wall 指标为 2/3 通过。直航 02 的 legacy 比例 1.334264831 未被改写。本组不能替代 PID/LQR 全部正式矩阵。

| 重复 | 到达时间/s（仿真） | legacy RMSE/m | 源时间 RMSE/m | 源时间/同编号 C++ | 到达时间/C++ | 全局搜索次数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 01 | 20.9 | 0.057474177 | 0.059768396 | 1.057886152 | 0.889361702 | 1 |
| 02 | 21.0 | 0.086004322 | 0.061445813 | 1.096310161 | 1.009615385 | 1 |
| 03 | 20.6 | 0.065146097 | 0.059457250 | 1.035858763 | 0.887931034 | 1 |

源时间计算使用运行前固定的 source_time_trapezoid 和 otter_fake_localization frame 契约，比较基线为 baseline-grid-interior-001 相同重复编号。双方源记录的最大间隔约 0.1 s、首端缺口约为零、尾端不超过 0.1 s，均满足 0.5 s 覆盖门槛。RMSE 与到达时间的比例门槛仍为 1.10。独立计分原件见 [候选比较报告](../grid-endpoint-candidate-comparison-001/README.md) 和 [report.json](../grid-endpoint-candidate-comparison-001/report.json)；后者 SHA256 为 `b0366b4f253bdc36f1d1f6d2cdbd95e35974e856b2a1357f4f4d79f08c6fd453`。

退出健康单独记录：01 的 launch.log 第 70 行收到 SIGINT、第 75 行向 joint_state_publisher 发 SIGINT，第 92 行记录 PID 112551 退出 -11；其他 7 个进程 clean。该时序不能证明具体崩溃原因。02、03 各 8 个进程 clean。01 完整日志保存在 [launch.log](python_pid_straight_01/launch.log)，SHA256 `74ec72f657edb9f09beab1600af7965c3ba1f52e74f84f0780746a3cda6bc52a`。02/03 各有 4/2 次 SIMULATION_PAUSED 状态观察，原始样本均保留；没有记录到 NO_PATH 或故障快照。状态话题约 1 Hz，不能据此排除未采到的亚秒级状态变化。

原始目录为 `/home/usv/usv_migration/results/python-grid-endpoint-straight-candidate-001`。完整 events.jsonl、初始地图/状态和日志保留在该目录；本地归档包含原样 summaries、manifest、参数读回、安装参数、状态时间线以及异常退出原日志。逐文件绝对路径、字节数、SHA256 见 [source_index.json](source_index.json)，运行汇总见 [candidate_audit.json](candidate_audit.json)。

前后 provenance 的 path/commit/status/diff/artifacts 逐字段完全相同，源文件 513、安装文件 680、验收脚本文件 88，详见 [provenance_check.json](provenance_check.json)。13 包构建和安装审计原件一并保存。场景 SHA256 为 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`，艇体 2.2×1.2 m、0.2 m 安全裕度、4 m 横向偏差上限及原有停止规则均未改变。本组没有 OS 旁路采样，性能结论限于已记录的节点窗口统计。
