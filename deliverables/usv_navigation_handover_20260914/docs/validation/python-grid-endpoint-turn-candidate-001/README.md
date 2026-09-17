# 栅格末点兼容修复：PID 转弯一次候选

固定安装 `3afb25f84232e560c62599a721dc0bfbe1a21657`，默认 PID lookahead_time=2.6 s、LOS lookahead_distance=1 m / switch_dist=5 m、obstacle_factor=0.25。没有配置覆盖或在线调参。本次是运行前声明的五条候选中的转弯重复 01，功能、源时间配对和节点窗口性能检查均通过。

| 到达时间/s（仿真） | legacy RMSE/m | 源时间 RMSE/m | C++ 源时间 RMSE/m | 源时间比例 | 到达时间比例 | 全局搜索次数 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 20.5 | 0.069297403 | 0.073302248 | 0.094218160 | 0.778005512 | 0.907079646 | 1 |

配对基线为 baseline-grid-interior-001 的 PID turn 01；legacy 比例 0.650671641 亦通过。源时间采用预先固定的 source_time_trapezoid / otter_fake_localization 契约，覆盖 valid，最大源间隔约 0.1 s、首端约零、末端不超过 0.1 s；覆盖上限 0.5 s 和比例上限 1.10 保持不变。原始独立计算见 [候选比较报告](../grid-endpoint-candidate-comparison-001/README.md) 和 [report.json](../grid-endpoint-candidate-comparison-001/report.json)，后者 SHA256 `b0366b4f253bdc36f1d1f6d2cdbd95e35974e856b2a1357f4f4d79f08c6fd453`。

8 个 launch 进程均 clean 退出。采集窗口有 2 次 SIMULATION_PAUSED 状态观察，没有记录到 NO_PATH 或故障快照，暂停期间原始样本未删除。约 1 Hz 状态话题不保证捕获全部亚秒级变化。

完整原始数据保留在 `/home/usv/usv_migration/results/python-grid-endpoint-turn-candidate-001`。归档中的 summary/manifest 为原样副本；状态、参数读回和退出审计见 [candidate_audit.json](candidate_audit.json)、[verified_parameters.json](verified_parameters.json)。所有原件路径、字节数和 SHA256 见 [source_index.json](source_index.json)，前后指纹逐字段一致的结果见 [provenance_check.json](provenance_check.json)；源/安装/验收文件数量分别为 513/680/88。

场景 SHA256 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`，艇体 2.2×1.2 m、0.2 m 裕度、4 m 横向误差门槛及其他原协议阈值不变。候选不含 OS 旁路采样，不代表完整正式矩阵通过。
