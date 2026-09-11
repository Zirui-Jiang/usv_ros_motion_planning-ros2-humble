# 栅格末点兼容修复：PID 单圆柱一次候选

本次固定安装 `3afb25f84232e560c62599a721dc0bfbe1a21657`，默认 PID lookahead_time=2.6 s、LOS lookahead_distance=1 m / switch_dist=5 m、obstacle_factor=0.25。没有配置覆盖或在线调参。本次是预先声明的五条候选中的 single_obstacle 重复 01，功能、源时间配对和节点窗口性能均通过。

| 到达时间/s（仿真） | legacy RMSE/m | 源时间 RMSE/m | 源时间/C++ | 到达时间/C++ | 全艇扫掠最小间隙/m | 全局搜索次数 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 22.5 | 0.191174856 | 0.197981216 | 0.405596452 | 0.740131579 | 0.688245188 | 5 |

固定配对为 baseline-grid-interior-001 的 PID single_obstacle 01，其源时间 RMSE 为 0.488123639 m；legacy 比例为 0.403832333。覆盖 valid，最大源间隔约 0.1 s、首端约零、末端不超过 0.1 s。采用预声明 source_time_trapezoid 和 otter_fake_localization 契约，0.5 s 覆盖上限、1.10 比例上限保持不变。原件见 [候选比较报告](../grid-endpoint-candidate-comparison-001/README.md) 和 [report.json](../grid-endpoint-candidate-comparison-001/report.json)，后者 SHA256 `b0366b4f253bdc36f1d1f6d2cdbd95e35974e856b2a1357f4f4d79f08c6fd453`。

半径 1 m 的明确生成圆柱位于 (29.125, -10.125)，艇体为 2.2×1.2 m 矩形。间隙使用艇体扫掠与圆柱几何的既定保守检测方法，超过固定 0.2 m 裕度；不是仅以船中心距离判碰撞。8 个 launch 进程均 clean。状态采集有 2 次 SIMULATION_PAUSED，没有记录到 NO_PATH 或故障快照，全部原始样本保留。约 1 Hz 状态话题不能证明不存在未采到的亚秒级状态变化。

完整原始目录 `/home/usv/usv_migration/results/python-grid-endpoint-obstacle-candidate-001` 保留 events.jsonl、初始 raw costmap/state 与 launch 日志。本归档保存原样 summaries/manifest、安装参数及状态/参数时间线。绝对来源路径、字节数、SHA256 见 [source_index.json](source_index.json)，逐例结果/退出见 [candidate_audit.json](candidate_audit.json)。前后 provenance 的 commit/status/diff/artifacts 等逐字段相同，源/安装/验收文件数为 513/680/88，核验见 [provenance_check.json](provenance_check.json)。

场景 SHA256 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`，4 m 横向误差上限及全部原有停止规则未改变。本候选未采 OS 旁路数据，不能替代正式 18+3 的覆盖和完整退出健康审核。
