# 固定 PID 3.0 s 预瞄：已完成转弯三次候选

本组按采集前提交的 [PID 3.0 s 预瞄协议](../pid-preview30-protocol.md)，在固定 `3afb25f84232e560c62599a721dc0bfbe1a21657` WSL 源码/普通安装上完成转弯 01/02/03。**三次功能、有效逐次源时间配对和节点窗口性能均通过。**在直航阶段启动之前，用户明确最终验收采用同场景三次平均 RMSE/平均时间并保留逐次和最差值，要求停止为逐编号失败继续调整默认参数。因此直航三次、圆柱三次未运行，结果目录未创建；停止原因不是本组三次转弯失败，未运行项也没有记为通过。

本候选独立保留其原逐次协议和原始报告，不混入正式 3afb25f 的 18+3 数据，也不提升为生产默认配置。

## 唯一变量与实际配置

外部完整文件为 `/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/config/pid_preview30.yaml`，SHA256 `98f941e18bd683fcf2d1cb58f19d1ec8dd1c1523507dd876494915ed7a4f3b13`。它从原安装 YAML 重新复制，唯一差异为 `usv_simulation.controller.pid_controller.lookahead_time: 2.6 → 3.0`；全部六个增益不变，特别是 k_d_course=0.0，没有继承此前失败的微分增益候选。完整加载器验证 84 项；解析后差异、文本 diff 和来源 SHA 见 [config_audit.json](config_audit.json)，逐次读回见 [verified_parameters.json](verified_parameters.json)。

格内场景 SHA256 为 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。factor=0.25、LOS 距离 1 m / 切换 5 m / 单步推进、全部其他算法配置、仿真插件和过期策略不变。保持 PID 实际位置容差 1.5 m、速度 0.25 m/s、艇体 2.2×1.2 m、圆柱裕度 0.2 m、横向偏差上限 4 m及原自动停止规则，没有在线调参。

## 保留的逐次结果

| 重复 | 仿真时间/s | legacy RMSE/m | 源时间 RMSE/m | 源时间/同编号 C++ | 时间/C++ | 原逐次配对 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| turn 01 | 20.6 | 0.086416373 | 0.088281297 | 0.936988124 | 0.911504425 | pass |
| turn 02 | 20.5 | 0.104934518 | 0.089335351 | 0.773615624 | 0.887445887 | pass |
| turn 03 | 20.5 | 0.082248820 | 0.077960892 | 0.986710355 | 0.876068376 | pass |

各执行一次全局搜索。比较仅使用 baseline-grid-interior-001 的 CPP PID turn 同编号，source_time_trapezoid / otter_fake_localization 契约、源覆盖上限 0.5 s 和比例门槛 1.10 均未改变。三对源覆盖 valid；legacy 比例为 0.811411115 / 0.858173488 / 0.971719652，亦为三对通过，原始样本未删除。

独立原样报告见 [comparison/report.json](comparison/report.json)，SHA256 `6814453ddd12b24b1d490126d7e45bac13073bdc7e50215de0dbe63c89641ad8`。完整报告保留基线的其他六对未运行 Python 候选，根汇总为 9 对、3 可比、3 通过、6 不可比、all_passed=false。这一 false 表示完整范围尚未覆盖，不能改写成全九条或正式 18+3 通过，也不能将本组三条标为失败。

## 推力、暂停和退出证据

三次从首个非零 sample 开始的一秒窗口内，right−left 均未出现正值，实际转向朝负 yaw 发展，没有观察到初始反向。原始事件行号、每轴命令接收事件和实际 yaw/r 见 [initial_direction_audit.json](initial_direction_audit.json)。异步回调的 sample/接收间隔不等于内部控制 dt，因此该观察不构成对任意时间步的证明。

turn 01 未观察到暂停状态；turn 02/03 分别有 4/2 条 SIMULATION_PAUSED 状态，全部保留。没有观察到 NO_PATH、CONTROL_ERROR 或 control_result_* 错误计数。状态话题约 1 Hz，不能排除亚秒级变化。首个非零命令后的零命令 sample 数为 26/64/45，包含仿真暂停与正常目标收尾，不能解释为同等数量的异常停车事件；完整位置/推力/状态仍在原始 events.jsonl。

三条 control/mapping/backlog 节点窗口检查均 pass，细节见原比较报告和 [mapping_status_timeline.json](mapping_status_timeline.json)。本组没有 OS 完整旁路采样，不替代正式资源验收。全部 24 个 launch 进程 clean，harness session 33615 自然退出码 0，未触发几何/横向边界停止，结束后没有自有仿真进程残留。

## 原始归档和后续状态

完整 WSL 原始目录为 `/home/usv/usv_migration/results/python-pid-preview30-turn-candidate-001`。before/after 的 path、commit、status、diff、artifacts 逐字段相同，源/安装/验收文件数量为 513/680/88；外部候选 SHA 前后不变。安装构建证据、原/候选 YAML、配置差异、三条原始摘要及完整比较报告共 13 份副本逐文件校验 SHA。位置、参考路径、推力和 raw map 原件保持 WSL 原样；绝对路径、字节数和 SHA 由 [source_index.json](source_index.json) 索引。

[run_audit.json](run_audit.json) 记录已完成三条、未运行六条及用户澄清后的停止原因。[provenance_check.json](provenance_check.json) 保存前后完整指纹的来源 SHA。当前默认参数仍为原 3afb25f；此前正式逐次失败、微分候选失败、预瞄候选结果均不追改。后续全新安装 smoke 依另行固定提交执行，不复用本候选编号或覆盖数据。
