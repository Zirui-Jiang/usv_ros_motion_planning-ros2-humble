# LQR Q1/R20 独立配置候选归档

本轮是三场景各一次的真实 Gazebo 候选诊断，未计入正式 3×3 矩阵。安装提交为 `1a82de47a21238b7c5b8e1c3794db5eb5ae7235a`；使用独立启动 YAML：Q=`[1,1,100,10,10,10]`、R=`[20,20]`、预瞄 2 s、最大迭代 5000、求解预算 80 ms。原始模型与 Riccati 求解形式未在本次试验中改变。当前源码的后续修改不属于这轮证据。

| 场景 | 导航结果 | 仿真耗时/s | 固定路径 RMSE/m | 固定路径最大横偏/m | 全艇最小扫掠间隙/m |
| --- | --- | ---: | ---: | ---: | ---: |
| 直航 | 通过 | 16.4 | 0.136565 | 0.352931 | 无障碍 |
| 转弯 | 通过 | 17.5 | 0.923808 | 2.046443 | 无障碍 |
| 单圆柱 | 通过 | 18.8 | 0.541927 | 0.946096 | 1.306592 |

到达位置门槛读取实际 LQR 参数 1.0 m，速度门槛为 0.25 m/s，持续观察 1 s 墙钟时间。manifest 中保留的场景默认 1.75 m 未作为通过门槛；每条 summary 的 `effective_arrival_distance_m` 保存实际门槛。圆柱半径 1 m，全艇采用 2.2×1.2 m 矩形及采样间扫掠界，最低允许间隙为 0.2 m。导航通过不能替代相对基线门槛、重复稳定性与工程跟踪质量评价；尤其本例转弯 RMSE 仍为 0.924 m。

整个采集窗口内，各状态携带的滚动控制耗时 P99 最大观测值为直航 33.265 ms、转弯 22.750 ms、圆柱 28.116 ms；最大执行时间依次为 33.413、39.145、30.693 ms。1 Hz `navigation_status` 中 TIMEOUT 观测均为 0，此处不将其称为逐控制步零超时。圆柱记录两次 SIMULATION_PAUSED 和一次 SETTLING，最终 GOAL_REACHED。每例八个 launch 进程均正常退出。

归档内容：

- `candidate_audit.json`：未经改写的候选审计结果。
- `summary.json` 与 `summaries/`：原始整轮摘要及三条试验摘要，逐字复制。
- `manifest.json`、`lqr_preview2_q1_r20.yaml`、`verified_parameters.json`：启动配置、原 YAML、运行中参数读回记录。YAML SHA-256 与 manifest 记录一致。
- `provenance_check.json`：运行前后 commit、Git 状态、差异与所有源码/安装/验收脚本条目完全一致；原始两份 provenance 的绝对路径和 SHA-256 同时保存。
- `exit_health.json`：与导航结果分开的退出日志检查。
- `source_index.json`：全部原始结果文件的绝对路径、字节数和 SHA-256，以及复制文件的来源映射。

完整 events、raw 地图、状态和日志保留在 WSL 的 `/home/usv/usv_migration/results/python-lqr-q1-r20-candidate-001`。前后 provenance 位于该目录同级，后缀为 `.provenance-before.json` 和 `.provenance-after.json`。本归档没有复制大体量事件流与地图，没有修改原始结果。
