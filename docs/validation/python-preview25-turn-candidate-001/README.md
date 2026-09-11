# 固定 PID 2.5 s 预瞄候选：turn

固定源码/安装 e381832，独立完整 controller YAML 仅将 PID lookahead_time 从 2.6 s 改为 2.5 s。统一加载器校验全部 84 项，其他参数不变，实际节点读回与配置 SHA 一致。原格内场景、0.2 m 全艇裕度、4 m 横向边界和到达门槛保持；无在线调参、无择优重试、无附加 OS 采样。

| 试验 | 仿真用时/s | RMSE/m | 最小全艇间隙/m | 搜索次数 | 原始配对RMSE比值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| python_pid_turn_01 | 19.9 | 0.100484 | 无障碍 | 1 | 0.943498 |

上述结果是预先声明候选的原始记录，功能到达与固定 10% 比较分别判断；比值超过 1.10 的证据保留，不提升为默认配置。运行前后源码、安装与验收脚本指纹一致。完整状态和非正常状态见 candidate_audit.json/status_timeline.json；状态为 1 Hz 采样，不能据此断言任一控制步均无瞬态。退出日志按实际 clean 标记与异常分开保存。

配置 SHA-256：`b9f4e2431f367a24313a2e90eb3b1984da80cbb9edb0824a9f29b06e86445eda`。7 份原始文件复制并逐字核对 SHA-256。完整原始目录：`/home/usv/usv_migration/results/python-preview25-turn-candidate-001`；原始事件/地图/轨迹保持 WSL 原样，source_index.json 索引绝对路径、字节数和 SHA。

本例八进程正常退出，配对 RMSE 比值 0.943498；采集窗口记录一条 SIMULATION_PAUSED，原样保留。一次转弯候选不等同于完整重复矩阵验证。
