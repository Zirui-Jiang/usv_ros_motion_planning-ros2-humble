# 固定 PID 2.4 s 预瞄候选：straight

固定源码/安装 e381832，独立完整 controller YAML 仅将 PID lookahead_time 从 2.6 s 改为 2.4 s。统一加载器校验全部 84 项，其他参数不变，实际节点读回与配置 SHA 一致。原格内场景、0.2 m 全艇裕度、4 m 横向边界和到达门槛保持；无在线调参、无择优重试、无附加 OS 采样。

| 试验 | 仿真用时/s | RMSE/m | 最小全艇间隙/m | 搜索次数 | 原始配对RMSE比值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| python_pid_straight_01 | 20.0 | 0.056625 | 无障碍 | 1 | 1.028547 |
| python_pid_straight_02 | 19.8 | 0.057768 | 无障碍 | 1 | 0.896209 |
| python_pid_straight_03 | 19.9 | 0.056817 | 无障碍 | 1 | 0.946163 |

上述结果是预先声明候选的原始记录，功能到达与固定 10% 比较分别判断；比值超过 1.10 的证据保留，不提升为默认配置。运行前后源码、安装与验收脚本指纹一致。完整状态和非正常状态见 candidate_audit.json/status_timeline.json；状态为 1 Hz 采样，不能据此断言任一控制步均无瞬态。退出日志按实际 clean 标记与异常分开保存。

配置 SHA-256：`bd4828f65c1302bccecfb63ce0e83b2a58b92e738053e3dea871a74b9046cf59`。13 份原始文件复制并逐字核对 SHA-256。完整原始目录：`/home/usv/usv_migration/results/python-preview24-straight-candidate-001`；原始事件/地图/轨迹保持 WSL 原样，source_index.json 索引绝对路径、字节数和 SHA。

退出核对：重复 1 的 Gazebo 退出 -6，其余七进程正常；重复 2/3 各八进程正常退出。重复 3 记录一条初始 WAITING_REFERENCE，随后恢复。三个配对 RMSE 比值均在 1.10 内，但此组三例不能覆盖转弯取舍。
