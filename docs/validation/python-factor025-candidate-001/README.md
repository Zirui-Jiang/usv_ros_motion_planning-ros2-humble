# factor=0.25 三次候选：首例持续软起点阻断后暂停

固定安装提交为 `79a109fc9e483fe01c7b1682d170443b3e794859`。干净 detached 检出后按原 sequential/Release 方式构建，13 包全部通过；安装一致性审计通过，实际安装默认与节点参数读回均为 `obstacle_factor=0.25`。PID 预瞄 2.6 s、原增益、LOS 距离 1 m/切换 5 m/单步推进保持不变。格内场景、艇体与 0.2 m 安全裕度均未修改，也未在线调参。

计划三次圆柱候选，实际第一例未到达，在持续 NO_PATH 后被监督器中断；第二、三例未运行。原始摘要为 completed=0、passed=0、run_finished=false、interrupted=true、safety_stopped=false，不能写成完成三次或到达通过。

第一例观察区间的最小全艇扫掠间隙为 **0.721369 m**，观察窗口固定路径 RMSE 为 **0.174135 m**，最后样本为 sim=83.5 s、距离目标 5.378507 m。RMSE 包含停止等待区间，仅用于中断诊断，不是完整试验性能指标，不能直接参与完成试验配对。状态中记录 65 条 NO_PATH，原因均为 `start or goal is blocked`；15.3 s 首次出现，22.7 s 短暂恢复，23.7 s 后再持续拒绝，最终未到达。

两份真实回调快照均显示起点 raw=75、目标 raw=0，而 factor=0.25 的全局阈值为 254×0.25=63.5；对应姿态的全艇 hard≥254 检查通过。

| 快照后缀 | map 时标/s | state 时标/s | 起点 raw | 最后非空轨迹时标/s |
| --- | ---: | ---: | ---: | ---: |
| c0a7f065 | 14.9 | 15.3 | 75 | 14.2 |
| 78bb0382 | 22.8 | 23.7 | 75 | 22.7 |

上述输入复制到 `failure_inputs/`，供恢复软代价起点逃逸契约的真实回归使用。它们是各订阅最新回调值，不是工作进程原子快照；第二份也不能冒充最终停车地图。原始失败与停止结果不追溯修改。

退出健康单独记录：七个 launch 进程正常结束；Gazebo 收到 SIGINT 后超过 5 秒未退出，ROS launch 升级 SIGTERM，再等待 10 秒后升级 SIGKILL，最终 -9。该项属于停止期间未正常退出，不能记为 clean，也不是运行中已证明的碰撞崩溃。停止后核对无自有 Gazebo、launch 或验收进程残留。

`summary.json`、`trial_summary.json`、`manifest.json`、`install_audit.json`、格内场景及两组回归输入逐字复制，十一份复制文件 SHA-256 已核对。`candidate_audit.json` 保存中断区间指标与退出证据；`verified_parameters.json` 保存真实读回；`status_timeline.json` 保留状态序列；`provenance_check.json` 证实新候选运行前后所有源码、安装和验收脚本条目相同，并索引旧 4ef 安装完整指纹。`source_index.json` 保存所有原始日志和地图绝对路径、大小及 SHA-256。

完整原始目录：`/home/usv/usv_migration/results/python-factor025-candidate-001`。完整前后 provenance 与安装审计位于同级后缀文件，原始事件和地图保留不覆盖。
