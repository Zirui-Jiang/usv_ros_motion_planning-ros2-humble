# 单步 LOS 候选 273a1fb 的真实结果与暂停原因

固定安装提交为 `273a1fb8d1ccccbd32cea25b6a30aad01265a473`，包含恢复的 LOS 单步推进、规划队列继承修复、LQR Q1/R20/2 s 默认配置及新增建图队列观测。未使用参数覆盖或外置 controller YAML。运行前后源码、安装、验收脚本及 Git 状态完全一致，详见 `provenance_check.json`。

| 用例 | 原始结果 | 仿真耗时/s | 固定路径 RMSE/m | 最小全艇间隙/m |
| --- | --- | ---: | ---: | ---: |
| PID 直航 | completed / passed | 20.4 | 0.173062 | 无障碍 |
| PID 转弯 | completed / passed | 20.6 | 0.194332 | 无障碍 |
| PID 圆柱 | completed / passed，曾有瞬态 NO_PATH | 23.1 | 0.302765 | 0.305367 |
| LQR 直航 | completed / passed | 15.7 | 0.105371 | 无障碍 |
| LQR 转弯 | completed / passed | 17.0 | 0.904809 | 无障碍 |
| LQR 圆柱 | interrupted / 未通过 | 不计完成耗时 | 不计完成指标 | 不计完成指标 |

审查 PID 圆柱的完整状态日志时发现 sim=14/15 s 曾报告 `NO_PATH: start or goal is blocked`，该例之后恢复并到达。依当时暂停规则，监督器在发现后中断仍运行的 LQR 圆柱；此前已完成的五例保留原始结果。不能将本轮写成六例全部完成，也不能因圆柱最终到达而抹掉中途安全门事件。后续试验采用另行明确的规则，不追溯改写本次暂停记录。

PID 圆柱的即时失败输入复制到 `pid_circle_failure/`。地图时间为 13.9 s、状态时间为 14.0 s，捕获源于各订阅回调的最新值，并非规划工作进程的原子快照。状态为 `(27.652036, -8.092282, 0.173531 rad)`；中心格 `(400,399)` 的 raw 代价为 162，因此原 A* 的 127 阈值会拒绝起点。对这份即时输入，完整矩形与 raw≥254 单元的检查通过，独立物理圆柱静态间隙约 0.511808 m。状态累计已有一次 map invalidation 和五次 current footprint block；这份较晚快照不足以否定更早的地图/姿态阻断，不能据此声明那些判定均为误报。原始状态时间线与输入均保留供重放。

PID 直航/转弯最终各一次全局搜索；圆柱因为失效与重新搜索累计十九次。按固定 repeat=1 对照 C++ 基线，PID 直航和圆柱 RMSE/时间两项通过；PID 转弯 RMSE 比例约 1.282605，超过 1.10，时间比例约 0.911504 通过。导航通过与相对基线门槛分别记录。

退出健康亦独立记录：LQR 转弯完成导航后 Gazebo 退出 -6，出现 `boost::thread_resource_error`；该例其他七个节点正常退出。其余五例均有八个 clean 退出标记，包括被监督器中断后按流程清理的 LQR 圆柱。

`summary.json`、`summaries/`、`manifest.json` 均逐字复制原始文件；`candidate_audit.json` 保存分类、状态事件和带行号退出异常；`verified_parameters.json` 保存实际参数读回。`source_index.json` 保存完整 WSL 原文件的绝对路径、字节数和 SHA-256，并列出复制来源。原始事件流与完整日志仍在 `/home/usv/usv_migration/results/python-single-step-candidate-001`，未覆盖旧试验。
