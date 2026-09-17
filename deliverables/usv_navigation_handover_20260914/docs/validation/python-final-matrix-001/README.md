# 原场景正式 Python 18+3 矩阵归档

固定数值与验收脚本提交为 `4ef01a645ada5fb816c8e1a412fa6cd779723539`，原场景 SHA-256 为 `be639c190dbed2cc1c7adacf65cf080f4f1e21c2b9e993dd24aab14c22753119`。PID/LQR 各三种场景、每场景三次，共十八条导航，以及目标更换、取消、里程计断流三条故障检查全部 completed/pass；原始 harness 返回 0，run_finished=true，interrupted=false。

本目录保留原始功能结果，不将它等同于所有验收门槛通过。按原场景同重复编号对照 baseline-matrix-002，九对 PID 有八对满足 RMSE 与耗时均不超过基线 1.10 倍；straight_02 的 RMSE 比例超限，原判定保持失败，见 `pid_pairwise_check.json`。起点位于栅格边界所产生的参考线差异另行诊断，后续格内平移场景必须使用独立配对结果，不能混入本目录替代失败项。C++ LQR 没有完整通过的配对矩阵，本目录不构造缺失对照。

三次 PID 圆柱的最小全艇扫掠间隙分别为 0.346779、0.533812、0.304069 m；LQR 圆柱分别为 0.670047、0.585905、0.551747 m，均高于 0.2 m 门槛。三次 LQR 转弯 RMSE 分别为 0.958725、1.160958、0.934308 m，这一跟踪质量差异保留，不能由到达通过掩盖。

故障检查中，取消后的零推力响应为 0.004406173 s，里程计断流后的零推力响应为 0.416040847 s，均未在截止后观察到非零推力。目标更换用例完成用时 25.7 s。里程计断流由独立受控发布 smoke 执行，不运行 Gazebo。

退出健康存在两项异常：`python_pid_single_obstacle_03` 和 `python_lqr_turn_01` 在导航完成后 Gazebo 退出 -6，日志含 `boost::thread_resource_error`。每例其余七个节点正常退出。其他十八条 Gazebo 用例均有八个 clean 标记，独立里程计 smoke 无 launch 标记但无异常记录。完整带行号退出证据、暂态非正常状态和失败捕获事件见 `run_audit.json`，不修改 summary 中原始导航结果。

运行前后源码、安装、验收脚本和 Git 状态逐项完全一致。`summary.json`、`manifest.json` 和 `summaries/` 中二十一条摘要逐字复制；`verified_parameters.json` 保存真实参数读回，`provenance_check.json` 保存前后校验摘要。`source_index.json` 记录全部原始事件、地图、状态与日志的绝对路径、字节数和 SHA-256；完整原始数据位于 WSL `/home/usv/usv_migration/results/python-final-matrix-001`，前后完整 provenance 位于同级后缀文件。二十三个复制文件均已核对 SHA-256。
