# 原 C++ 基线验证摘要

固定基线 632734b，原 cpp-baseline 标签 ed4ed18 保持不变。LOS lookahead=1 m、switch_dist=5 m；未应用待选 switch1 补丁。验收脚本为冻结的 8ebb23e。

## PID 导航结果

9/9 导航样本通过。到达、轨迹/推力、横向偏差和艇体间隙判据与进程退出健康分列；本结果不能称为完整生命周期验收通过。

|样本|仿真耗时/秒|初始路径横偏最大值/米|最小艇体间隙/米|
|---|---:|---:|---:|
|cpp_pid_single_obstacle_01|26.100|0.6081|0.3332|
|cpp_pid_single_obstacle_02|27.800|0.5981|0.3134|
|cpp_pid_single_obstacle_03|27.800|0.5882|0.2758|
|cpp_pid_straight_01|26.900|0.4182|不适用|
|cpp_pid_straight_02|23.600|0.3530|不适用|
|cpp_pid_straight_03|26.900|0.4139|不适用|
|cpp_pid_turn_01|22.600|0.3523|不适用|
|cpp_pid_turn_02|22.400|0.3526|不适用|
|cpp_pid_turn_03|22.700|0.3031|不适用|

逐例原始摘要位于 pid-summaries；完整原始日志目录为 /home/usv/usv_migration/results/baseline-matrix-002。

## LQR 有效观察与限制

直航、转弯、圆柱三个场景均实际启动 LQR，取得路径、轨迹和推力，并从节点回读参数。直航发生距离到达锁存后的惯性越界，转弯横偏超过 4 m，圆柱的艇体几何代理间隙为负。均按诊断停止规则提前中止，原摘要保留 interrupted；没有把未运行重复例视为失败，也不把三例当作完整重复矩阵。

详见 [LQR 三场景报告](lqr-three-scenario-report.md)和[直航停机诊断](lqr-straight-stop-diagnosis.md)。复制的原报告内部路径仍按原 WSL 目录解释，仓库对应诊断就在本目录。LQR 转弯、圆柱完整日志在 /home/usv/usv_migration/results/baseline-lqr-diagnostics-003，直航在 /home/usv/usv_migration/results/baseline-matrix-002/cpp_lqr_straight_01。三例原 sim_env_node 收尾均为 -6；转弯诊断的 gzserver 收尾也为 -6，均保留在报告 failed_processes 字段，不影响前面已发生的越限事实。

## 可追溯文件

copied-evidence-manifest.json 记录每个精简副本的原始 WSL 路径、SHA256 和字节数。未复制大日志、ROS事件流或黑色截图。

额外运行前/后证据仍在 /home/usv/usv_migration/results/baseline-matrix-002-provenance、baseline-matrix-002-provenance-after.json、python-matrix-002-provenance-after.json；after 文件只代表采样时状态，不能替代运行前或运行中证据。
