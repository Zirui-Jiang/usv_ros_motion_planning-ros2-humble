# Python 迁移实施记录

- 原始基线：`ed4ed18`，标签 `cpp-baseline`，分支 `main`。
- Python 开发分支：`refactor/python-navigation`。
- 提交身份：仅本仓库配置为 Zirui。
- 范围：原包内迁移导航、控制、轨迹和 LiDAR 建图；保留 Gazebo/水动力/推进器/RViz 插件与真值定位。

## 验证记录

本文件随实施更新。未执行的检查不得视为通过。

| 层次 | 当前状态 |
|---|---|
| 静态检查与构建 | 原 C++ 13 包构建通过；Python 13 包构建通过，独立对照工具通过 COLCON_IGNORE 隔离 |
| 单元测试 | 已校准候选在 WSL 全仓库 264 项通过，包含真实 ROS 节点测试；JUnit 位于 docs/validation/source-tests-calibrated-candidate.xml |
| ROS 集成 | 已验证地图/TF/时钟、重复扫描、在线调参、独立规划进程、地图失效停止、输入断流、取消和时间回退；完整安装区能够启动 Gazebo 导航 |
| 闭环验收 | Python `python-pid-final-001` 的 PID 9 个导航与 3 个故障检查通过；部分相对 C++ 的固定路径 RMSE 超过 10%，仍在修复和复验；LQR 配置正在独立验证，暂未完成最终验收 |

最近 PID 正式试验版本为 `1a82de4`。最新调度修复为 `bbf629d`，
已通过 13 项队列/ROS 回归。后续候选还恢复 LOS 原单步航点推进并增加建图队列
驻留观测，整仓库 237 项通过；尚不能把旧安装区试验归于这些修复。
C++ 对照采用 `baseline-validation`：
`d9a6023` 仅修复库安装、控制器启动选择与等值数组类型；`632734b` 单独记录
LOS 前视距离 1 m 和模型生成等待 120 s 的校准配置。原 `cpp-baseline` 保持不变。

`273a1fb` 单步 LOS 候选完成 PID 三场景、LQR 直航/转弯，共五条导航通过；
LQR 圆柱在矩阵暂停时标记为 interrupted，未计通过。PID 转弯相对基线 RMSE
仍增加 28.26%；独立将 PID 预瞄校准至 2.6 s 后，单次转弯 RMSE 降至
0.136225 m、耗时 20.5 s，两项对照通过。当前已将这一参数纳入待正式复验配置。
候选与中断证据见 [单步 LOS 归档](validation/python-single-step-candidate-001/README.md)
及 [PID 预瞄归档](validation/python-pid-preview26-turn-candidate-001/README.md)。

matrix002 圆柱第 3 例最小全艇几何间隙约 0.442 m，但 LOS 预测被代价地图拒绝后未到达。
离线捕获输入表明航点切换距离 5 m 会跨越转角，1 m 是待闭环验证的候选值。
只有初始地图与较晚状态可用于本轮重放，不能宣称精确复现停车末期的地图。
详见 [失败数据说明](../scripts/validation/diagnostics/obstacle_stall_002/provenance.json)。

后续 switch=1 m 候选也发生停车，即时 raw 快照证明原局部中心软代价硬门有误。
现已接入全艇与 lethal/unknown 方格的扫掠检查，保留全局搜索阈值；捕获输入回归通过。
同时修正近目标减速重置与高速入圈过早完成。LQR 零预瞄候选虽然最终到达，
但实际发生持续转圈，123.3 s 才完成，不能作为稳定跟踪的交付配置。
保持原 A/B 与 Riccati 求解的两秒预瞄、低位置权重候选正在验证，依据见
[LQR 数值诊断](validation/lqr-preview2-q1-r20-diagnostic.md)。

`python-pid-final-001` 使用固定配置完成 9/9 导航和 3/3 故障检查，
取消到零推力约 2.57 ms，里程计断流到零推力约 0.413 s。
这些成功不替代相对误差、性能和退出健康门槛。
转弯首例的固定参考 RMSE 为 0.449 m；启动时第二次全局搜索将参考线
从 x=23.25 m 切到 x=22.75 m，在线参考 RMSE 为 0.147 m。
`bbf629d` 把成功首搜路径带入同一任务最新待处理快照，避免空路径重复搜索；
最新状态、地图及参数仍保留，且继续进行地图和艇体复核。
该诊断不改变预先规定的固定参考评估方法。

C++ PID 基线 9/9 导航成功，但旧导航进程退出异常单独记录。
C++ LQR 直航高速入圈后保持停车状态并漂出目标，转弯超过 4 m 横偏边界，
圆柱用例出现负艇体几何间隙；这些场景没有有效完成的对照结果，
不能把未执行的剩余重复记为失败或通过。详见 [基线证据](validation/baseline/)。

已完成的正常数值对照为 **9/9**，最大绝对差约 `3.05e-15`，直接链接原 C++ 库执行。
覆盖和未覆盖范围见 [对照报告](../scripts/validation/parity/evidence/report.md)。这不等于
所有算法逐项与 C++ 全量判等，也不替代 Gazebo 闭环。

早期直航与转弯已经有成功运行记录。原 LOS 前视距离 5 m 在圆柱场景导致预测轨迹
穿越代价地图障碍，Python 安全检查保持零推力。单独诊断中在线改为 1 m 后约
22.6 s 到达，全艇与圆柱的保守最小间隙约 0.647 m。该诊断中途改变参数，
不纳入固定配置矩阵；原始资料位于 [诊断记录](../scripts/validation/diagnostics/obstacle_actual_001/provenance.json)。

Nav2 支撑节点的退出缺陷已修复并通过独立回归：正常活动场景退出返回 0；
缺 TF 时取消启动明确返回 1，未被计为成功启动。测试依据见
[退出回归说明](../src/sim_env/test/README.md)。正式矩阵还会记录完整 Gazebo 退出状态。

环境使用 Gazebo Classic 11.10.2。安装时替换了 49 个冲突的 Garden/gz-sim7 包，
未清除用户数据；原源备份、变更和版本见 [环境记录](validation/environment-changes.md)
及 [版本清单](validation/environment.json)。

## 已确认的迁移修复范围

二维索引越界、LOS 空路径及航向分支、轨迹时间与非负速度、本艇速度坐标系、
建图重复积分及无效射线、旧参数键与层级、失效输入零推力。

原始 C++ 输出在这些缺陷场景中不能作为正确性判据。
