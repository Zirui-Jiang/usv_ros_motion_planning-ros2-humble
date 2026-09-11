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
| 单元测试 | WSL 全仓库 228 项通过；包含真实 ROS 节点测试，JUnit 位于 docs/validation/source-tests-footprint-settling.xml |
| ROS 集成 | 已验证地图/TF/时钟、重复扫描、在线调参、独立规划进程、地图失效停止、输入断流、取消和时间回退；完整安装区能够启动 Gazebo 导航 |
| 闭环验收 | Python matrix002 已完成 PID 9 例，其中 8 例通过、圆柱第 3 例停车超时；后续矩阵已暂停，C++ 对照正在串行执行，暂未完成最终验收 |

当前正式试验版本为 `8ebb23e`。C++ 对照采用 `baseline-validation`：
`d9a6023` 仅修复库安装、控制器启动选择与等值数组类型；`632734b` 单独记录
LOS 前视距离 1 m 和模型生成等待 120 s 的校准配置。原 `cpp-baseline` 保持不变。

matrix002 圆柱第 3 例最小全艇几何间隙约 0.442 m，但 LOS 预测被代价地图拒绝后未到达。
离线捕获输入表明航点切换距离 5 m 会跨越转角，1 m 是待闭环验证的候选值。
只有初始地图与较晚状态可用于本轮重放，不能宣称精确复现停车末期的地图。
详见 [失败数据说明](../scripts/validation/diagnostics/obstacle_stall_002/provenance.json)。

后续 switch=1 m 候选也发生停车，即时 raw 快照证明原局部中心软代价硬门有误。
现已接入全艇与 lethal/unknown 方格的扫掠检查，保留全局搜索阈值；捕获输入回归通过。
同时修正 LQR 当前参考时刻、近目标减速重置与高速入圈过早完成，待新版本闭环验收。

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
