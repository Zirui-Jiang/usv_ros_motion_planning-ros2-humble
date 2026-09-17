# Python 迁移实施状态

原功能包内的 Python 重构、旧算法源码清理、独立构建和无界面闭环复验已完成。
当前开发分支为 `refactor/python-navigation`；实际被测源码固定为
`dfb1fb7fcb77feb868ee87ac986c70d7c4cab537`。后续提交只补充文档与证据，不自动更新
这个独立安装。原始 `main` / `cpp-baseline` 保持 `ed4ed18`，本仓库 Git 身份为 Zirui。

六个算法/建图包使用 `ament_python`，`sim_env` 使用 `ament_cmake_python`，保留
C++ 真值定位和 Nav2 代价地图支撑。Gazebo、水动力、推进器、传感器、RViz 插件及
模型资源继续复用。默认算法为 A* → LOS → PID，也可选择 Dijkstra、GBFS、LQR。
被替换的 78 个 C++ 文件已精确移除，原实现可从基线恢复，见
[M5 清理记录](validation/m5-cleanup/README.md)。

| 验证层次 | 实际结果 | 证据与边界 |
| --- | --- | --- |
| 静态检查与构建 | 全新普通安装 13 包构建通过，33 项安装审计通过 | 六包及 sim_env 跨包导入、节点入口、C++ 支撑动态链接均核对实际 install；基线另有独立 13 包构建 |
| 单元测试及源码回归 | 全仓 439 项通过，无失败、错误或跳过；正常 C++ 数值对照 9/9 通过 | 覆盖算法、建图、参数、真实 ROS 回归、目标容差、源时间计分与均值比较；不是全输入 C++ 等价证明 |
| ROS 集成 | 已执行的接口及故障检查通过 | goal/odom/scan/TF/clock/raw costmap/路径/推力、在线参数、取消、输入过期、时钟回退和规划进程隔离 |
| 闭环验收 | 清理后固定版本 18/18 导航、3/3 故障功能通过；完整可比场景均值 2/2 通过 | 4 组缺少合格完整基线；控制与扫描耗时 18/18 达标，积压 15 通过、3 项证据不足，退出异常另列 |

[干净构建与安装](validation/python-clean-final-001/README.md)、
[439 项 JUnit](validation/python-clean-final-001/source-tests.xml)、
[正常数值对照](validation/python-clean-final-001/parity/report.md)保存了实际版本、路径和
SHA。九项正常数值 fixture 在 `atol=1e-6, rtol=1e-5` 下通过，最大绝对差约
`3.05e-15`；SG 端点和轨迹括区间两项已知修复另列，不将 A*/LOS/建图全部输入
声称为旧输出逐项一致。

用户已确认“按场景均值验收，同时报告每次和最差值”。本轮在采集前提交了
[固定协议](validation/scenario-mean-release-protocol.md)，使用每个场景固定三次的
RMSE 算术均值和到达时间算术均值，主度量为 `source_time_trapezoid`，Python/C++
门槛为 ≤1.10。完整结果见
[清理后比较报告](validation/python-clean-release-comparison-001/report.md)。

| 完整可比场景 | Python / C++ 平均 RMSE / m | 均值 RMSE 比值 | Python / C++ 平均到达时间 / s | 均值时间比值 |
| --- | --- | --- | --- | --- |
| PID 直航 | 0.059918 / 0.056648 | 1.057712，通过 | 20.800 / 22.500 | 0.924444，通过 |
| PID 转弯 | 0.085698 / 0.096236 | 0.890497，通过 | 20.633 / 23.033 | 0.895803，通过 |

本轮七个有效逐次配对也全部通过，最大逐次源时间 RMSE 比值为 1.075555；另外
11 对不可比。完整三次场景不可比范围为 PID 圆柱（C++ 02/03 未通过安全裕度门槛）
及 LQR 三场景（无有效原版结果）。已有失败值继续展示，不用有效子集拼成三次均值。
Python 六个场景各自的三次值、样本标准差与最差值均保留；LQR 转弯平均 RMSE
约 0.993 m，不能仅据到达就认为其跟踪质量与 PID 相同。

本轮六次受控圆柱绕障的最小艇体扫掠间隙为 0.753900 m，高于 0.2 m 门槛。
取消后零推力观察延迟约 2.796 ms，独立里程计断流约 424.893 ms，均未观察到
截止后继续发布非零推力。完整原始来源、各次参数与退出记录见
[新正式矩阵](validation/python-clean-release-matrix-001/README.md)。运行前后
435 个源码、680 个安装文件和 90 个验证文件指纹完全一致。

性能和运行异常分别保留：三例因连续 TF 过期而无法证明正常 TF 条件下持续吞吐，
积压状态为 unknown，不能改成通过；三次 LQR 达到 80 ms 求解预算后输出零推力
并恢复。20 条 Gazebo 试验中 18 条各八个进程正常退出；PID 圆柱 01/03 的 gzserver
在 SIGINT 收尾时出现 boost self-join、退出 -6，其余节点正常。独立 odom 夹具未
直接记录返回码，退出健康仍为 unknown。比较器总字段 `all_passed=false` 保留了
不可比及性能证据不足的范围，不以场景均值通过代替所有证据全通过。

独立正常 TF 吞吐试验的 450 帧、2000 束、15 Hz 负载全部积分一次，处理 P95
23.288 ms，无过期、溢出或持续积压。它与本轮 mapper 三个组件源码一致，但仍是
独立输入环境，不能回写 live unknown，见
[正常 TF 吞吐](validation/normal-tf-throughput-001/README.md)。全部导航窗口的
CPU/RSS 旁路观测与独立断流尾部覆盖限制见
[进程性能](validation/python-clean-release-matrix-001-process-performance.md)。

原始矩阵、逐次超限、旧候选和中断记录全部保留在
[完整验收报告](python-migration-validation.md)。旧 `3afb25f` 的逐次结果仍为 6/7，
其用户事后澄清口径的补充结果见
[历史均值报告](validation/scenario-mean-acceptance/README.md)；本轮独立采集的 7/7
不替换它。Kd=0.10 和预瞄 3.0 s 候选均未成为默认配置。

环境为 WSL2 Ubuntu 22.04、Python 3.10、ROS 2 Humble 与 Gazebo Classic 11.10.2。
安装时替换的冲突依赖及备份见 [环境变更](validation/environment-changes.md)。
WSLg/GUI 初始化通过，实际可视画面尚未核验，不声明 GUI 画面验收通过，见
[GUI 记录](validation/gui/README.md)。

启动、YAML 路径覆盖、别名兼容、在线调参、参数导出和回退操作见
[工程使用指南](python-navigation-guide.md)；行为修复、RDP/SG 实际独立调用状态及
未迁移曲线清单见 [迁移差异](python-migration-differences.md)。本次没有新增
AIS/Radar 融合、COLREGs、DWA 或 MPC。
