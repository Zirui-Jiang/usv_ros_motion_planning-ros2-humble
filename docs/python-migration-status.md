# Python 迁移实施状态

当前源码位于 `refactor/python-navigation`，原始 C++ 基线保存在 `main` 与不可变标签
`cpp-baseline`（`ed4ed18`）。Git 作者身份仅在本仓库配置为 Zirui。

原来的七个 ROS 功能包继续保留：六个算法/建图包使用 `ament_python`；`sim_env`
使用 `ament_cmake_python`，保留真值定位和 C++ 代价地图支撑。Gazebo、水动力、
推进器、传感器、RViz 插件及模型资源沿用原工程。

| 验证层次 | 当前状态 | 证据与边界 |
| --- | --- | --- |
| 静态检查与构建 | C++ 与 Python 各 13 包构建通过 | 替换源码清理后的全新普通安装区构建待最后执行 |
| 单元测试及源码回归 | `3afb25f` 全仓 404 项通过 | 含算法、实际 ROS 节点、软区驶出、路线复用、建图 deadline、源时间度量、栅格终点与小容差目标连接 |
| ROS 集成 | 已通过已执行的接口与故障回归 | 覆盖目标、状态、地图、TF、时钟、扫描、推力、在线参数、任务取消、断流、时间回退和规划进程隔离 |
| 闭环验收 | 已完成 `e381832` 格内矩阵 18/18 导航、3/3 故障；新终点修复待复验 | 旧墙钟七个有效配对六过一败；源时间回溯四过三败；11 个无有效 C++ 基线；live 性能 17 通过、1 unknown |

原场景矩阵的数值与验收脚本版本为 `4ef01a6`；原始结果和运行前后 SHA 校验见
[正式矩阵归档](validation/python-final-matrix-001/README.md)。已运行结果与仍未满足的
门槛集中列在 [验收报告](python-migration-validation.md)，逐例比例、滚动窗口耗时与
积压判定见 [比较报告](validation/original-final-comparison/report.md)。

正常数值对照直接链接原 C++ 库，9/9 通过，最大绝对差约 `3.05e-15`，
使用 `atol=1e-6, rtol=1e-5`。覆盖重采样、RDP、SG 内部卷积、轨迹查询、PID 和 LQR，
不表示 A*/LOS/建图所有输入均与旧版逐项判等。
见 [数值对照](validation/parity-1a82de4/report.md) 与
[当前复验 JUnit](validation/source-tests-grid-endpoint.xml)。早先 264 项记录对应 `4ef01a6`，
334 项对应 `79a109f`，368 项对应 `e381832`。数值库对照将在清理后的新安装版本重跑。

追加的 [格内起点协议](validation/grid-interior-protocol.md) 在执行前固定，专门检查
原起点恰在栅格边界造成的参考路径差异。它不替换原协议、失败试验或重复编号；
追加 C++ PID 已完成九次，七次通过，两次圆柱安全裕度不足。
Python 对应矩阵已暂停：九条 PID 已完成，其中两条圆柱安全裕度不足；LQR 首例中断，
其余未运行。第二条圆柱的原扫掠保守下界为负，精细插值复核的下界仍为正约 0.030 m，
不支持已观测艇体相交，但明显未达到 0.2 m。factor=0.25 首例候选的最小观察间隙
增至约 0.72 m，却在 raw75 的软膨胀起点持续 `NO_PATH`，已中断且未计通过。
`c0139be` 已恢复原 C++ 的软区严格降代价驶出能力，并统一路径段/重采样检查，
三次 PID 圆柱候选通过，间隙为 0.747–0.829 m。
`e381832` 补齐调度层路线复用的一致性；完整复验按
[预先固定协议](validation/soft-escape-release-protocol.md) 单独记录，已经完成。
该版六次圆柱均功能通过，PID 最小间隙 0.792–0.831 m，LQR 为 0.869–0.875 m。
详见 [格内复验归档](validation/python-release-grid-001/README.md) 与
[原墙钟比较报告](validation/python-release-grid-comparison/report.md)。

随后发现墙钟采样重复计入暂停期间的同一姿态；源时间修复明确选用
`source_time_trapezoid`，保留旧值并列。旧版三个直航仍失败，PID 2.5 s 候选五条中
也仅两条通过，已撤回该参数调整。`3afb25f` 保留 PID 2.6 s，并恢复原栅格终点，
以终止圆包含关系解决精确目标容差；按
[新预固定协议](validation/grid-endpoint-source-time-protocol.md) 进行独立复验。
当前尚未启动最终旧 C++ 源码清理。

正常 TF 已可查的独立建图吞吐验证已通过：450 帧、每帧 2000 束、15 Hz，处理
P95 为 23.288 ms，无过期、溢出或持续积压，三次重复扫描未重复积分。
这只验证已声明的正常 TF 条件，不回写 live 矩阵中的 unknown。
完整证据及第一次受系统时间回退影响的失败均见
[受控建图吞吐记录](validation/normal-tf-throughput-001/README.md)。

迁移已修复二维索引越界、LOS 空路径与时间/速度不一致、速度坐标系混用、重复扫描
积分、过期输入沿用、首搜后的重复搜索、局部碰撞复核及高速入圈提前完成等问题。
代码、配置校准、原行为差异和未迁移曲线清单见
[迁移差异](python-migration-differences.md)。历史候选与失败证据完整保留在各自归档，
不用于替补正式矩阵。

Nav2 支撑节点的析构顺序缺陷已经修复；独立退出回归和闭环节点退出均有记录。
Gazebo 在原正式矩阵的两个用例结束后仍出现 Boost self-join 异常，导航结果与进程退出
健康分别统计，当前缺少足以锁定调用链的崩溃栈。
见 [退出诊断](validation/gazebo-exit-race.md)。

本机使用 Gazebo Classic 11.10.2；安装时替换了 49 个冲突的 Garden/gz-sim7 包，
未清除用户数据。依赖、备份和版本见 [环境记录](validation/environment-changes.md)
与 [清单](validation/environment.json)。WSLg/GUI 初始化通过，但实际可视画面的核验
范围见 [GUI 记录](validation/gui/README.md)，不据初始化推断画面已经验收。

构建、原启动入口、YAML 路径覆盖、别名兼容、在线调参与回退操作见
[工程使用指南](python-navigation-guide.md)。本次未新增 AIS/Radar、COLREGs、DWA 或 MPC。
