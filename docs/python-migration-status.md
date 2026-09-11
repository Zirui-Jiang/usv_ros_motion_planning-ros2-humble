# Python 迁移实施状态

当前源码位于 `refactor/python-navigation`，原始 C++ 基线保存在 `main` 与不可变标签
`cpp-baseline`（`ed4ed18`）。Git 作者身份仅在本仓库配置为 Zirui。

用户已明确最终按同场景三次均值验收，同时报告逐次及最差值。旧逐次失败保持原样，
新安装矩阵按 [提前固定的收尾协议](validation/scenario-mean-release-protocol.md)执行。
M5已精确移除78个被替换C++文件，[删除记录](validation/m5-cleanup/README.md)
完整保留；全新普通安装及闭环复验正在准备。

原来的七个 ROS 功能包继续保留：六个算法/建图包使用 `ament_python`；`sim_env`
使用 `ament_cmake_python`，保留真值定位和 C++ 代价地图支撑。Gazebo、水动力、
推进器、传感器、RViz 插件及模型资源沿用原工程。

| 验证层次 | 当前状态 | 证据与边界 |
| --- | --- | --- |
| 静态检查与构建 | C++ 与 Python 各 13 包构建通过 | 替换源码清理后的全新普通安装区构建待最后执行 |
| 单元测试及源码回归 | 当前源码全仓 423 项通过；闭环被测 `3afb25f` 为 404 项 | 最新增加目标容差数值精度守卫；含算法、真实 ROS 节点、建图 deadline、源时间度量及栅格终点检查 |
| ROS 集成 | 已通过已执行的接口与故障回归 | 覆盖目标、状态、地图、TF、时钟、扫描、推力、在线参数、任务取消、断流、时间回退和规划进程隔离 |
| 闭环验收 | `3afb25f` 完整矩阵 18/18 导航、3/3 故障功能通过，完整验收未通过 | 预固定源时间指标七个有效配对六过一败；PID 转弯 01 比值 1.10912972 超过 1.10；11 对不可比；live 性能 16 通过、2 unknown |

原场景矩阵的数值与验收脚本版本为 `4ef01a6`；原始结果和运行前后 SHA 校验见
[正式矩阵归档](validation/python-final-matrix-001/README.md)。已运行结果与仍未满足的
门槛集中列在 [验收报告](python-migration-validation.md)，逐例比例、滚动窗口耗时与
积压判定见 [比较报告](validation/original-final-comparison/report.md)。

正常数值对照直接链接原 C++ 库，9/9 通过，最大绝对差约 `3.05e-15`，
使用 `atol=1e-6, rtol=1e-5`。覆盖重采样、RDP、SG 内部卷积、轨迹查询、PID 和 LQR，
不表示 A*/LOS/建图所有输入均与旧版逐项判等。
见 [数值对照](validation/parity-1a82de4/report.md) 与
[当前源码 JUnit](validation/source-tests-goal-tolerance.xml)。早先 264 项记录对应 `4ef01a6`，
334 项对应 `79a109f`，368 项对应 `e381832`。数值库对照将在清理后的新安装版本重跑。

最新目标容差守卫拒绝 `≤1e-9 m` 的配置，避免 LOS 去重精度与控制器完成圆冲突。
定向 97 项及六次真实 ROS 源码启动检查通过，PID/LQR 默认参数、runtime 和 LOS
计算均未改动，见 [红绿记录](validation/goal-tolerance-boundary/README.md)。这项源码
回归不冒充已重新构建安装或闭环复验。

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
[新预固定协议](validation/grid-endpoint-source-time-protocol.md) 完成独立复验。
详见 [完整矩阵](validation/python-grid-endpoint-final-matrix-001/README.md) 与
[源时间比较](validation/python-grid-endpoint-final-comparison/report.md)。全部 27 份正常
导航源时间覆盖有效。PID 转弯 01 的超标集中于实际前、中段横偏，同配置候选的通过
不能替补正式失败；[分段诊断](validation/python-grid-endpoint-final-comparison/turn01-diagnostic/README.md)
保留了启动延迟和暂停相位差。原 world 的 PMS 波浪仍生效、起航绝对仿真时刻未锁定，
属于 [未控制因素](validation/ocean-repeatability-audit/README.md)，尚不能据此确定失败原因。
当时按逐次口径继续诊断，尚未执行M5。随后用户明确采用场景均值口径；原完整矩阵
的PID直航/转弯RMSE均值比分别为1.053058和0.928758，到达时间均值比为0.921481
和0.887120。完整三重复圆柱基线不足、LQR缺少有效基线，保持不可比范围。

后续单参数 `k_d_course=0.10` 候选三次转弯均到达，但源时间配对仅 1/3 通过，
按预先固定协议停止直航和圆柱后续阶段；该值未成为默认配置。
见 [阻尼诊断](validation/pid-course-damping-diagnostic/README.md) 和
[失败候选](validation/python-pid-course-d010-turn-candidate-001/README.md)。

本轮控制与扫描计算耗时各 18/18 通过，PID/LQR 直航 03 因连续 TF 过期保留积压
unknown。两次 LQR 求解达到 80 ms 预算后输出零推力并恢复；两次 gzserver 在收尾
SIGINT 时退出 -6，分别统计，不影响原始采集是否完成的记录。独立进程采样完整覆盖
18 条导航，导航加 worker CPU 均值为单核的 10.46%、P95 为 20.01%，同刻 RSS 合计
峰值 115.63 MiB，详见 [进程性能](validation/python-grid-endpoint-final-matrix-001-process-performance.md)。

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
