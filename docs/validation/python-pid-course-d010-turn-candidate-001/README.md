# 固定 PID 航向微分增益 0.10：转弯三次候选

本组依照采集前提交的 [航向阻尼协议](../pid-course-damping-protocol.md)，在原 `3afb25f84232e560c62599a721dc0bfbe1a21657` WSL 源码及普通安装上，使用外部完整 controller YAML，仅将 PID k_d_course 从 0.0 改为 0.10。三次转弯均实际采集并保留，**功能 3/3 通过，但有效源时间配对只有 1/3 通过**。协议要求三对全部通过后才能继续，因此直航三次、圆柱三次均未运行，相关结果目录未创建；不替换编号或择优重试。

## 固定参数与复现来源

外部文件为 `/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/config/pid_course_d010.yaml`，SHA256 `ad00230c6e99bb14a3c811c33a48e7b928b8e2b17f445a4ade291023f2732f1e`。完整参数加载器验证 84 项，唯一差异是 `usv_simulation.controller.pid_controller.k_d_course: 0.0 → 0.1`。PID 预瞄 2.6 s、其余五个增益、目标位置/速度容差以及全部 LQR 配置不变，原安装 YAML SHA256 为 `3e11175489dd7d32d06bd56c62e917a1285c47190f5d60276cebefa9deeb0e72`。原文 diff 和解析后差异见 [config_audit.json](config_audit.json)；逐例实际节点读回见 [verified_parameters.json](verified_parameters.json)。

沿用格内场景 SHA256 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`、factor=0.25、LOS 距离 1 m / 切换 5 m / 单步推进、2.2×1.2 m 船体、0.2 m 裕度、4 m 横向上限与全部原停止规则。没有在线参数变更，也没有修改原 PID 公式或 yaw wrap。harness 使用 domain 182，session 94089 自然退出 0；before/after 源/安装/验收文件 513/680/88 逐字段完全相同，候选 YAML SHA 前后也一致。

## 三条原始结果和阶段门槛

| 重复 | 功能 | 仿真时间/s | legacy RMSE/m | 源时间 RMSE/m | 源时间/同编号 C++ | 时间/C++ | 源时间配对 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| turn 01 | pass | 20.6 | 0.116792041 | 0.117950929 | 1.251891668 | 0.911504425 | fail |
| turn 02 | pass | 20.4 | 0.099625523 | 0.100576994 | 0.870964667 | 0.883116883 | pass |
| turn 03 | pass | 20.5 | 0.108854286 | 0.109788111 | 1.389530876 | 0.876068376 | fail |

三个试验各执行一次全局搜索。对照固定为 baseline-grid-interior-001 的 CPP PID turn 同编号。source_time_trapezoid / otter_fake_localization 契约与 0.5 s 覆盖上限、1.10 比例门槛均未改变。三对覆盖全部 valid，最大源间隔约 0.1 s，首尾覆盖缺口为零或浮点舍入量。legacy 比例为 1.096625064 / 0.814755563 / 1.286047011，即 2/3 通过；它与源时间的 1/3 分别保留，不能互换通过项。

原样独立比较报告见 [comparison/report.json](comparison/report.json)，SHA256 `d138bb73190b507ba331995dabbb3357bdd6a8f9e7b5cfd0fc21a9f1b2efb0dd`。完整报告仍包含基线其余六个未运行 Python 候选的不可比项，根汇总为 9 对、3 可比、1 pass、2 fail、6 不可比、all_passed=false。不能删除未运行项或把功能通过扩大为候选配置准入。

## 推力方向、状态与退出

三例从首个非零 sample 开始的一秒窗口，均未观察到正的 right−left 差动推力；初始转弯方向为朝向南方的负 yaw。原始 sample 行号、每轴命令接收事件、实际 yaw/r 与来源 SHA 保存在 [initial_direction_audit.json](initial_direction_audit.json)。该证据仅支持这些记录没有观察到初始反向：命令/状态按异步回调采集，接收间隔不等于控制器内部 dt，不能据此声称内部 dt<0.0532 s 的离线 wrap 风险已经排除。后段差动推力反号可能是正常阻尼，未将其自动判为初始翻转。

三例已接收的状态均为 NAVIGATING、SETTLING、GOAL_REACHED，没有观察到 SIMULATION_PAUSED、NO_PATH 或 CONTROL_ERROR，也没有 control_result_* 错误计数。约 1 Hz 状态不能排除未采到的亚秒级变化。首个非零命令后，零命令 sample 数分别为 25/27/25，该计数包含正常目标收尾，不能解释成异常停车次数；原始推力、状态和位置记录保持 WSL 原样。全部 24 个 launch 进程均 clean 退出，结束后没有自有仿真或验收进程残留。

## 性能范围和归档

三例节点窗口的 control/mapping/backlog 子项均 pass。最大已报告控制 P99 为 5.2961 / 7.3932 / 7.4682 ms；扫描 P95 为 23.2137 / 23.5941 / 24.4183 ms，扫描周期约 67 ms。队列完整观察区间数 2/3/3，跨度 10.7464/16.0554/16.0344 s，观察区间内 expired/overflow 增量均为零。这里的“最大已报告分位数”不是池化分位数或算法 WCET。

本候选没有 OS 完整旁路采样，节点窗口结果不能替代正式全矩阵资源验收。环境代理在候选开始阶段报告其短源码/ROS 预检已结束；本组仍按候选范围报告，不升级为隔离正式性能结论。

完整原始目录为 `/home/usv/usv_migration/results/python-pid-course-d010-turn-candidate-001`，同级保留 before/after 指纹及配置差异审计。归档共 13 份原件复制后逐个核对 SHA，包含 summaries、manifest、场景、构建/安装证据、原安装及候选 YAML、配置差异和比较报告；审计派生表另存状态、建图窗口、参数与初始方向。[source_index.json](source_index.json) 索引完整原始事件/地图/日志的绝对路径、字节数和 SHA。[run_audit.json](run_audit.json) 明确记录后两阶段未运行与停止原因。默认配置与旧失败记录均未修改。
