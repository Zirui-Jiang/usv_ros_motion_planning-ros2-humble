# 栅格末点兼容修复：固定正式 18+3 原始验收归档

本轮固定源码/安装为 `3afb25f84232e560c62599a721dc0bfbe1a21657`。PID/LQR 各三场景、每场景三次，共 18 条导航，以及目标更换、取消、独立 odom 断流三条故障试验，均实际完成，功能门槛为 **21/21 通过**。原 harness PID 113855、starttime_ticks 7948542、ROS domain 182；会话中断后核对原进程身份并继续监督，没有重启、覆盖编号或择优重试。harness 自然退出码 0。

**整体验收没有全部通过。**独立冻结源时间对照为 7 对可比中的 6 对通过，PID turn 01 比例 1.10912971776 超过 1.10；11 对不可比保留。建图积压有 2 条 unknown；2 次 LQR 求解超时和 2 次 Gazebo 收尾异常均单列。原始 summary 的 all_passed 仅表示该 harness 的功能门槛，不能扩大解释为全部相对精度、性能和退出健康通过。

## 固定条件和原始导航结果

格内场景 SHA256 为 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。实际参数读回为 PID 预瞄 2.6 s、原增益；LQR Q=[1,1,100,10,10,10]、R=[20,20]、预瞄 2 s；LOS 距离 1 m、切换距离 5 m、原单步推进；obstacle_factor=0.25。没有外部 controller YAML、目标前覆盖或运行中调参，见 [manifest.json](manifest.json) 与 [verified_parameters.json](verified_parameters.json)。

艇体按 2.2×1.2 m 矩形，圆柱半径 1 m，保持 0.2 m 全艇扫掠裕度和 4 m 横向偏差上限。实际目标位置门槛读取 PID/LQR 配置 1.5/1.0 m，速度门槛 0.25 m/s。原始场景中 1.75 m 字段没有用于放宽到达判定。负扫掠下界或横向越界将自动停止当前及剩余矩阵，本轮没有触发。

每格按重复 01/02/03 给出。以下 RMSE 是原始 legacy_wall 采样口径，与后面的源时间对照分别保留。

| 控制器/场景 | 仿真用时/s | legacy 固定路径 RMSE/m | 最小全艇扫掠间隙/m | 全局搜索次数 |
| --- | --- | --- | --- | --- |
| PID 直航 | 20.8 / 20.6 / 20.8 | 0.058293 / 0.069667 / 0.066088 | 无受控障碍 | 1 / 1 / 1 |
| PID 转弯 | 20.5 / 20.4 / 20.4 | 0.103295 / 0.087048 / 0.088512 | 无受控障碍 | 1 / 1 / 1 |
| PID 圆柱 | 22.4 / 22.3 / 22.5 | 0.274166 / 0.207153 / 0.205217 | 0.663200 / 0.785057 / 0.810616 | 4 / 4 / 3 |
| LQR 直航 | 16.3 / 16.6 / 16.2 | 0.053618 / 0.090512 / 0.053831 | 无受控障碍 | 1 / 1 / 1 |
| LQR 转弯 | 18.0 / 17.8 / 17.6 | 0.941915 / 0.916339 / 0.976796 | 无受控障碍 | 1 / 1 / 1 |
| LQR 圆柱 | 18.4 / 18.5 / 18.4 | 0.406373 / 0.409840 / 0.406258 | 0.885133 / 0.867981 / 0.887848 | 4 / 4 / 4 |

## 故障、暂停与控制超时

目标更换例在 25.7 s 仿真时间到达新目标，legacy RMSE 为 0.316806 m，全局搜索 2 次。取消后的零命令观察延迟为 0.003667729 s；独立 odom 断流后的延迟为 0.424809112 s，两例均未观察到截止后非零命令。独立断流夹具按原协议隔离命名空间，在启动时使用 state_timeout=0.4 s、map_timeout=2 s、零命令截止 0.7 s；这些不是 18 条导航的在线参数变更。

以 goal_published 至 acquisition_finished 为窗口，20 条 Gazebo 试验中 11 条记录到 SIMULATION_PAUSED，共 30 条约 1 Hz 状态观察。LQR straight 01 和 PID single_obstacle 03 各有一条初始 WAITING_REFERENCE，相关 raw map/state/context 快照完整归档；没有观察到 NO_PATH。独立断流的 WAITING_REFERENCE/WAITING_STATE 与取消例 CANCELLED 按故障协议解释，不与正常导航混算。暂停样本没有删除。

LQR straight 01 与 LQR turn 01 的累计 control_result_timeout 均为 1。前者在 sim=16.0、monotonic=79800.931574934 观察到 CONTROL_ERROR，原因是 LQR solve exceeded its time budget，control_ms 最大值 80.325309 ms；后者计数首次被观察到时已经为 NAVIGATING，最大值 80.180329 ms。不能仅以约 1 Hz 的状态名推断没有超时。直航超时邻近 sample 在 monotonic=79800.949525998 记录左右推力均为零，下一 sample=79801.049768604 恢复非零，最终到达；状态更新间隔不等于精确错误持续时间。逐例累计计数、非正常状态及其下次观察边界见 [run_audit.json](run_audit.json)，全部状态原件见 [status_timeline.json](status_timeline.json)。

## 退出健康与环境一致性

20 条 Gazebo 试验中，18 条各 8 个 launch 进程 clean；以下两条各有 7 个 clean 和一个异常退出：

| 试验 | 进程 | 日志时序 | 原始日志 SHA256 |
| --- | --- | --- | --- |
| PID turn 02 | gzserver PID 114737 | 第 69 行 launch SIGINT，第 76 行发送 SIGINT，第 87–88 行 boost self-join 错误，第 93 行 exit -6 | `62329a6eb49a72a8fdb0c574c7060214b34f811de3e45b48e2a220cfa2dc915c` |
| LQR straight 01 | gzserver PID 115809 | 第 69 行 launch SIGINT，第 76 行发送 SIGINT，第 84–85 行 boost self-join 错误，第 93 行 exit -6 | `21772470119225581002496b1d4a00f32e765ef657904ebcecbfffafc51f3a53` |

完整日志保存在各试验子目录。该时序只证明异常出现在收尾日志段，不能将其记为正常退出或直接推定更深层根因。本轮所有 fake_localization、joint_state_publisher 进程均有 clean 记录。独立 odom 节点日志没有直接子进程返回码，退出健康为 unknown；没有错误文本不等于 clean。结束后只读核实没有 Gazebo、sim_env launch、harness 或固定 Python 安装节点残留。

before/after 的 path、commit、status、diff、artifacts 逐字段相同，源/安装/验收文件数量分别为 513/680/88；完整指纹的绝对路径和 SHA256 见 [provenance_check.json](provenance_check.json)。13 包构建日志、安装审计及安装 controller YAML 原样保留。只读 OS 采样器沿原 harness 生命周期运行，无重启；首样本早于首 goal 10.714042272 s，最终自然结束、623 个样本。OS 覆盖及资源数值由独立报告审核，本原始归档不提前将资源门槛判为通过。

## 冻结比较与来源

比较使用运行前固定的 source_time_trapezoid / otter_fake_localization frame 契约、0.5 s 源覆盖上限和 1.10 比例门槛，只与 baseline-grid-interior-001 相同控制器/场景/重复编号配对。全部 27 条双方正常导航源记录覆盖 valid。18 对中 7 对可比、6 对通过、1 对失败、11 对不可比。失败为 PID turn 01 源时间比例 1.10912971776；legacy 也是 6/7，但失败编号为 PID straight 03，比例 1.10054400653。两种原始结论都保留，不能互相替换通过项。C++ 缺乏合格 LQR 对照以及两条圆柱安全裕度失败造成的不可比项未被删掉。

控制/扫描耗时子项 18 条均通过；积压在 PID straight 03（expired 增量 [2,2,0,0]）和 LQR straight 03（[1,1,0]）出现连续过期，按冻结规则为 unknown。因此性能为 16 pass / 2 unknown，all_passed=false。求解超时事件与 P99 门槛分别报告，不把 P99 通过解释成零超时。

独立原始比较报告位于 `/home/usv/usv_migration/results/python-grid-endpoint-final-comparison-001/report.json`，SHA256 `c7257f7b756978dcfe6d2e0f2627b05908528bfd0a0bd48c80d6f11a9321539e`；该报告及图表由独立比较归档保存。本目录没有修改比较器或阈值。

完整原始运行目录为 `/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001`，位置、路径、局部轨迹、推力、地图与参数事件保持 WSL 原样。本地复制 36 份原件并逐文件核对 SHA256，包含 21 份逐例摘要、总摘要、manifest、场景、构建/安装审计、安装参数、两组失败快照、两份异常退出日志和独立 odom 日志。其余 JSON 为从原件提取的审计表；[source_index.json](source_index.json) 保存每个原始文件及同级指纹证据的绝对路径、字节数和 SHA256。旧失败、旧候选与本轮记录均保留，未追改判定。
