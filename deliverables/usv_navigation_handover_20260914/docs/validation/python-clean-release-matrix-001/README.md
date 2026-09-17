# 全新普通安装：正式 18+3 原始验收归档

固定提交 `dfb1fb7fcb77feb868ee87ac986c70d7c4cab537` 在全新工作区 `/home/usv/usv_migration/python-clean-final-001` 完成 18 条导航和 3 条故障测试，**功能结果 21/21 通过**。按采集前提交的 [场景均值协议](../scenario-mean-release-protocol.md)，两个具备完整合格基线的场景均值均通过，7 个有效逐次对照也全部通过；另外 4 个场景保持不可比。

验收边界仍保留：建图积压有 3 条 unknown，LQR 累计观察到 3 次求解超时，两次 gzserver 在 SIGINT 收尾后退出 -6，独立 odom 夹具的直接退出码未记录。不能将功能通过或有效场景通过解释为所有性能、全部比较范围和退出健康均通过。

## 固定安装和采集条件

本次使用清理后的独立普通 install，未复用旧工作区的安装。新工作区 13 包构建、439 项源测试、数值 parity 和定位边界检查的前置证据见 [新安装归档](../python-clean-final-001/README.md)。本目录的 build.log 与 install_audit.json 原件来自新工作区 `validation/`，不是旧 3afb25f 的构建记录。

默认参数保持 PID 预瞄 2.6 s、原六增益（k_d_course=0）；LQR Q=[1,1,100,10,10,10]、R=[20,20]、预瞄 2 s；LOS 距离 1 m / 切换 5 m / 单步推进；obstacle_factor=0.25。没有候选 YAML、目标前参数覆盖或在线调参。逐例节点读回保存在 [verified_parameters.json](verified_parameters.json)，启动矩阵见 [manifest.json](manifest.json)。

格内场景 SHA256 为 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。艇体平面矩形为 2.2×1.2 m、受控圆柱半径 1 m，保持全艇扫掠裕度至少 0.2 m、最大横向偏差 4 m。目标位置容差读取 PID/LQR 的 1.5/1.0 m，速度容差 0.25 m/s；没有使用场景中 1.75 m 的旧预备字段放宽判定。原负间隙/横偏自动停止、TTL 和超时门槛均未改变，本轮未触发安全停止。

harness PID 123115、starttime_ticks 8461018，ROS domain 182，session 41630 自然退出 0。before/after 的 path、commit、status、diff、artifacts 逐字段完全相同，源/安装/验收脚本文件数量为 **435/680/90**，完整指纹来源和 SHA 见 [provenance_check.json](provenance_check.json)。结束后只读核实没有 Gazebo、sim_env launch、harness 或该新安装节点残留。

## 源时间逐次、均值和最差结果

每条分别采用冻结的 source_time_trapezoid，随后对同一控制器/场景的三条结果取算术平均；没有拼接轨迹或按运行时长加权。标准差为样本标准差 ddof=1，最差值为最大值。下面顺序均为重复 01/02/03，全部组 n=3。

| 控制器/场景 | 源时间 RMSE 逐次/m | RMSE 均值±标准差/m | 最差 RMSE/m | 仿真用时逐次/s | 平均/最长用时/s |
| --- | --- | --- | ---: | --- | --- |
| PID 直航 | 0.060767 / 0.059080 / 0.059906 | 0.059918 ± 0.000843 | 0.060767 | 20.9 / 20.7 / 20.8 | 20.800 / 20.9 |
| PID 转弯 | 0.082819 / 0.096331 / 0.077942 | 0.085698 ± 0.009526 | 0.096331 | 20.5 / 20.8 / 20.6 | 20.633 / 20.8 |
| PID 圆柱 | 0.202081 / 0.204752 / 0.203523 | 0.203452 ± 0.001337 | 0.204752 | 22.5 / 22.5 / 22.7 | 22.567 / 22.7 |
| LQR 直航 | 0.057832 / 0.059244 / 0.059730 | 0.058935 ± 0.000986 | 0.059730 | 16.1 / 16.1 / 16.3 | 16.167 / 16.3 |
| LQR 转弯 | 0.997360 / 0.988384 / 0.992200 | 0.992648 ± 0.004505 | 0.997360 | 17.9 / 17.9 / 17.8 | 17.867 / 17.9 |
| LQR 圆柱 | 0.413823 / 0.365843 / 0.408900 | 0.396189 ± 0.026395 | 0.413823 | 18.4 / 18.5 / 18.3 | 18.400 / 18.5 |

对照固定为 baseline-grid-interior-001。PID 直航的平均 RMSE/平均用时比分别为 **1.05771220 / 0.92444444**；PID 转弯为 **0.89049724 / 0.89580318**，均不超过 1.10。CPP PID 圆柱 02/03 原安全裕度失败，因此整个圆柱场景均值不可比；原版 LQR 无完整有效基线，三个场景也不可比。没有删掉失败或缺失项、换重复编号、补零或推定相对提升。

源时间与 legacy_wall 的有效逐次配对均为 7/7，通过范围以外的 11 对保留不可比。完整精度、用时、标准差、最差值和并列 legacy 数值见 [scenario_mean_audit.json](scenario_mean_audit.json)。该表原样提取自独立冻结报告 `/home/usv/usv_migration/results/python-clean-release-comparison-001/report.json`，SHA256 `ee36898a8ad47b8cb337c578e8525529066db5f3b320ae2075b139a4d7f4ff6d`。旧 3afb25f 和各候选结果不混入本轮、不追改。

## 几何、故障和状态观察

PID 圆柱最小全艇扫掠间隙为 **0.753900 / 0.757024 / 0.847320 m**，LQR 为 **0.856415 / 0.873451 / 0.861326 m**，均超过 0.2 m。这里使用既定矩形艇体—圆柱的保守扫掠检测，不是仅以艇中心距离判定。直航/转弯各次全局搜索均为 1；PID 圆柱 4/5/3，LQR 圆柱 3/3/3。

目标更换在 26.0 s 仿真时间到达新目标，执行 2 次全局搜索。取消后的零命令观察延迟为 **0.002796115 s**，独立 odom 断流为 **0.424893369 s**，两例均没有观察到截止后非零命令。断流夹具沿用独立命名空间、启动 state_timeout=0.4 s / map_timeout=2 s、0.7 s 零命令截止；这些不是正式 18 条导航的在线参数修改。

按 goal_published 至 acquisition_finished 窗口，11 条 Gazebo 试验中共观察到 34 条约 1 Hz 的 SIMULATION_PAUSED 状态，全部保留。PID 圆柱 01、LQR 转弯 02/03、LQR 圆柱 03 各有一条初始 WAITING_REFERENCE，四组 latest-callback raw map/state/context 快照完整保存；它们不是同一时刻的原子 worker 输入。未观察到 NO_PATH。

LQR 直航 03、圆柱 01、圆柱 03 的累计 control_result_timeout 各为 1，共观察到 **3 次求解超时**，均最终恢复并到达。本批约 1 Hz 状态没有采到 CONTROL_ERROR，不能据状态名写成零超时。计数和全部状态保存在 [run_audit.json](run_audit.json)、[status_timeline.json](status_timeline.json)；状态首末或下一次观察间隔不等于精确状态持续时间。

## 节点性能、OS 覆盖和退出健康

18 条控制耗时及 18 条扫描处理耗时子项全部通过冻结门槛；积压为 **15 pass / 3 unknown**。unknown 为 PID turn 03 的过期增量 [0,2,2,0]、LQR straight 01 的 [0,0,1,3]、LQR turn 02 的 [1,3,0]，按原规则保留连续过期边界，不扩大 TTL、不删除暂停数据，也不以独立正常 TF 吞吐测试替代本轮 unknown。控制 P99 通过不代表未发生求解超时。

独立 OS 采样器 PID 123069 随原 harness 自然退出，记录 609 个样本，首样本早于首 goal **8.807595315 s**。18/18 导航窗口均覆盖，共 379 个进程快照、361 个完整 CPU 区间。样本 584 有一次未细化原因的 census 错误，位于 cancel 的 acquisition_finished 之后；独立 odom 没有 acquisition_finished，末样本早于 trial_finished 0.181475 s，尾部不声称完整覆盖。完整边界见 [独立 OS 报告](../python-clean-release-matrix-001-process-performance.md)，旁路原件 SHA256 `406488f94f1175caa5f36efef9c9a3a720b0d1d2cab87f04593743ed7b1678ac`。

20 条 Gazebo 试验中，18 条各 8 个 launch 进程 clean；以下两条各 7 个 clean、一个异常退出：

| 试验 | 异常进程 | SIGINT 与退出日志 | 完整日志 SHA256 |
| --- | --- | --- | --- |
| PID 圆柱 01 | gzserver PID 124411 | launch SIGINT 第 69 行、向 gzserver 发信号第 76 行、boost self-join 第 84–85 行、exit -6 第 93 行 | `3c1fc2b609d3652b92460c835160652cd918bb7743d8b898c7c7ff2fef109e11` |
| PID 圆柱 03 | gzserver PID 124819 | launch SIGINT 第 70 行、向 gzserver 发信号第 77 行、boost self-join 第 83–84 行、exit -6 第 94 行 | `a4ce005df0aecba195acbf7ea17ed0af4483b0a9472f94d224f9175d905a8855` |

日志顺序只说明异常出现在收尾阶段，不将其改写为正常退出或推断更深层根因。本轮所有 fake_localization 和 joint_state_publisher 都有 clean 记录。独立 odom 节点没有直接返回码，退出健康为 unknown；没有错误文本不能当作正常退出证据。

## 来源与可复查范围

正常 otter 启动的实际 frame 证据来自本轮 PID straight 01 的 launch.log 第 22 行：`otter/map → otter/odom → otter/base_link`，日志 SHA256 `8223d931726310d8b565f81018924ef45971a30c8b61fc4c2ced7f65ca67e90d`。新定位源码和二进制 SHA 分别为 `ec5f0d59cc35e6f76066add8cd0e3531d422ae3d7b6f5edfc6566bf800a36068`、`4c9c583ebbf9eeed0db5c05e41587ada7ce2443def653744be8afbda63a58378`，绝对路径及原始行见 [frame_receipt.json](frame_receipt.json)。这与前置随机命名空间定位 green 分别保留，适用范围限于本项目已核查的恒等 map→odom 契约。

完整原始目录为 `/home/usv/usv_migration/results/python-clean-release-matrix-001`。42 份原件复制后逐个核对 SHA，包含总/逐例摘要、manifest、固定场景、新构建和安装审计、安装参数、四组输入快照、两份异常退出日志和独立 odom 日志；其他 JSON 是从这些原件提取的审计表。完整位置、参考路径、局部轨迹、推力和地图仍保持 WSL 原样。[source_index.json](source_index.json) 索引绝对路径、字节数、SHA，原始摘要未重写。

本目录将静态/构建、功能、安全、有效基线均值、逐次与最差值、节点性能、OS 覆盖和退出健康分开呈现。独立报告的 all_passed=false 保留其不可比范围与性能 unknown；不会用一个总布尔值隐藏这些限制。
