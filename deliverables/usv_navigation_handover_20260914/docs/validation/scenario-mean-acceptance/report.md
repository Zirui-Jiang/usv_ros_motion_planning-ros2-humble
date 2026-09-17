# Python/C++ 导航结果对照

全证据总体判定：未通过或证据不完整。保留的逐次诊断：可比 7/18 对，通过 6 对，失败 1 对，不可比 11 对；故障用例通过 3/3。

Python 正常导航性能：通过 16、失败 0、证据不足 2；性能证据不足也不能总体通过。

仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ 1.10；零基线比例不定义，按绝对容差 1e-06 判断。

## 显式场景三次均值验收（scenario_mean）

主度量 `source_time_trapezoid`：适用的完整三次组通过 2/2，失败 0 组，不可比 4/6 组。适用组通过不等于所有组通过，也不替代独立功能与性能验收。
双方必须各有固定编号1、2、3，全部 completed/passed 且 source-time valid；不以有效子集或最好几次替代。分别取三次 RMSE 的算术均值和三次到达时间的算术均值，再比较均值之比；不是合并样本 RMSE，也不是三次比例的平均。下列标准差采用 ddof=1，最差取最大值；不可比组的已有数值仅作描述，不用于通过判定。

| 控制器/场景 | 双方观测数 Python/CPP | 均值RMSE比例 | 均值时间比例 | 场景判定/原因 |
| --- | --- | --- | --- | --- |
| lqr/single_obstacle | 3/0 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_requires_exact_repeats_1_2_3; cpp_repeat_1_missing; cpp_repeat_2_missing; cpp_repeat_3_missing |
| lqr/straight | 3/0 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_requires_exact_repeats_1_2_3; cpp_repeat_1_missing; cpp_repeat_2_missing; cpp_repeat_3_missing |
| lqr/turn | 3/0 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_requires_exact_repeats_1_2_3; cpp_repeat_1_missing; cpp_repeat_2_missing; cpp_repeat_3_missing |
| pid/single_obstacle | 3/3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_repeat_2_not_completed_and_passed; cpp_repeat_3_not_completed_and_passed |
| pid/straight | 3/3 | 1.05306 | 0.921481 | 通过 |
| pid/turn | 3/3 | 0.928758 | 0.88712 | 通过 |

| 控制器/场景/模式 | 度量 | 固定重复1 / 2 / 3 | 有值数 | 算术均值 | 标准差 ddof=1 | 最差 |
| --- | --- | --- | --- | --- | --- | --- |
| lqr/single_obstacle/python | source_time_trapezoid RMSE m | 0.401826 / 0.408997 / 0.404939 | 3 | 0.405254 | 0.00359584 | 0.408997 |
| lqr/single_obstacle/python | legacy_wall RMSE m | 0.406373 / 0.40984 / 0.406258 | 3 | 0.40749 | 0.00203603 | 0.40984 |
| lqr/single_obstacle/python | 到达时间 s | 18.4 / 18.5 / 18.4 | 3 | 18.4333 | 0.057735 | 18.5 |
| lqr/single_obstacle/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/single_obstacle/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/single_obstacle/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/python | source_time_trapezoid RMSE m | 0.0605255 / 0.0621486 / 0.0596374 | 3 | 0.0607705 | 0.00127341 | 0.0621486 |
| lqr/straight/python | legacy_wall RMSE m | 0.0536185 / 0.0905115 / 0.0538308 | 3 | 0.0659869 | 0.0212392 | 0.0905115 |
| lqr/straight/python | 到达时间 s | 16.3 / 16.6 / 16.2 | 3 | 16.3667 | 0.208167 | 16.6 |
| lqr/straight/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/python | source_time_trapezoid RMSE m | 0.993474 / 0.996412 / 0.98571 | 3 | 0.991865 | 0.00552917 | 0.996412 |
| lqr/turn/python | legacy_wall RMSE m | 0.941915 / 0.916339 / 0.976796 | 3 | 0.945017 | 0.0303476 | 0.976796 |
| lqr/turn/python | 到达时间 s | 18 / 17.8 / 17.6 | 3 | 17.8 | 0.2 | 18 |
| lqr/turn/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| pid/single_obstacle/python | source_time_trapezoid RMSE m | 0.200898 / 0.207855 / 0.207458 | 3 | 0.205404 | 0.00390671 | 0.207855 |
| pid/single_obstacle/python | legacy_wall RMSE m | 0.274166 / 0.207153 / 0.205217 | 3 | 0.228845 | 0.0392603 | 0.274166 |
| pid/single_obstacle/python | 到达时间 s | 22.4 / 22.3 / 22.5 | 3 | 22.4 | 0.1 | 22.5 |
| pid/single_obstacle/cpp | source_time_trapezoid RMSE m | 0.488124 / 0.449319 / 0.443372 | 3 | 0.460272 | 0.0243031 | 0.488124 |
| pid/single_obstacle/cpp | legacy_wall RMSE m | 0.473402 / 0.450566 / 0.441366 | 3 | 0.455111 | 0.0164942 | 0.473402 |
| pid/single_obstacle/cpp | 到达时间 s | 30.4 / 27.9 / 26.8 | 3 | 28.3667 | 1.84481 | 30.4 |
| pid/straight/python | source_time_trapezoid RMSE m | 0.0607553 / 0.0569399 / 0.0612665 | 3 | 0.0596539 | 0.00236425 | 0.0612665 |
| pid/straight/python | legacy_wall RMSE m | 0.058293 / 0.0696672 / 0.0660878 | 3 | 0.0646826 | 0.00581584 | 0.0696672 |
| pid/straight/python | 到达时间 s | 20.8 / 20.6 / 20.8 | 3 | 20.7333 | 0.11547 | 20.8 |
| pid/straight/cpp | source_time_trapezoid RMSE m | 0.0564979 / 0.0560478 / 0.057399 | 3 | 0.0566483 | 0.000688005 | 0.057399 |
| pid/straight/cpp | legacy_wall RMSE m | 0.0550533 / 0.0644582 / 0.0600501 | 3 | 0.0598539 | 0.00470551 | 0.0644582 |
| pid/straight/cpp | 到达时间 s | 23.5 / 20.8 / 23.2 | 3 | 22.5 | 1.47986 | 23.5 |
| pid/turn/python | source_time_trapezoid RMSE m | 0.1045 / 0.0828963 / 0.0807421 | 3 | 0.0893795 | 0.0131391 | 0.1045 |
| pid/turn/python | legacy_wall RMSE m | 0.103295 / 0.0870484 / 0.0885124 | 3 | 0.0929519 | 0.0089872 | 0.103295 |
| pid/turn/python | 到达时间 s | 20.5 / 20.4 / 20.4 | 3 | 20.4333 | 0.057735 | 20.5 |
| pid/turn/cpp | source_time_trapezoid RMSE m | 0.0942182 / 0.115478 / 0.0790109 | 3 | 0.0962356 | 0.0183169 | 0.115478 |
| pid/turn/cpp | legacy_wall RMSE m | 0.106501 / 0.122277 / 0.0846425 | 3 | 0.104473 | 0.0188988 | 0.122277 |
| pid/turn/cpp | 到达时间 s | 22.6 / 23.1 / 23.4 | 3 | 23.0333 | 0.404145 | 23.4 |

## 每次配对

| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |
| --- | --- | --- | --- | --- | --- |
| lqr | single_obstacle | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | straight | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | straight | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | straight | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| pid | single_obstacle | 1 | 0.411573 | 0.736842 | 通过 |
| pid | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | straight | 1 | 1.07535 | 0.885106 | 通过 |
| pid | straight | 2 | 1.01592 | 0.990385 | 通过 |
| pid | straight | 3 | 1.06738 | 0.896552 | 通过 |
| pid | turn | 1 | 1.10913 | 0.90708 | 失败 |
| pid | turn | 2 | 0.717856 | 0.883117 | 通过 |
| pid | turn | 3 | 1.02191 | 0.871795 | 通过 |

## 跟踪度量与覆盖证据

本报告主度量：`source_time_trapezoid`。legacy_wall 保留原 summary 的墙钟循环采样值；source_time_trapezoid 对接收窗口内唯一源 odom 的 e² 作非均匀时间梯形近似。两者并列，旧结论不回写，比例阈值不变。
源时间窗口包含 goal 前最新 odom 和之后至 acquisition_finished 的全部 odom，以真实首尾源时刻积分；不删除启动、近终点或暂停前后状态，不外推边界。源间隔与首尾绝对时间偏移上限预先固定为0.5s。路径按接收顺序冻结首条非空结果，不按C++旧错误header排序。P95/最大值只描述唯一已观测状态，离散积分不保证连续真实误差。缺数据、时间回退或冲突均unknown，不回退墙钟值代替主判。

显式坐标系契约：`{'name': 'otter_fake_localization', 'status': 'explicit_project_assertion', 'scope': 'Only the audited otter fake-localization project: map->odom is identity; this does not establish identity for arbitrary recorded data.', 'path_frame': 'otter/map', 'odom_frames': ['otter/odom', 'otter/map'], 'child_frame': 'otter/base_link', 'verified_sources': [{'mode': 'cpp', 'git_commit': '632734ba31478bdb833e15949287b53ae9334cf3', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [89, 94], 'source_sha256': 'b3f0e7a47b1328787c9fba138b0ec04701b679e6951828ed4a653df176789a4e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 114.', 'launch_log': '/home/usv/usv_migration/results/baseline-grid-interior-001/cpp_pid_straight_01/launch.log', 'launch_log_line': 10, 'launch_log_sha256': '113d5b195135cd9366886196b22a4b8dc4ab7c30b5d7a967c57eab94d4d2bcc1', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}, {'mode': 'python', 'git_commit': 'e38183218fce7044e686a0f697ed13f456f49039', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [91, 96], 'source_sha256': '5f9f2bf08715742c1384c22c7c932ec2f3d0630ffbd16ad15103f48d54aaac5e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 116.', 'launch_log': '/home/usv/usv_migration/results/python-release-grid-001/python_pid_straight_01/launch.log', 'launch_log_line': 20, 'launch_log_sha256': 'b028adf5e279fbc28e68571b33e657608d3d57042923ffdb1f1bade31ec5b08f', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}], 'version_scope': 'These are the audited baseline/e381 implementations and logs. For later source versions the explicit caller assertion requires confirming this unchanged transform contract; matching frame strings alone is not proof.'}`。仅限已核查的本项目 otter fake localization，不能推广为任意map/odom恒等。

| 试验 | source状态/原因 | 主RMSE m | legacy RMSE m | source RMSE m | 源样本/跨度s | 最大源间隔s | 首/尾覆盖缺口s | 源误差P95/最大m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | valid:  | 0.0607553 | 0.058293 | 0.0607553 | 209/20.8 | 0.1 | 4.44089e-16/3.55271e-15 | 0.123135/0.176158 |
| python_pid_straight_02 | valid:  | 0.0569399 | 0.0696672 | 0.0569399 | 206/20.5 | 0.1 | 0/0.1 | 0.11943/0.175936 |
| python_pid_straight_03 | valid:  | 0.0612665 | 0.0660878 | 0.0612665 | 209/20.8 | 0.1 | 0/0 | 0.123355/0.175805 |
| python_pid_turn_01 | valid:  | 0.1045 | 0.103295 | 0.1045 | 205/20.4 | 0.1 | 4.44089e-16/0.1 | 0.176048/0.183052 |
| python_pid_turn_02 | valid:  | 0.0828963 | 0.0870484 | 0.0828963 | 205/20.4 | 0.1 | 0/3.55271e-15 | 0.14091/0.176267 |
| python_pid_turn_03 | valid:  | 0.0807421 | 0.0885124 | 0.0807421 | 204/20.3 | 0.1 | 0/0.1 | 0.14134/0.176028 |
| python_pid_single_obstacle_01 | valid:  | 0.200898 | 0.274166 | 0.200898 | 225/22.4 | 0.1 | 0/0 | 0.441255/0.528281 |
| python_pid_single_obstacle_02 | valid:  | 0.207855 | 0.207153 | 0.207855 | 224/22.3 | 0.1 | 8.88178e-16/0 | 0.466989/0.531614 |
| python_pid_single_obstacle_03 | valid:  | 0.207458 | 0.205217 | 0.207458 | 226/22.5 | 0.1 | 0/0 | 0.450663/0.528736 |
| python_lqr_straight_01 | valid:  | 0.0605255 | 0.0536185 | 0.0605255 | 163/16.2 | 0.1 | 0/0.1 | 0.128622/0.176077 |
| python_lqr_straight_02 | valid:  | 0.0621486 | 0.0905115 | 0.0621486 | 166/16.5 | 0.1 | 4.44089e-16/0.1 | 0.138734/0.176231 |
| python_lqr_straight_03 | valid:  | 0.0596374 | 0.0538308 | 0.0596374 | 163/16.2 | 0.1 | 4.44089e-16/0 | 0.125372/0.177111 |
| python_lqr_turn_01 | valid:  | 0.993474 | 0.941915 | 0.993474 | 180/17.9 | 0.1 | 4.44089e-16/0.1 | 2.16353/2.24032 |
| python_lqr_turn_02 | valid:  | 0.996412 | 0.916339 | 0.996412 | 179/17.8 | 0.1 | 4.44089e-16/0 | 2.18072/2.24041 |
| python_lqr_turn_03 | valid:  | 0.98571 | 0.976796 | 0.98571 | 177/17.6 | 0.1 | 0/0 | 2.14545/2.2063 |
| python_lqr_single_obstacle_01 | valid:  | 0.401826 | 0.406373 | 0.401826 | 184/18.3 | 0.1 | 8.88178e-16/0.1 | 0.736102/0.779821 |
| python_lqr_single_obstacle_02 | valid:  | 0.408997 | 0.40984 | 0.408997 | 185/18.4 | 0.1 | 8.88178e-16/0.1 | 0.746827/0.77883 |
| python_lqr_single_obstacle_03 | valid:  | 0.404939 | 0.406258 | 0.404939 | 184/18.3 | 0.1 | 8.88178e-16/0.1 | 0.745254/0.789015 |
| cpp_pid_straight_01 | valid:  | 0.0564979 | 0.0550533 | 0.0564979 | 235/23.4 | 0.1 | 4.44089e-16/0.1 | 0.119133/0.176016 |
| cpp_pid_straight_02 | valid:  | 0.0560478 | 0.0644582 | 0.0560478 | 209/20.8 | 0.1 | 0/0 | 0.120301/0.175996 |
| cpp_pid_straight_03 | valid:  | 0.057399 | 0.0600501 | 0.057399 | 233/23.2 | 0.1 | 0/3.55271e-15 | 0.120005/0.175802 |
| cpp_pid_turn_01 | valid:  | 0.0942182 | 0.106501 | 0.0942182 | 227/22.6 | 0.1 | 0/3.55271e-15 | 0.176875/0.193742 |
| cpp_pid_turn_02 | valid:  | 0.115478 | 0.122277 | 0.115478 | 231/23 | 0.1 | 0/0.1 | 0.204075/0.22474 |
| cpp_pid_turn_03 | valid:  | 0.0790109 | 0.0846425 | 0.0790109 | 235/23.4 | 0.1 | 0/3.55271e-15 | 0.168865/0.175841 |
| cpp_pid_single_obstacle_01 | valid:  | 0.488124 | 0.473402 | 0.488124 | 305/30.4 | 0.1 | 8.88178e-16/0 | 0.862469/0.921716 |
| cpp_pid_single_obstacle_02 | valid:  | 0.449319 | 0.450566 | 0.449319 | 280/27.9 | 0.1 | 0/0 | 0.778517/0.829262 |
| cpp_pid_single_obstacle_03 | valid:  | 0.443372 | 0.441366 | 0.443372 | 268/26.7 | 0.1 | 0/0.1 | 0.781362/0.827238 |

## Python 正常导航性能验收

窗口为第一次 goal_published 至 acquisition_finished；旧记录明确回退到 trial_finished，该末端可能包含统计落盘时间。状态中的耗时分位数来自节点计算历史，可能含启动样本，各窗口样本数不相加。控制 P99 必须小于100ms；有实际积分扫描的建图处理 P95 不得超过同一状态中扫描时间戳间隔的 P50。扫描周期仅作为预算，不充当算法耗时。C++ 无对应遥测不要求性能对等。

积压至少需要3次同一进程/时钟周期的有效快照、2个连续约5s间隔，并覆盖试验窗口首尾。连续两个间隔 overflow 增长判失败；单次溢出、采样缺口、停滞或计数不自洽标待审。处理持续进展、队列计数平衡、无溢出增长时，有明确队列回落的末端1～2扫描瞬态净增不单独否决；持续非减且净增仍待审。两次 queue>0 不能证明持续积压，偶发 TF 过期单列记录。结论仅覆盖观测窗口，不保证无限时长稳定。

新增 queue_observation_window 使用独立 post_drain_latency_v1 规则：每个5s后drain观测窗至少覆盖80%，最大观测间隔和末端新鲜度不超过配置的TF墙钟截止；已有积分进展和计数守恒之外，还要求完成扫描等待时间及有效pending年龄分布有界、持续非空时有最老身份轮换。实测空队列以empty_observations表示，不伪造年龄零样本。满足时允许固定单帧流水等待，无需一定看到队列清空；连续区间TF过期增长仅能待审，不能由剩余扫描年龄推断没有过期。旧记录仍沿用上述稀疏观测规则，缺失/混合新字段不退回宽松判定。后drain最大值不代表剔除过期项前或连续时域的峰值。

计数按字段版本校验：新字段queue_counters.invalid_consumed存在时，processed必须等于integrated_scans，invalid_consumed必须等于invalid_scans，accepted减去processed、invalid_consumed、expired、overflow等于pending；旧字段缺失时，processed必须等于integrated_scans加invalid_scans，守恒式不重复扣除无效扫描。入队前拒绝的invalid/duplicate不计入accepted。无reset历史时另查累计守恒，reset曾清空的队列缺少丢弃计数时仅查同epoch区间守恒。缺失、负数、版本混合或计数不一致均不当作零或通过，具体原因保存在sample_counter_issues及intervals。

| Python试验 | 总状态 | 控制耗时 | 建图耗时 | 积压 | 状态样本/完整间隔/跨度s |
| --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0635 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2946 |
| python_pid_straight_03 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 5/4/21.3047 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.3055 |
| python_pid_turn_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2726 |
| python_pid_turn_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.3127 |
| python_pid_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.264 |
| python_pid_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.3477 |
| python_pid_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0861 |
| python_lqr_straight_01 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0884 |
| python_lqr_straight_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.9721 |
| python_lqr_straight_03 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 4/3/15.9726 |
| python_lqr_turn_01 | 通过 | 通过 | 通过 | 通过 | 4/3/15.9721 |
| python_lqr_turn_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.3319 |
| python_lqr_turn_03 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0863 |
| python_lqr_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.7445 |
| python_lqr_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 3/2/10.6539 |
| python_lqr_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0488 |

各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。

## 重复试验汇总

统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。

| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |
| --- | --- | --- | --- | --- | --- | --- |
| cpp/pid/single_obstacle | 3/3/1 | 0.460272/0.488124 | 28.3667/30.4 | 0.0402385 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 49.883 |
| cpp/pid/straight | 3/3/3 | 0.0566483/0.057399 | 22.5/23.5 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 6.4111 |
| cpp/pid/turn | 3/3/3 | 0.0962356/0.115478 | 23.0333/23.4 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 22.9784 |
| python/lqr/single_obstacle | 3/3/3 | 0.405254/0.408997 | 18.4333/18.5 | 0.867981 | 28.4254/30.4882；13.6489/15.2278 | 12.6573 |
| python/lqr/straight | 3/3/3 | 0.0607705/0.0621486 | 16.3667/16.6 | 缺失/不可比 | 33.7183/33.9625；12.3117/13.3074 | 8.24179 |
| python/lqr/turn | 3/3/3 | 0.991865/0.996412 | 17.8/18 | 缺失/不可比 | 31.6152/36.2915；12.6194/14.7482 | 7.99365 |
| python/pid/single_obstacle | 3/3/3 | 0.205404/0.207855 | 22.4/22.5 | 0.6632 | 5.53677/5.58951；14.9254/15.375 | 50.0479 |
| python/pid/straight/cancel | 1/1/1 | 0.109768/0.109768 | 5.6/5.6 | 缺失/不可比 | 6.96979/6.96979；13.6556/13.6556 | 4.30275 |
| python/pid/straight/goal_change | 1/1/1 | 缺失/不可比/缺失/不可比 | 25.7/25.7 | 缺失/不可比 | 4.44462/4.44462；11.2489/11.2489 | 50.7336 |
| python/pid/straight | 3/3/3 | 0.0596539/0.0612665 | 20.7333/20.8 | 缺失/不可比 | 4.23702/4.67587；11.1946/14.0084 | 11.3866 |
| python/pid/turn | 3/3/3 | 0.0893795/0.1045 | 20.4333/20.5 | 缺失/不可比 | 5.20342/5.85203；12.4007/16.4223 | 39.2776 |
| python/None/None/odom_dropout | 1/1/1 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 6.91782/6.91782；9.37509/9.37509 | 3.36421 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 3.53203/71076 | 32.2071/69868 | 33.9885/103.381 | 5.52358/4.89731/0 |
| python_pid_straight_02 | 3.71467/70464 | 34.2881/69600 | 29.8486/102.653 | 5.6819/4.98325/0 |
| python_pid_straight_03 | 3.48962/71244 | 29.022/69544 | 29.2153/109.592 | 7.15405/5.91959/0 |
| python_pid_turn_01 | 3.94099/71248 | 35.2161/69760 | 38.1924/103.925 | 19.9865/20.7085/0 |
| python_pid_turn_02 | 3.63799/71216 | 31.5168/69872 | 27.771/101.497 | 18.4258/20.1724/0 |
| python_pid_turn_03 | 3.46814/71084 | 28.5754/69276 | 39.5988/100.724 | 18.5009/20.0387/0 |
| python_pid_single_obstacle_01 | 3.7928/71204 | 30.9772/69080 | 38.6708/102.31 | 28.3397/24.9811/0 |
| python_pid_single_obstacle_02 | 3.93335/70952 | 32.3262/68924 | 38.4613/100.471 | 25.5285/24.6309/0 |
| python_pid_single_obstacle_03 | 3.95659/71552 | 35.1892/68700 | 40.4746/103.714 | 24.0066/22.6569/0 |
| python_lqr_straight_01 | 10.73/71504 | 44.3639/69504 | 64.0184/106.185 | 2.25448/2.11447/0 |
| python_lqr_straight_02 | 8.41248/71272 | 27.4574/68208 | 30.5798/111.407 | 6.75349/6.06028/0 |
| python_lqr_straight_03 | 9.04561/71572 | 29.9077/69080 | 32.0111/145.656 | 3.76366/3.77898/0 |
| python_lqr_turn_01 | 9.19309/71264 | 32.568/69484 | 41.978/104.054 | 4.34107/3.90525/0 |
| python_lqr_turn_02 | 9.48978/71568 | 33.279/69648 | 45.8308/107.97 | 4.19755/3.83058/0 |
| python_lqr_turn_03 | 10.8153/71044 | 41.6442/69668 | 38.5855/105.428 | 4.13314/3.57336/0 |
| python_lqr_single_obstacle_01 | 9.8048/71384 | 39.5228/69340 | 41.0809/103.886 | 7.1835/5.09724/0 |
| python_lqr_single_obstacle_02 | 10.3783/71600 | 36.3864/68492 | 34.577/100.76 | 7.7123/5.42254/0 |
| python_lqr_single_obstacle_03 | 8.61/71708 | 34.964/68164 | 29.8166/105.076 | 7.39736/5.15892/0 |
| python_pid_goal_change_01 | 4.06245/70664 | 34.0435/69128 | 33.2556/103.423 | 25.2101/25.5235/0 |
| python_pid_cancel_01 | 3.92247/70988 | 缺失/不可比/68464 | 28.0846/114.921 | 2.2946/2.00815/0 |
| python_pid_odom_dropout | 5.00671/63296 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 1.36421/2/0 |
| cpp_pid_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.13305/2.92195/0 |
| cpp_pid_straight_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.34128/3.56666/0 |
| cpp_pid_straight_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.26584/3.00452/0 |
| cpp_pid_turn_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 10.1061/9.97386/0 |
| cpp_pid_turn_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 11.9619/11.9872/0 |
| cpp_pid_turn_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 12.4533/12.4529/0 |
| cpp_pid_single_obstacle_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 27.9832/26.9739/0 |
| cpp_pid_single_obstacle_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 23.5353/22.9854/0 |
| cpp_pid_single_obstacle_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 24.0448/24.1265/0 |

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 306, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 310, "processed": 306, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 337, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 339, "processed": 337, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 307, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 311, "processed": 307, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 336, "scan_overruns": 1, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 337, "processed": 336, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_02` 建图最后观测计数：`{"integrated_scans": 337, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 340, "processed": 337, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_03` 建图最后观测计数：`{"integrated_scans": 298, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 303, "processed": 298, "overflow": 0, "expired": 5, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 377, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 380, "processed": 377, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 379, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 382, "processed": 379, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 380, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 383, "processed": 380, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_01` 建图最后观测计数：`{"integrated_scans": 276, "scan_overruns": 1, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 277, "processed": 276, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_02` 建图最后观测计数：`{"integrated_scans": 216, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 221, "processed": 216, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_03` 建图最后观测计数：`{"integrated_scans": 222, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 224, "processed": 222, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_01` 建图最后观测计数：`{"integrated_scans": 258, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 260, "processed": 258, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_02` 建图最后观测计数：`{"integrated_scans": 303, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 307, "processed": 303, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_03` 建图最后观测计数：`{"integrated_scans": 285, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 285, "processed": 285, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 282, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 282, "processed": 282, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 285, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 286, "processed": 285, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 342, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 343, "processed": 342, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_goal_change_01` 建图最后观测计数：`{"integrated_scans": 385, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 388, "processed": 385, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_cancel_01` 建图最后观测计数：`{"integrated_scans": 50, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 51, "processed": 50, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001`：角色 python，run_finished=True，选中 21 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-grid-interior-001`：角色 cpp，run_finished=True，选中 9 条，未观测计划 0 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
