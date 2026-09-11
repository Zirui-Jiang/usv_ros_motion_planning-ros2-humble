# Python/C++ 导航结果对照

全证据总体判定：未通过或证据不完整。保留的逐次诊断：可比 7/18 对，通过 7 对，失败 0 对，不可比 11 对；故障用例通过 3/3。

Python 正常导航性能：通过 15、失败 0、证据不足 3；性能证据不足也不能总体通过。

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
| pid/straight | 3/3 | 1.05771 | 0.924444 | 通过 |
| pid/turn | 3/3 | 0.890497 | 0.895803 | 通过 |

| 控制器/场景/模式 | 度量 | 固定重复1 / 2 / 3 | 有值数 | 算术均值 | 标准差 ddof=1 | 最差 |
| --- | --- | --- | --- | --- | --- | --- |
| lqr/single_obstacle/python | source_time_trapezoid RMSE m | 0.413823 / 0.365843 / 0.4089 | 3 | 0.396189 | 0.0263955 | 0.413823 |
| lqr/single_obstacle/python | legacy_wall RMSE m | 0.414257 / 0.339852 / 0.412689 | 3 | 0.388933 | 0.0425126 | 0.414257 |
| lqr/single_obstacle/python | 到达时间 s | 18.4 / 18.5 / 18.3 | 3 | 18.4 | 0.1 | 18.5 |
| lqr/single_obstacle/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/single_obstacle/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/single_obstacle/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/python | source_time_trapezoid RMSE m | 0.0578321 / 0.0592442 / 0.0597298 | 3 | 0.0589354 | 0.000985848 | 0.0597298 |
| lqr/straight/python | legacy_wall RMSE m | 0.0476137 / 0.0550827 / 0.0543691 | 3 | 0.0523552 | 0.00412169 | 0.0550827 |
| lqr/straight/python | 到达时间 s | 16.1 / 16.1 / 16.3 | 3 | 16.1667 | 0.11547 | 16.3 |
| lqr/straight/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/straight/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/python | source_time_trapezoid RMSE m | 0.99736 / 0.988384 / 0.9922 | 3 | 0.992648 | 0.00450482 | 0.99736 |
| lqr/turn/python | legacy_wall RMSE m | 0.965314 / 1.08636 / 0.998392 | 3 | 1.01669 | 0.062561 | 1.08636 |
| lqr/turn/python | 到达时间 s | 17.9 / 17.9 / 17.8 | 3 | 17.8667 | 0.057735 | 17.9 |
| lqr/turn/cpp | source_time_trapezoid RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/cpp | legacy_wall RMSE m | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| lqr/turn/cpp | 到达时间 s | 缺失/不可比 / 缺失/不可比 / 缺失/不可比 | 0 | 缺失/不可比 | 缺失/不可比 | 缺失/不可比 |
| pid/single_obstacle/python | source_time_trapezoid RMSE m | 0.202081 / 0.204752 / 0.203523 | 3 | 0.203452 | 0.00133728 | 0.204752 |
| pid/single_obstacle/python | legacy_wall RMSE m | 0.204577 / 0.195656 / 0.203761 | 3 | 0.201331 | 0.00493152 | 0.204577 |
| pid/single_obstacle/python | 到达时间 s | 22.5 / 22.5 / 22.7 | 3 | 22.5667 | 0.11547 | 22.7 |
| pid/single_obstacle/cpp | source_time_trapezoid RMSE m | 0.488124 / 0.449319 / 0.443372 | 3 | 0.460272 | 0.0243031 | 0.488124 |
| pid/single_obstacle/cpp | legacy_wall RMSE m | 0.473402 / 0.450566 / 0.441366 | 3 | 0.455111 | 0.0164942 | 0.473402 |
| pid/single_obstacle/cpp | 到达时间 s | 30.4 / 27.9 / 26.8 | 3 | 28.3667 | 1.84481 | 30.4 |
| pid/straight/python | source_time_trapezoid RMSE m | 0.0607667 / 0.05908 / 0.059906 | 3 | 0.0599176 | 0.000843391 | 0.0607667 |
| pid/straight/python | legacy_wall RMSE m | 0.0571737 / 0.0557643 / 0.0632276 | 3 | 0.0587218 | 0.00396521 | 0.0632276 |
| pid/straight/python | 到达时间 s | 20.9 / 20.7 / 20.8 | 3 | 20.8 | 0.1 | 20.9 |
| pid/straight/cpp | source_time_trapezoid RMSE m | 0.0564979 / 0.0560478 / 0.057399 | 3 | 0.0566483 | 0.000688005 | 0.057399 |
| pid/straight/cpp | legacy_wall RMSE m | 0.0550533 / 0.0644582 / 0.0600501 | 3 | 0.0598539 | 0.00470551 | 0.0644582 |
| pid/straight/cpp | 到达时间 s | 23.5 / 20.8 / 23.2 | 3 | 22.5 | 1.47986 | 23.5 |
| pid/turn/python | source_time_trapezoid RMSE m | 0.0828194 / 0.0963307 / 0.0779425 | 3 | 0.0856975 | 0.00952602 | 0.0963307 |
| pid/turn/python | legacy_wall RMSE m | 0.0854216 / 0.101163 / 0.0783623 | 3 | 0.0883155 | 0.0116723 | 0.101163 |
| pid/turn/python | 到达时间 s | 20.5 / 20.8 / 20.6 | 3 | 20.6333 | 0.152753 | 20.8 |
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
| pid | single_obstacle | 1 | 0.413995 | 0.740132 | 通过 |
| pid | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | straight | 1 | 1.07556 | 0.889362 | 通过 |
| pid | straight | 2 | 1.0541 | 0.995192 | 通过 |
| pid | straight | 3 | 1.04368 | 0.896552 | 通过 |
| pid | turn | 1 | 0.879017 | 0.90708 | 通过 |
| pid | turn | 2 | 0.834194 | 0.900433 | 通过 |
| pid | turn | 3 | 0.986477 | 0.880342 | 通过 |

## 跟踪度量与覆盖证据

本报告主度量：`source_time_trapezoid`。legacy_wall 保留原 summary 的墙钟循环采样值；source_time_trapezoid 对接收窗口内唯一源 odom 的 e² 作非均匀时间梯形近似。两者并列，旧结论不回写，比例阈值不变。
源时间窗口包含 goal 前最新 odom 和之后至 acquisition_finished 的全部 odom，以真实首尾源时刻积分；不删除启动、近终点或暂停前后状态，不外推边界。源间隔与首尾绝对时间偏移上限预先固定为0.5s。路径按接收顺序冻结首条非空结果，不按C++旧错误header排序。P95/最大值只描述唯一已观测状态，离散积分不保证连续真实误差。缺数据、时间回退或冲突均unknown，不回退墙钟值代替主判。

显式坐标系契约：`{'name': 'otter_fake_localization', 'status': 'explicit_project_assertion', 'scope': 'Only the audited otter fake-localization project: map->odom is identity; this does not establish identity for arbitrary recorded data.', 'path_frame': 'otter/map', 'odom_frames': ['otter/odom', 'otter/map'], 'child_frame': 'otter/base_link', 'verified_sources': [{'mode': 'cpp', 'git_commit': '632734ba31478bdb833e15949287b53ae9334cf3', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [89, 94], 'source_sha256': 'b3f0e7a47b1328787c9fba138b0ec04701b679e6951828ed4a653df176789a4e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 114.', 'launch_log': '/home/usv/usv_migration/results/baseline-grid-interior-001/cpp_pid_straight_01/launch.log', 'launch_log_line': 10, 'launch_log_sha256': '113d5b195135cd9366886196b22a4b8dc4ab7c30b5d7a967c57eab94d4d2bcc1', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}, {'mode': 'python', 'git_commit': 'e38183218fce7044e686a0f697ed13f456f49039', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [91, 96], 'source_sha256': '5f9f2bf08715742c1384c22c7c932ec2f3d0630ffbd16ad15103f48d54aaac5e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 116.', 'launch_log': '/home/usv/usv_migration/results/python-release-grid-001/python_pid_straight_01/launch.log', 'launch_log_line': 20, 'launch_log_sha256': 'b028adf5e279fbc28e68571b33e657608d3d57042923ffdb1f1bade31ec5b08f', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}], 'version_scope': 'These are the audited baseline/e381 implementations and logs. For later source versions the explicit caller assertion requires confirming this unchanged transform contract; matching frame strings alone is not proof.'}`。仅限已核查的本项目 otter fake localization，不能推广为任意map/odom恒等。

| 试验 | source状态/原因 | 主RMSE m | legacy RMSE m | source RMSE m | 源样本/跨度s | 最大源间隔s | 首/尾覆盖缺口s | 源误差P95/最大m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | valid:  | 0.0607667 | 0.0571737 | 0.0607667 | 210/20.9 | 0.1 | 0/0 | 0.123587/0.175922 |
| python_pid_straight_02 | valid:  | 0.05908 | 0.0557643 | 0.05908 | 207/20.6 | 0.1 | 0/0.1 | 0.121417/0.176161 |
| python_pid_straight_03 | valid:  | 0.059906 | 0.0632276 | 0.059906 | 209/20.8 | 0.1 | 4.44089e-16/0 | 0.122545/0.177004 |
| python_pid_turn_01 | valid:  | 0.0828194 | 0.0854216 | 0.0828194 | 205/20.4 | 0.1 | 0/0.1 | 0.148722/0.175552 |
| python_pid_turn_02 | valid:  | 0.0963307 | 0.101163 | 0.0963307 | 208/20.7 | 0.1 | 0/0.1 | 0.157286/0.175787 |
| python_pid_turn_03 | valid:  | 0.0779425 | 0.0783623 | 0.0779425 | 206/20.5 | 0.1 | 4.44089e-16/0.1 | 0.152613/0.176115 |
| python_pid_single_obstacle_01 | valid:  | 0.202081 | 0.204577 | 0.202081 | 226/22.5 | 0.1 | 0/3.55271e-15 | 0.459384/0.522075 |
| python_pid_single_obstacle_02 | valid:  | 0.204752 | 0.195656 | 0.204752 | 225/22.4 | 0.1 | 0/0.1 | 0.464849/0.528814 |
| python_pid_single_obstacle_03 | valid:  | 0.203523 | 0.203761 | 0.203523 | 228/22.7 | 0.1 | 0/0 | 0.424209/0.525839 |
| python_lqr_straight_01 | valid:  | 0.0578321 | 0.0476137 | 0.0578321 | 161/16 | 0.1 | 0/0.1 | 0.124552/0.176004 |
| python_lqr_straight_02 | valid:  | 0.0592442 | 0.0550827 | 0.0592442 | 162/16.1 | 0.1 | 0/0 | 0.125073/0.175784 |
| python_lqr_straight_03 | valid:  | 0.0597298 | 0.0543691 | 0.0597298 | 163/16.2 | 0.1 | 4.44089e-16/0.1 | 0.124767/0.176444 |
| python_lqr_turn_01 | valid:  | 0.99736 | 0.965314 | 0.99736 | 180/17.9 | 0.1 | 0/0 | 2.18025/2.24658 |
| python_lqr_turn_02 | valid:  | 0.988384 | 1.08636 | 0.988384 | 180/17.9 | 0.1 | 0/0 | 2.15446/2.22045 |
| python_lqr_turn_03 | valid:  | 0.9922 | 0.998392 | 0.9922 | 178/17.7 | 0.1 | 0/0.1 | 2.1673/2.22721 |
| python_lqr_single_obstacle_01 | valid:  | 0.413823 | 0.414257 | 0.413823 | 184/18.3 | 0.1 | 0/0.1 | 0.757284/0.80486 |
| python_lqr_single_obstacle_02 | valid:  | 0.365843 | 0.339852 | 0.365843 | 186/18.5 | 0.1 | 0/0 | 0.667633/0.724432 |
| python_lqr_single_obstacle_03 | valid:  | 0.4089 | 0.412689 | 0.4089 | 184/18.3 | 0.1 | 0/0 | 0.747756/0.797953 |
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
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0179 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 3/2/10.6914 |
| python_pid_straight_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.4409 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.4331 |
| python_pid_turn_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2869 |
| python_pid_turn_03 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 5/4/21.4076 |
| python_pid_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2851 |
| python_pid_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.4716 |
| python_pid_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.3475 |
| python_lqr_straight_01 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 5/4/21.2663 |
| python_lqr_straight_02 | 通过 | 通过 | 通过 | 通过 | 3/2/10.6447 |
| python_lqr_straight_03 | 通过 | 通过 | 通过 | 通过 | 3/2/10.7167 |
| python_lqr_turn_01 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0338 |
| python_lqr_turn_02 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 4/3/16.0378 |
| python_lqr_turn_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.9569 |
| python_lqr_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 4/3/16.0962 |
| python_lqr_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 4/3/16.2155 |
| python_lqr_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 3/2/10.4452 |

各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。

## 重复试验汇总

统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。

| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |
| --- | --- | --- | --- | --- | --- | --- |
| cpp/pid/single_obstacle | 3/3/1 | 0.460272/0.488124 | 28.3667/30.4 | 0.0402385 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 49.883 |
| cpp/pid/straight | 3/3/3 | 0.0566483/0.057399 | 22.5/23.5 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 6.4111 |
| cpp/pid/turn | 3/3/3 | 0.0962356/0.115478 | 23.0333/23.4 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 22.9784 |
| python/lqr/single_obstacle | 3/3/3 | 0.396189/0.413823 | 18.4/18.5 | 0.856415 | 32.3623/41.8182；10.6063/10.9227 | 15.6573 |
| python/lqr/straight | 3/3/3 | 0.0589354/0.0597298 | 16.1667/16.3 | 缺失/不可比 | 31.5519/36.0093；10.4609/11.0055 | 4.23749 |
| python/lqr/turn | 3/3/3 | 0.992648/0.99736 | 17.8667/17.9 | 缺失/不可比 | 28.7752/30.4681；10.904/11.4581 | 9.6185 |
| python/pid/single_obstacle | 3/3/3 | 0.203452/0.204752 | 22.5667/22.7 | 0.7539 | 5.14304/5.84011；10.8846/12.519 | 46.6129 |
| python/pid/straight/cancel | 1/1/1 | 0.106998/0.106998 | 5.5/5.5 | 缺失/不可比 | 4.76814/4.76814；9.58273/9.58273 | 4.20896 |
| python/pid/straight/goal_change | 1/1/1 | 缺失/不可比/缺失/不可比 | 26/26 | 缺失/不可比 | 4.22909/4.22909；9.34832/9.34832 | 61.3296 |
| python/pid/straight | 3/3/3 | 0.0599176/0.0607667 | 20.8/20.9 | 缺失/不可比 | 4.41242/4.48538；11.3206/11.7342 | 10.8427 |
| python/pid/turn | 3/3/3 | 0.0856975/0.0963307 | 20.6333/20.8 | 缺失/不可比 | 4.34312/4.90194；10.1225/10.8293 | 43.8545 |
| python/None/None/odom_dropout | 1/1/1 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 7.70147/7.70147；8.40081/8.40081 | 3.3641 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 4.29358/70704 | 41.1019/68248 | 35.8044/105.429 | 5.7348/5.12355/0 |
| python_pid_straight_02 | 4.35887/71208 | 43.8784/69284 | 45.9151/104.699 | 5.59132/5.00646/0 |
| python_pid_straight_03 | 3.9785/71176 | 34.0587/69384 | 29.8564/103.769 | 5.80097/5.27114/0 |
| python_pid_turn_01 | 3.90712/71100 | 33.7982/68420 | 31.7172/104.093 | 18.885/20.9004/0 |
| python_pid_turn_02 | 3.70117/71252 | 32.1013/69096 | 26.1625/102.002 | 19.1791/20.7916/0 |
| python_pid_turn_03 | 3.52634/71088 | 28.7101/69360 | 27.0937/106.865 | 23.4053/28.4022/0 |
| python_pid_single_obstacle_01 | 3.77743/71384 | 31.8886/69236 | 27.3688/107.402 | 25.2794/24.457/0 |
| python_pid_single_obstacle_02 | 3.78014/71104 | 31.6689/69312 | 26.0804/105.331 | 24.3971/24.479/0 |
| python_pid_single_obstacle_03 | 3.81862/71116 | 31.7449/69692 | 25.5362/105.853 | 21.3839/19.8422/0 |
| python_lqr_straight_01 | 7.84686/71164 | 26.4762/69908 | 27.4902/107.872 | 2.38153/2.20928/0 |
| python_lqr_straight_02 | 9.72044/70972 | 36.2863/68504 | 36.7391/106.951 | 2.11293/1.98362/0 |
| python_lqr_straight_03 | 9.50173/71548 | 35.1558/69504 | 28.459/107.168 | 2.08999/1.93512/0 |
| python_lqr_turn_01 | 8.7767/71600 | 32.0014/68932 | 26.1467/102.757 | 6.25135/3.91794/0 |
| python_lqr_turn_02 | 8.65144/71676 | 26.7692/69472 | 33.3219/105.131 | 4.3372/6.71495/0 |
| python_lqr_turn_03 | 9.62633/71464 | 35.3437/69852 | 27.0951/103.376 | 4.16616/3.46789/0 |
| python_lqr_single_obstacle_01 | 8.9562/71336 | 30.573/69576 | 26.4929/107.42 | 7.75679/5.55699/0 |
| python_lqr_single_obstacle_02 | 7.24216/71236 | 27.0691/69132 | 25.0703/104.106 | 14.7569/6.06388/0 |
| python_lqr_single_obstacle_03 | 9.22702/72024 | 34.606/69408 | 28.3274/104.154 | 7.44695/5.39033/0 |
| python_pid_goal_change_01 | 3.93759/70680 | 33.7996/69236 | 25.1357/103.59 | 30.1668/31.1628/0 |
| python_pid_cancel_01 | 3.53973/71152 | 缺失/不可比/68224 | 24.9461/119.198 | 2.26334/1.94562/0 |
| python_pid_odom_dropout | 4.92135/63528 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 1.3641/2/0 |
| cpp_pid_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.13305/2.92195/0 |
| cpp_pid_straight_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.34128/3.56666/0 |
| cpp_pid_straight_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.26584/3.00452/0 |
| cpp_pid_turn_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 10.1061/9.97386/0 |
| cpp_pid_turn_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 11.9619/11.9872/0 |
| cpp_pid_turn_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 12.4533/12.4529/0 |
| cpp_pid_single_obstacle_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 27.9832/26.9739/0 |
| cpp_pid_single_obstacle_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 23.5353/22.9854/0 |
| cpp_pid_single_obstacle_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 24.0448/24.1265/0 |

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 346, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 348, "processed": 346, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 256, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 256, "processed": 256, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 334, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 335, "processed": 334, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 332, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 334, "processed": 332, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_02` 建图最后观测计数：`{"integrated_scans": 333, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 335, "processed": 333, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_03` 建图最后观测计数：`{"integrated_scans": 306, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 310, "processed": 306, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 416, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 418, "processed": 416, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 419, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 421, "processed": 419, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 417, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 417, "processed": 417, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_01` 建图最后观测计数：`{"integrated_scans": 276, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 281, "processed": 276, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_02` 建图最后观测计数：`{"integrated_scans": 210, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 211, "processed": 210, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_03` 建图最后观测计数：`{"integrated_scans": 211, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 212, "processed": 211, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_01` 建图最后观测计数：`{"integrated_scans": 258, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 260, "processed": 258, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_02` 建图最后观测计数：`{"integrated_scans": 225, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 229, "processed": 225, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_03` 建图最后观测计数：`{"integrated_scans": 290, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 291, "processed": 290, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 325, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 327, "processed": 325, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 301, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 304, "processed": 301, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 286, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 287, "processed": 286, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_goal_change_01` 建图最后观测计数：`{"integrated_scans": 371, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 374, "processed": 371, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_cancel_01` 建图最后观测计数：`{"integrated_scans": 53, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 53, "processed": 53, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-clean-release-matrix-001`：角色 python，run_finished=True，选中 21 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-grid-interior-001`：角色 cpp，run_finished=True，选中 9 条，未观测计划 0 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
