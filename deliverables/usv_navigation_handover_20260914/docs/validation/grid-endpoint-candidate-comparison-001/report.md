# Python/C++ 导航结果对照

总体判定：未通过或证据不完整。可比 5/9 对，通过 5 对，失败 0 对，不可比 4 对；故障用例通过 0/0。

Python 正常导航性能：通过 5、失败 0、证据不足 0；性能证据不足也不能总体通过。

仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ 1.10；零基线比例不定义，按绝对容差 1e-06 判断。

## 每次配对

| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |
| --- | --- | --- | --- | --- | --- |
| pid | single_obstacle | 1 | 0.405596 | 0.740132 | 通过 |
| pid | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | straight | 1 | 1.05789 | 0.889362 | 通过 |
| pid | straight | 2 | 1.09631 | 1.00962 | 通过 |
| pid | straight | 3 | 1.03586 | 0.887931 | 通过 |
| pid | turn | 1 | 0.778006 | 0.90708 | 通过 |
| pid | turn | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing |
| pid | turn | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing |

## 跟踪度量与覆盖证据

本报告主度量：`source_time_trapezoid`。legacy_wall 保留原 summary 的墙钟循环采样值；source_time_trapezoid 对接收窗口内唯一源 odom 的 e² 作非均匀时间梯形近似。两者并列，旧结论不回写，比例阈值不变。
源时间窗口包含 goal 前最新 odom 和之后至 acquisition_finished 的全部 odom，以真实首尾源时刻积分；不删除启动、近终点或暂停前后状态，不外推边界。源间隔与首尾绝对时间偏移上限预先固定为0.5s。路径按接收顺序冻结首条非空结果，不按C++旧错误header排序。P95/最大值只描述唯一已观测状态，离散积分不保证连续真实误差。缺数据、时间回退或冲突均unknown，不回退墙钟值代替主判。

显式坐标系契约：`{'name': 'otter_fake_localization', 'status': 'explicit_project_assertion', 'scope': 'Only the audited otter fake-localization project: map->odom is identity; this does not establish identity for arbitrary recorded data.', 'path_frame': 'otter/map', 'odom_frames': ['otter/odom', 'otter/map'], 'child_frame': 'otter/base_link', 'verified_sources': [{'mode': 'cpp', 'git_commit': '632734ba31478bdb833e15949287b53ae9334cf3', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [89, 94], 'source_sha256': 'b3f0e7a47b1328787c9fba138b0ec04701b679e6951828ed4a653df176789a4e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 114.', 'launch_log': '/home/usv/usv_migration/results/baseline-grid-interior-001/cpp_pid_straight_01/launch.log', 'launch_log_line': 10, 'launch_log_sha256': '113d5b195135cd9366886196b22a4b8dc4ab7c30b5d7a967c57eab94d4d2bcc1', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}, {'mode': 'python', 'git_commit': 'e38183218fce7044e686a0f697ed13f456f49039', 'source': 'src/sim_env/src/fake_localization_node.cpp', 'source_lines': [91, 96], 'source_sha256': '5f9f2bf08715742c1384c22c7c932ec2f3d0630ffbd16ad15103f48d54aaac5e', 'identity_evidence': 'Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 116.', 'launch_log': '/home/usv/usv_migration/results/python-release-grid-001/python_pid_straight_01/launch.log', 'launch_log_line': 20, 'launch_log_sha256': 'b028adf5e279fbc28e68571b33e657608d3d57042923ffdb1f1bade31ec5b08f', 'logged_frame_chain': ['otter/map', 'otter/odom', 'otter/base_link']}], 'version_scope': 'These are the audited baseline/e381 implementations and logs. For later source versions the explicit caller assertion requires confirming this unchanged transform contract; matching frame strings alone is not proof.'}`。仅限已核查的本项目 otter fake localization，不能推广为任意map/odom恒等。

| 试验 | source状态/原因 | 主RMSE m | legacy RMSE m | source RMSE m | 源样本/跨度s | 最大源间隔s | 首/尾覆盖缺口s | 源误差P95/最大m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | valid:  | 0.0597684 | 0.0574742 | 0.0597684 | 210/20.9 | 0.1 | 4.44089e-16/3.55271e-15 | 0.122597/0.176436 |
| python_pid_straight_02 | valid:  | 0.0614458 | 0.0860043 | 0.0614458 | 211/21 | 0.1 | 4.44089e-16/0 | 0.125208/0.176409 |
| python_pid_straight_03 | valid:  | 0.0594573 | 0.0651461 | 0.0594573 | 207/20.6 | 0.1 | 4.44089e-16/0 | 0.122936/0.176336 |
| python_pid_turn_01 | valid:  | 0.0733022 | 0.0692974 | 0.0733022 | 205/20.4 | 0.1 | 4.44089e-16/0.1 | 0.149057/0.176422 |
| python_pid_single_obstacle_01 | valid:  | 0.197981 | 0.191175 | 0.197981 | 226/22.5 | 0.1 | 8.88178e-16/0 | 0.460984/0.516941 |
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
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.6492 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2956 |
| python_pid_straight_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.4004 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.4486 |
| python_pid_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.2578 |

各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。

## 重复试验汇总

统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。

| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |
| --- | --- | --- | --- | --- | --- | --- |
| cpp/pid/single_obstacle | 3/3/1 | 0.460272/0.488124 | 28.3667/30.4 | 0.0402385 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 49.883 |
| cpp/pid/straight | 3/3/3 | 0.0566483/0.057399 | 22.5/23.5 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 6.4111 |
| cpp/pid/turn | 3/3/3 | 0.0962356/0.115478 | 23.0333/23.4 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 22.9784 |
| python/pid/single_obstacle | 1/1/1 | 0.197981/0.197981 | 22.5/22.5 | 0.688245 | 5.37569/5.37569；13.2746/13.2746 | 55.4771 |
| python/pid/straight | 3/3/3 | 0.0602238/0.0614458 | 20.8333/21 | 缺失/不可比 | 4.96674/5.53954；11.2638/12.0237 | 14.5758 |
| python/pid/turn | 1/1/1 | 0.0733022/0.0733022 | 20.5/20.5 | 缺失/不可比 | 4.38059/4.38059；10.9275/10.9275 | 42.1334 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 3.96103/70704 | 37.3615/69352 | 34.8676/105.832 | 5.50415/4.92168/0 |
| python_pid_straight_02 | 3.96215/71084 | 34.199/69760 | 34.299/107.925 | 11.9961/10.11/0 |
| python_pid_straight_03 | 4.04358/70636 | 36.2955/69232 | 32.347/104.096 | 5.96986/5.22557/0 |
| python_pid_turn_01 | 4.16349/71068 | 37.0468/68064 | 36.1276/103.126 | 19.8429/22.2904/0 |
| python_pid_single_obstacle_01 | 4.22924/71484 | 35.366/69200 | 36.9474/107.464 | 28.0718/27.4053/0 |
| cpp_pid_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.13305/2.92195/0 |
| cpp_pid_straight_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.34128/3.56666/0 |
| cpp_pid_straight_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.26584/3.00452/0 |
| cpp_pid_turn_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 10.1061/9.97386/0 |
| cpp_pid_turn_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 11.9619/11.9872/0 |
| cpp_pid_turn_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 12.4533/12.4529/0 |
| cpp_pid_single_obstacle_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 27.9832/26.9739/0 |
| cpp_pid_single_obstacle_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 23.5353/22.9854/0 |
| cpp_pid_single_obstacle_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 24.0448/24.1265/0 |

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 275, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 276, "processed": 275, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 306, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 310, "processed": 306, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 343, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 344, "processed": 343, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 335, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 336, "processed": 335, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 414, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 414, "processed": 414, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-grid-endpoint-straight-candidate-001`：角色 python，run_finished=True，选中 3 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/python-grid-endpoint-turn-candidate-001`：角色 python，run_finished=True，选中 1 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/python-grid-endpoint-obstacle-candidate-001`：角色 python，run_finished=True，选中 1 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-grid-interior-001`：角色 cpp，run_finished=True，选中 9 条，未观测计划 0 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
