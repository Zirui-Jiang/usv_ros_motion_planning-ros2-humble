# Python/C++ 导航结果对照

总体判定：未通过或证据不完整。可比 9/18 对，通过 8 对，失败 1 对，不可比 9 对；故障用例通过 3/3。

Python 正常导航性能：通过 17、失败 0、证据不足 1；性能证据不足也不能总体通过。

仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ 1.10；零基线比例不定义，按绝对容差 1e-06 判断。

## 每次配对

| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |
| --- | --- | --- | --- | --- | --- |
| lqr | single_obstacle | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | straight | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=interrupted, passed=False, manifest_mode=live, reason=user interrupted this live trial |
| lqr | straight | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | straight | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| lqr | turn | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_result_missing |
| pid | single_obstacle | 1 | 0.631255 | 0.854406 | 通过 |
| pid | single_obstacle | 2 | 0.561082 | 0.798561 | 通过 |
| pid | single_obstacle | 3 | 0.923766 | 0.766187 | 通过 |
| pid | straight | 1 | 0.817755 | 0.710037 | 通过 |
| pid | straight | 2 | 1.25672 | 0.826271 | 失败 |
| pid | straight | 3 | 0.832923 | 0.72119 | 通过 |
| pid | turn | 1 | 0.69264 | 0.902655 | 通过 |
| pid | turn | 2 | 0.838051 | 0.901786 | 通过 |
| pid | turn | 3 | 0.743237 | 0.907489 | 通过 |

## Python 正常导航性能验收

窗口为第一次 goal_published 至 acquisition_finished；旧记录明确回退到 trial_finished，该末端可能包含统计落盘时间。状态中的耗时分位数来自节点计算历史，可能含启动样本，各窗口样本数不相加。控制 P99 必须小于100ms；有实际积分扫描的建图处理 P95 不得超过同一状态中扫描时间戳间隔的 P50。扫描周期仅作为预算，不充当算法耗时。C++ 无对应遥测不要求性能对等。

积压至少需要3次同一进程/时钟周期的有效快照、2个连续约5s间隔，并覆盖试验窗口首尾。连续两个间隔 overflow 增长判失败；单次溢出、采样缺口、停滞或计数不自洽标待审。处理持续进展、队列计数平衡、无溢出增长时，有明确队列回落的末端1～2扫描瞬态净增不单独否决；持续非减且净增仍待审。两次 queue>0 不能证明持续积压，偶发 TF 过期单列记录。结论仅覆盖观测窗口，不保证无限时长稳定。

新增 queue_observation_window 使用独立 post_drain_latency_v1 规则：每个5s后drain观测窗至少覆盖80%，最大观测间隔和末端新鲜度不超过配置的TF墙钟截止；已有积分进展和计数守恒之外，还要求完成扫描等待时间及有效pending年龄分布有界、持续非空时有最老身份轮换。实测空队列以empty_observations表示，不伪造年龄零样本。满足时允许固定单帧流水等待，无需一定看到队列清空；连续区间TF过期增长仅能待审，不能由剩余扫描年龄推断没有过期。旧记录仍沿用上述稀疏观测规则，缺失/混合新字段不退回宽松判定。后drain最大值不代表剔除过期项前或连续时域的峰值。

| Python试验 | 总状态 | 控制耗时 | 建图耗时 | 积压 | 状态样本/完整间隔/跨度s |
| --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.4963 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7167 |
| python_pid_straight_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7326 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 5/4/20.9631 |
| python_pid_turn_02 | 通过 | 通过 | 通过 | 通过 | 5/4/20.9684 |
| python_pid_turn_03 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 5/4/20.9882 |
| python_pid_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.0058 |
| python_pid_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.6968 |
| python_pid_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 5/4/20.9154 |
| python_lqr_straight_01 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7357 |
| python_lqr_straight_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7401 |
| python_lqr_straight_03 | 通过 | 通过 | 通过 | 通过 | 3/2/10.4899 |
| python_lqr_turn_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.4496 |
| python_lqr_turn_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7399 |
| python_lqr_turn_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7623 |
| python_lqr_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.4943 |
| python_lqr_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7356 |
| python_lqr_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 3/2/10.5185 |

各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。

## 重复试验汇总

统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。

| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |
| --- | --- | --- | --- | --- | --- | --- |
| cpp/lqr/single_obstacle | 0/0/0 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 缺失/不可比 |
| cpp/lqr/straight | 1/0/0 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 缺失/不可比 |
| cpp/lqr/turn | 0/0/0 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 缺失/不可比 |
| cpp/pid/single_obstacle | 3/3/3 | 0.287502/0.307967 | 27.2333/27.8 | 0.275823 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 45.3255 |
| cpp/pid/straight | 3/3/3 | 0.159091/0.182465 | 25.8/26.9 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 12.7454 |
| cpp/pid/turn | 3/3/3 | 0.145933/0.151514 | 22.5667/22.7 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 22.4252 |
| python/lqr/single_obstacle | 3/3/3 | 0.2358/0.257309 | 17.0333/17.1 | 0.551747 | 25.3333/33.7993；12.2303/15.4604 | 13.383 |
| python/lqr/straight | 3/3/3 | 0.113943/0.116835 | 15.6667/15.7 | 缺失/不可比 | 21.1983/25.2924；9.50321/10.2173 | 6.56896 |
| python/lqr/turn | 3/3/3 | 1.018/1.16096 | 17.3667/17.6 | 缺失/不可比 | 24.7158/28.7398；10.1444/11.9342 | 10.8162 |
| python/pid/single_obstacle | 3/3/3 | 0.205233/0.28449 | 21.9333/22.3 | 0.304069 | 4.32919/4.49687；10.252/11.3411 | 54.046 |
| python/pid/straight/cancel | 1/1/1 | 0.206295/0.206295 | 5.6/5.6 | 缺失/不可比 | 6.91185/6.91185；9.80947/9.80947 | 4.86237 |
| python/pid/straight/goal_change | 1/1/1 | 0.28342/0.28342 | 25.7/25.7 | 缺失/不可比 | 4.91416/4.91416；12.7774/12.7774 | 54.3329 |
| python/pid/straight | 3/3/3 | 0.147625/0.150996 | 19.3333/19.5 | 缺失/不可比 | 4.03062/4.24054；9.76973/10.6439 | 12.8884 |
| python/pid/turn | 3/3/3 | 0.110381/0.118635 | 20.4/20.6 | 缺失/不可比 | 3.9769/4.07997；9.64889/10.3748 | 47.592 |
| python/None/None/odom_dropout | 1/1/1 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 5.99749/5.99749；7.74851/7.74851 | 3.35258 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 3.67911/71092 | 33.5066/68844 | 23.3708/106.937 | 6.84239/5.70589/0 |
| python_pid_straight_02 | 3.85306/70556 | 34.2837/69128 | 25.9753/102.304 | 7.37317/6.22175/0 |
| python_pid_straight_03 | 3.55632/70664 | 32.1214/69436 | 25.1099/103.951 | 6.95924/5.56286/0 |
| python_pid_turn_01 | 3.58689/71884 | 29.1106/69420 | 23.9348/102.557 | 25.7502/28.2784/0 |
| python_pid_turn_02 | 3.63064/71004 | 32.5369/69964 | 27.3262/105.254 | 21.3723/23.4447/0 |
| python_pid_turn_03 | 3.70763/71224 | 29.8861/69672 | 25.7658/104.27 | 19.9974/23.9328/0 |
| python_pid_single_obstacle_01 | 3.6972/70988 | 30.4641/69152 | 24.8842/104.466 | 23.4269/24.1781/0 |
| python_pid_single_obstacle_02 | 3.61739/71204 | 30.084/68556 | 24.1041/100.528 | 28.3231/30.5736/0 |
| python_pid_single_obstacle_03 | 3.48882/71460 | 28.532/69732 | 21.8225/107.891 | 27.7868/27.8496/0 |
| python_lqr_straight_01 | 7.70029/71328 | 31.0085/68500 | 24.0233/105.223 | 3.80226/3.41436/0 |
| python_lqr_straight_02 | 6.92006/71700 | 29.0451/69676 | 23.765/106.235 | 3.41601/3.02858/0 |
| python_lqr_straight_03 | 8.24946/71908 | 33.9043/69764 | 25.4365/105.304 | 3.30973/2.73593/0 |
| python_lqr_turn_01 | 8.18441/71604 | 31.8426/68536 | 26.0602/110.158 | 4.89796/4.05757/0 |
| python_lqr_turn_02 | 8.07614/71668 | 27.9386/69724 | 25.0723/129.259 | 10.108/4.2773/0 |
| python_lqr_turn_03 | 9.35969/72092 | 34.1692/68936 | 27.7736/102.896 | 4.88562/4.22212/0 |
| python_lqr_single_obstacle_01 | 7.90585/71448 | 29.4051/69948 | 24.559/106.03 | 5.04168/5.61746/0 |
| python_lqr_single_obstacle_02 | 8.17908/71596 | 32.0673/69704 | 74.5773/102.442 | 8.83891/9.01068/0 |
| python_lqr_single_obstacle_03 | 7.35441/71292 | 32.9129/69644 | 25.7698/106.374 | 6.54687/5.09343/0 |
| python_pid_goal_change_01 | 4.28187/71260 | 37.2206/69704 | 33.321/107.558 | 27.5157/26.8173/0 |
| python_pid_cancel_01 | 3.56029/70812 | 缺失/不可比/69060 | 22.7839/109.987 | 2.74134/2.12103/0 |
| python_pid_odom_dropout | 4.79693/63660 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 1.35258/2/0 |
| cpp_pid_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 8.2375/7.02682/0 |
| cpp_pid_straight_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.19725/3.69151/0 |
| cpp_pid_straight_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 8.69412/7.38913/0 |
| cpp_pid_turn_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 11.5334/11.5266/0 |
| cpp_pid_turn_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 12.1144/12.0343/0 |
| cpp_pid_turn_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 10.1678/9.89904/0 |
| cpp_pid_single_obstacle_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 27.3445/27.6508/0 |
| cpp_pid_single_obstacle_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 20.7824/20.0177/0 |
| cpp_pid_single_obstacle_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 20.665/19.516/0 |
| cpp_lqr_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.4531/4.87237/0 |

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 274, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 274, "processed": 274, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 286, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 287, "processed": 286, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 262, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 262, "processed": 262, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 316, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 320, "processed": 316, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_02` 建图最后观测计数：`{"integrated_scans": 340, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 341, "processed": 340, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_03` 建图最后观测计数：`{"integrated_scans": 317, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 320, "processed": 317, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 416, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 418, "processed": 416, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 340, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 342, "processed": 340, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 395, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 399, "processed": 395, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_straight_01` 建图最后观测计数：`{"integrated_scans": 261, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 262, "processed": 261, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_straight_02` 建图最后观测计数：`{"integrated_scans": 246, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 247, "processed": 246, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_straight_03` 建图最后观测计数：`{"integrated_scans": 207, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 209, "processed": 207, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_turn_01` 建图最后观测计数：`{"integrated_scans": 235, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 239, "processed": 235, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_turn_02` 建图最后观测计数：`{"integrated_scans": 238, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 241, "processed": 238, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_turn_03` 建图最后观测计数：`{"integrated_scans": 263, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 264, "processed": 263, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 265, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 266, "processed": 265, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 337, "scan_overruns": 5, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 339, "processed": 337, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 285, "scan_overruns": 1, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 286, "processed": 285, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_goal_change_01` 建图最后观测计数：`{"integrated_scans": 364, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 365, "processed": 364, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_cancel_01` 建图最后观测计数：`{"integrated_scans": 53, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 53, "processed": 53, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-final-matrix-001`：角色 python，run_finished=True，选中 21 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-matrix-002`：角色 cpp，run_finished=False，选中 10 条，未观测计划 8 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
