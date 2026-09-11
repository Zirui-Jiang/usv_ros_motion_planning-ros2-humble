# Python/C++ 导航结果对照

总体判定：未通过或证据不完整。可比 7/18 对，通过 6 对，失败 1 对，不可比 11 对；故障用例通过 3/3。

Python 正常导航性能：通过 17、失败 0、证据不足 1；性能证据不足也不能总体通过。

仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ 1.10；零基线比例不定义，按绝对容差 1e-06 判断。

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
| pid | single_obstacle | 1 | 0.412365 | 0.746711 | 通过 |
| pid | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：cpp_not_completed_and_passed: result=completed, passed=False, manifest_mode=live, reason=['footprint_clearance_limit_or_missing_observation'] |
| pid | straight | 1 | 1.57046 | 0.880851 | 失败 |
| pid | straight | 2 | 1.01663 | 1.00962 | 通过 |
| pid | straight | 3 | 1.01295 | 0.896552 | 通过 |
| pid | turn | 1 | 0.845846 | 0.911504 | 通过 |
| pid | turn | 2 | 0.694743 | 0.887446 | 通过 |
| pid | turn | 3 | 0.808252 | 0.884615 | 通过 |

## Python 正常导航性能验收

窗口为第一次 goal_published 至 acquisition_finished；旧记录明确回退到 trial_finished，该末端可能包含统计落盘时间。状态中的耗时分位数来自节点计算历史，可能含启动样本，各窗口样本数不相加。控制 P99 必须小于100ms；有实际积分扫描的建图处理 P95 不得超过同一状态中扫描时间戳间隔的 P50。扫描周期仅作为预算，不充当算法耗时。C++ 无对应遥测不要求性能对等。

积压至少需要3次同一进程/时钟周期的有效快照、2个连续约5s间隔，并覆盖试验窗口首尾。连续两个间隔 overflow 增长判失败；单次溢出、采样缺口、停滞或计数不自洽标待审。处理持续进展、队列计数平衡、无溢出增长时，有明确队列回落的末端1～2扫描瞬态净增不单独否决；持续非减且净增仍待审。两次 queue>0 不能证明持续积压，偶发 TF 过期单列记录。结论仅覆盖观测窗口，不保证无限时长稳定。

新增 queue_observation_window 使用独立 post_drain_latency_v1 规则：每个5s后drain观测窗至少覆盖80%，最大观测间隔和末端新鲜度不超过配置的TF墙钟截止；已有积分进展和计数守恒之外，还要求完成扫描等待时间及有效pending年龄分布有界、持续非空时有最老身份轮换。实测空队列以empty_observations表示，不伪造年龄零样本。满足时允许固定单帧流水等待，无需一定看到队列清空；连续区间TF过期增长仅能待审，不能由剩余扫描年龄推断没有过期。旧记录仍沿用上述稀疏观测规则，缺失/混合新字段不退回宽松判定。后drain最大值不代表剔除过期项前或连续时域的峰值。

计数按字段版本校验：新字段queue_counters.invalid_consumed存在时，processed必须等于integrated_scans，invalid_consumed必须等于invalid_scans，accepted减去processed、invalid_consumed、expired、overflow等于pending；旧字段缺失时，processed必须等于integrated_scans加invalid_scans，守恒式不重复扣除无效扫描。入队前拒绝的invalid/duplicate不计入accepted。无reset历史时另查累计守恒，reset曾清空的队列缺少丢弃计数时仅查同epoch区间守恒。缺失、负数、版本混合或计数不一致均不当作零或通过，具体原因保存在sample_counter_issues及intervals。

| Python试验 | 总状态 | 控制耗时 | 建图耗时 | 积压 | 状态样本/完整间隔/跨度s |
| --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.1318 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.8357 |
| python_pid_straight_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.0677 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.0857 |
| python_pid_turn_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.1072 |
| python_pid_turn_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.1457 |
| python_pid_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 5/4/21.1232 |
| python_pid_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 5/4/21.0486 |
| python_pid_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 5/4/21.1344 |
| python_lqr_straight_01 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 4/3/15.816 |
| python_lqr_straight_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.8842 |
| python_lqr_straight_03 | 通过 | 通过 | 通过 | 通过 | 3/2/10.6768 |
| python_lqr_turn_01 | 通过 | 通过 | 通过 | 通过 | 3/2/10.56 |
| python_lqr_turn_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7929 |
| python_lqr_turn_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.8304 |
| python_lqr_single_obstacle_01 | 通过 | 通过 | 通过 | 通过 | 4/3/15.826 |
| python_lqr_single_obstacle_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.8326 |
| python_lqr_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.818 |

各子项的原因、每个窗口分位值/预算/样本数、逐间隔计数差及 TF 过期数保存在 report.json 的 performance 中。

## 重复试验汇总

统计使用所有实际 completed 的试验，包括验收失败试验；passed 数量单列。均值/最大由有值记录计算，不用零补缺失。

| 模式/控制器/场景 | 观测/完成/通过 | RMSE 均值/最大 m | 时间均值/最大 s | 最小间隙 m | 控制、规划P99均值/最大 ms | 推力总变差均值 |
| --- | --- | --- | --- | --- | --- | --- |
| cpp/pid/single_obstacle | 3/3/1 | 0.455111/0.473402 | 28.3667/30.4 | 0.0402385 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 49.883 |
| cpp/pid/straight | 3/3/3 | 0.0598539/0.0644582 | 22.5/23.5 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 6.4111 |
| cpp/pid/turn | 3/3/3 | 0.104473/0.122277 | 23.0333/23.4 | 缺失/不可比 | 缺失/不可比/缺失/不可比；缺失/不可比/缺失/不可比 | 22.9784 |
| python/lqr/single_obstacle | 3/3/3 | 0.425961/0.453294 | 18.2667/18.3 | 0.86858 | 27.1544/31.7298；18.4342/20.393 | 14.3744 |
| python/lqr/straight | 3/3/3 | 0.0537959/0.0566618 | 15.9333/16 | 缺失/不可比 | 29.5446/32.2158；9.57991/9.64277 | 6.03346 |
| python/lqr/turn | 3/3/3 | 1.09823/1.28964 | 17.4667/17.6 | 缺失/不可比 | 23.4482/24.5341；11.2068/11.2623 | 10.1685 |
| python/pid/single_obstacle | 3/3/3 | 0.20976/0.242246 | 22.6667/22.7 | 0.791767 | 4.88411/5.3304；10.9944/11.8259 | 44.6429 |
| python/pid/straight/cancel | 1/1/1 | 0.102739/0.102739 | 5.5/5.5 | 缺失/不可比 | 7.11374/7.11374；13.4996/13.4996 | 4.16509 |
| python/pid/straight/goal_change | 1/1/1 | 0.294669/0.294669 | 25/25 | 缺失/不可比 | 5.46668/5.46668；19.0851/19.0851 | 52.3827 |
| python/pid/straight | 3/3/3 | 0.0709391/0.0864591 | 20.8333/21 | 缺失/不可比 | 3.67079/3.94808；10.3022/12.1606 | 15.2298 |
| python/pid/turn | 3/3/3 | 0.081149/0.0900838 | 20.6/20.7 | 缺失/不可比 | 5.30491/6.05142；12.3368/14.2079 | 38.1234 |
| python/None/None/odom_dropout | 1/1/1 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 6.03613/6.03613；8.11045/8.11045 | 3.35304 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 2.17709/70636 | 17.0039/69496 | 35.5961/102.173 | 10.6803/8.79316/0 |
| python_pid_straight_02 | 2.31535/71084 | 16.9105/69236 | 38.7431/106.897 | 6.38564/5.82864/0 |
| python_pid_straight_03 | 2.34068/71092 | 18.427/69760 | 31.7621/104.63 | 7.51473/6.48694/0 |
| python_pid_turn_01 | 4.06366/70668 | 35.5816/69384 | 29.9319/97.5607 | 16.6776/19.2338/0 |
| python_pid_turn_02 | 4.36613/71284 | 38.0081/69348 | 40.6632/107.506 | 17.5919/19.5415/0 |
| python_pid_turn_03 | 4.00018/71756 | 34.7902/69460 | 45.4088/102.656 | 19.9599/21.3655/0 |
| python_pid_single_obstacle_01 | 3.73832/71844 | 31.2287/68908 | 24.741/107.763 | 21.9178/20.847/0 |
| python_pid_single_obstacle_02 | 3.80984/70792 | 31.8633/69572 | 28.1182/100.133 | 21.1799/19.9388/0 |
| python_pid_single_obstacle_03 | 3.98326/71316 | 32.3111/69736 | 37.5778/107.463 | 24.0485/25.9968/0 |
| python_lqr_straight_01 | 8.71694/71480 | 30.6495/69316 | 46.8166/105.687 | 4.10164/3.936/0 |
| python_lqr_straight_02 | 9.35403/71464 | 35.438/69432 | 38.561/107.048 | 2.71027/2.71304/0 |
| python_lqr_straight_03 | 9.52398/71256 | 38.4964/69500 | 32.4734/103.398 | 2.33371/2.30572/0 |
| python_lqr_turn_01 | 8.84195/71572 | 34.8481/69092 | 31.6669/108.613 | 4.43786/3.93449/0 |
| python_lqr_turn_02 | 8.69835/70976 | 31.2109/69464 | 30.578/111.059 | 6.62598/6.41305/0 |
| python_lqr_turn_03 | 9.46899/71872 | 34.6819/69304 | 33.8059/102.239 | 4.62995/4.46427/0 |
| python_lqr_single_obstacle_01 | 9.92263/71520 | 38.4325/69436 | 40.6857/99.3594 | 7.71237/5.70729/0 |
| python_lqr_single_obstacle_02 | 9.1553/71552 | 33.8919/69480 | 32.1321/124.369 | 8.464/6.202/0 |
| python_lqr_single_obstacle_03 | 9.33905/71676 | 37.1229/69116 | 45.9964/100.543 | 8.78937/6.2482/0 |
| python_pid_goal_change_01 | 4.0631/70644 | 34.0705/69128 | 43.5911/106.872 | 25.6771/26.7056/0 |
| python_pid_cancel_01 | 4.12404/70568 | 缺失/不可比/69224 | 27.0941/84.5512 | 2.22104/1.94405/0 |
| python_pid_odom_dropout | 4.85854/63244 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 1.35304/2/0 |
| cpp_pid_straight_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.13305/2.92195/0 |
| cpp_pid_straight_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.34128/3.56666/0 |
| cpp_pid_straight_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 3.26584/3.00452/0 |
| cpp_pid_turn_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 10.1061/9.97386/0 |
| cpp_pid_turn_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 11.9619/11.9872/0 |
| cpp_pid_turn_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 12.4533/12.4529/0 |
| cpp_pid_single_obstacle_01 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 27.9832/26.9739/0 |
| cpp_pid_single_obstacle_02 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 23.5353/22.9854/0 |
| cpp_pid_single_obstacle_03 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 24.0448/24.1265/0 |

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 327, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 330, "processed": 327, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 283, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 284, "processed": 283, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 335, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 339, "processed": 335, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 340, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 341, "processed": 340, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_02` 建图最后观测计数：`{"integrated_scans": 336, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 338, "processed": 336, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_turn_03` 建图最后观测计数：`{"integrated_scans": 336, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 338, "processed": 336, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 418, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 420, "processed": 418, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 416, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 420, "processed": 416, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 393, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 395, "processed": 393, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_01` 建图最后观测计数：`{"integrated_scans": 229, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 234, "processed": 229, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_02` 建图最后观测计数：`{"integrated_scans": 261, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 262, "processed": 261, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_straight_03` 建图最后观测计数：`{"integrated_scans": 209, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 211, "processed": 209, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_01` 建图最后观测计数：`{"integrated_scans": 225, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 229, "processed": 225, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_02` 建图最后观测计数：`{"integrated_scans": 230, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 233, "processed": 230, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_turn_03` 建图最后观测计数：`{"integrated_scans": 258, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 259, "processed": 258, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 336, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 337, "processed": 336, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 311, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 314, "processed": 311, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_lqr_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 331, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 333, "processed": 331, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_goal_change_01` 建图最后观测计数：`{"integrated_scans": 388, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 392, "processed": 388, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。
- `python_pid_cancel_01` 建图最后观测计数：`{"integrated_scans": 51, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 52, "processed": 51, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "invalid_consumed": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-release-grid-001`：角色 python，run_finished=True，选中 21 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-grid-interior-001`：角色 cpp，run_finished=True，选中 9 条，未观测计划 0 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
