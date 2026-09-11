# Python/C++ 导航结果对照

总体判定：未通过或证据不完整。可比 9/18 对，通过 4 对，失败 5 对，不可比 9 对；故障用例通过 3/3。

Python 正常导航性能：通过 7、失败 0、证据不足 2；性能证据不足也不能总体通过。

仅双方 result=completed 且 passed=true 才计算比例。阈值 Python/C++ ≤ 1.10；零基线比例不定义，按绝对容差 1e-06 判断。

## 每次配对

| 控制器 | 场景 | 重复 | RMSE 比例 | 时间比例 | 判定/原因 |
| --- | --- | --- | --- | --- | --- |
| lqr | single_obstacle | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | single_obstacle | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | single_obstacle | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | straight | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_not_completed_and_passed: result=interrupted, passed=False, manifest_mode=live, reason=user interrupted this live trial |
| lqr | straight | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | straight | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | turn | 1 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | turn | 2 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| lqr | turn | 3 | 缺失/不可比 | 缺失/不可比 | 不可比：python_result_missing; cpp_result_missing |
| pid | single_obstacle | 1 | 1.14916 | 0.862069 | 失败 |
| pid | single_obstacle | 2 | 1.22574 | 0.809353 | 失败 |
| pid | single_obstacle | 3 | 0.959221 | 0.823741 | 通过 |
| pid | straight | 1 | 0.736336 | 0.795539 | 通过 |
| pid | straight | 2 | 1.1294 | 0.902542 | 失败 |
| pid | straight | 3 | 0.735187 | 0.791822 | 通过 |
| pid | turn | 1 | 2.96489 | 0.902655 | 失败 |
| pid | turn | 2 | 1.1047 | 0.919643 | 失败 |
| pid | turn | 3 | 1.04895 | 0.903084 | 通过 |

## Python 正常导航性能验收

窗口为第一次 goal_published 至 acquisition_finished；旧记录明确回退到 trial_finished，该末端可能包含统计落盘时间。状态中的耗时分位数来自节点计算历史，可能含启动样本，各窗口样本数不相加。控制 P99 必须小于100ms；有实际积分扫描的建图处理 P95 不得超过同一状态中扫描时间戳间隔的 P50。扫描周期仅作为预算，不充当算法耗时。C++ 无对应遥测不要求性能对等。

积压至少需要3次同一进程/时钟周期的有效快照、2个连续约5s间隔，并覆盖试验窗口首尾。连续两个间隔 overflow 增长判失败；单次溢出、采样缺口、停滞或计数不自洽标待审。处理持续进展、队列计数平衡、无溢出增长时，有明确队列回落的末端1～2扫描瞬态净增不单独否决；持续非减且净增仍待审。两次 queue>0 不能证明持续积压，偶发 TF 过期单列记录。结论仅覆盖观测窗口，不保证无限时长稳定。

| Python试验 | 总状态 | 控制耗时 | 建图耗时 | 积压 | 状态样本/完整间隔/跨度s |
| --- | --- | --- | --- | --- | --- |
| python_pid_straight_01 | 通过 | 通过 | 通过 | 通过 | 5/4/20.8636 |
| python_pid_straight_02 | 通过 | 通过 | 通过 | 通过 | 5/4/20.8927 |
| python_pid_straight_03 | 通过 | 通过 | 通过 | 通过 | 5/4/20.9077 |
| python_pid_turn_01 | 通过 | 通过 | 通过 | 通过 | 4/3/15.6794 |
| python_pid_turn_02 | 通过 | 通过 | 通过 | 通过 | 4/3/15.8241 |
| python_pid_turn_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.6754 |
| python_pid_single_obstacle_01 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 5/4/20.8655 |
| python_pid_single_obstacle_02 | 待审/证据不足 | 通过 | 通过 | 待审/证据不足 | 4/3/15.6604 |
| python_pid_single_obstacle_03 | 通过 | 通过 | 通过 | 通过 | 4/3/15.7165 |

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
| python/pid/single_obstacle | 3/3/3 | 0.317743/0.3292 | 22.6333/22.9 | 0.52824 | 4.72945/5.06377；11.0444/12.024 | 43.0596 |
| python/pid/straight/cancel | 1/1/1 | 0.228864/0.228864 | 5.5/5.5 | 缺失/不可比 | 6.99287/6.99287；10.7738/10.7738 | 4.27999 |
| python/pid/straight/goal_change | 1/1/1 | 0.330486/0.330486 | 25.7/25.7 | 缺失/不可比 | 4.85632/4.85632；20.0173/20.0173 | 38.5949 |
| python/pid/straight | 3/3/3 | 0.131949/0.134356 | 21.3333/21.4 | 缺失/不可比 | 3.99174/4.40518；12.5772/14.9559 | 19.7729 |
| python/pid/turn | 3/3/3 | 0.252471/0.449221 | 20.5/20.6 | 缺失/不可比 | 4.19283/4.42504；11.1402/11.7875 | 34.2633 |
| python/None/None/odom_dropout | 1/1/1 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 缺失/不可比 | 7.12563/7.12563；8.85558/8.85558 | 3.30456 |

## 进程与建图观测

CPU 按同 PID 的累计 CPU 秒增量/monotonic 时间增量计算，可超过100%；导航进程数据不含规划子进程和 Gazebo。P99 为每例最后报告窗口的 P99，均值/最大不等于合并样本的总体 P99。RSS 单位沿用 Linux 字段 KiB。

| 试验 | 导航 CPU% /峰值KiB | 建图 CPU% /峰值KiB | 建图处理/排队P99 ms | 左/右/侧推总变差 |
| --- | --- | --- | --- | --- |
| python_pid_straight_01 | 2.29485/71296 | 17.4462/69604 | 30.0698/107.574 | 10.1868/8.2983/0 |
| python_pid_straight_02 | 2.34879/71356 | 17.9272/69468 | 36.0619/108.341 | 8.91364/9.94529/0 |
| python_pid_straight_03 | 2.21346/71172 | 17.0733/68612 | 29.8231/107.732 | 11.7016/10.2729/0 |
| python_pid_turn_01 | 2.27582/72260 | 17.2942/69428 | 28.9106/103.518 | 13.8363/15.1459/0 |
| python_pid_turn_02 | 2.39742/71944 | 17.4878/69536 | 28.9092/106.932 | 17.9254/17.8628/0 |
| python_pid_turn_03 | 3.9826/70888 | 36.1536/69024 | 29.0577/105.685 | 18.8443/19.1752/0 |
| python_pid_single_obstacle_01 | 3.6506/71172 | 31.3301/69376 | 25.6788/107.593 | 20.2921/19.8221/0 |
| python_pid_single_obstacle_02 | 3.77126/71208 | 32.5856/69500 | 27.0865/105.547 | 20.5801/20.0388/0 |
| python_pid_single_obstacle_03 | 4.21428/71596 | 34.7412/69560 | 30.464/106.462 | 24.5357/23.9101/0 |
| python_pid_goal_change_01 | 4.32156/71300 | 35.07/68288 | 37.7254/106.356 | 19.4105/19.1844/0 |
| python_pid_cancel_01 | 3.94627/71192 | 缺失/不可比/68848 | 26.7276/112.191 | 2.30559/1.9744/0 |
| python_pid_odom_dropout | 5.17443/63524 | 缺失/不可比/缺失/不可比 | 缺失/不可比/缺失/不可比 | 1.41274/1.89182/0 |
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

- `python_pid_straight_01` 建图最后观测计数：`{"integrated_scans": 333, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 337, "processed": 333, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_straight_02` 建图最后观测计数：`{"integrated_scans": 335, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 338, "processed": 335, "overflow": 0, "expired": 2, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_straight_03` 建图最后观测计数：`{"integrated_scans": 333, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 337, "processed": 333, "overflow": 0, "expired": 3, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_01` 建图最后观测计数：`{"integrated_scans": 301, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 302, "processed": 301, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_02` 建图最后观测计数：`{"integrated_scans": 305, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 307, "processed": 305, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_turn_03` 建图最后观测计数：`{"integrated_scans": 316, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 318, "processed": 316, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_01` 建图最后观测计数：`{"integrated_scans": 395, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 400, "processed": 395, "overflow": 0, "expired": 4, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_02` 建图最后观测计数：`{"integrated_scans": 361, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 1, "queue_counters": {"accepted": 363, "processed": 361, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_single_obstacle_03` 建图最后观测计数：`{"integrated_scans": 361, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 361, "processed": 361, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_goal_change_01` 建图最后观测计数：`{"integrated_scans": 404, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 2, "queue_counters": {"accepted": 407, "processed": 404, "overflow": 0, "expired": 1, "duplicate": 0, "invalid": 0, "resets": 0}}`。
- `python_pid_cancel_01` 建图最后观测计数：`{"integrated_scans": 49, "scan_overruns": 0, "invalid_scans": 0, "invalid_beams": 0, "queue_size": 0, "queue_counters": {"accepted": 49, "processed": 49, "overflow": 0, "expired": 0, "duplicate": 0, "invalid": 0, "resets": 0}}`。

## 来源与选择

重复策略：`error`。缺失通道/性能字段保留 null，不把不可比或未运行当作通过。

- `/home/usv/usv_migration/results/python-pid-final-001`：角色 python，run_finished=True，选中 12 条，未观测计划 0 条。
- `/home/usv/usv_migration/results/baseline-matrix-002`：角色 cpp，run_finished=False，选中 10 条，未观测计划 8 条。

逐例原始 summary、来源路径/哈希、CPU 有效区间及全部聚合字段保存在 report.json。推进器总变差使用完整已记录事件范围，不补起始/结束零值；计数器仅汇总每例最后累计快照。
