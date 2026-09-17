from pathlib import Path
import ast
import difflib
import io
import json
import tokenize

ROOT = Path(__file__).resolve().parents[3]
TARGET = ROOT / 'scripts/validation/test_compare_results.py'
BASE = ROOT / 'deliverables/_chinese_comments_20260914/before/scripts/validation/test_compare_results.py'
comments = {
    'result': '构造单次导航摘要，显式保留 result、passed、RMSE 与仿真耗时，缺失间隙等字段保持 None。',
    'write_run': '生成与验收日志一致的目录、manifest、总摘要和逐次事件文件，供聚合器从磁盘读取测试。',
    'performance_events': '生成目标发布至试验结束的状态流；accepted 按处理、溢出、过期和在队数量守恒构造。',
    'paired_report': '为同一重复号准备 Python 与 C++ 结果，性能遥测只放入 Python 侧，返回两侧汇总报告。',
    'test_source_selection_preserves_legacy_and_raw_summary': '切换到源时间 RMSE 后使用新的比值判定，同时保留原始摘要和旧墙钟指标，不覆盖历史观测。',
    'test_selected_source_without_contract_is_unknown_not_wall_fallback': '请求源时间指标却缺少显式坐标系约定时必须不可比，不能因旧指标可通过就自动回退。',
    'test_nonfinite_source_pose_yields_serializable_unknown': '源位姿含 NaN 时输出可被严格 JSON 序列化的 unknown；默认旧路径仍按非有限 JSON 拒绝该日志。',
    'test_completed_pairs_at_ten_percent_limit_and_group_counts': '误差与耗时恰好增加 10% 仍可通过；按已观测重复号计数并求均值，缺失间隙不补零。',
    'test_invalid_baseline_never_computes_ratio_or_counts_as_passed': 'C++ 基线只有同时 completed 且 passed 才能作比较；失败、阻塞、未运行或中断均不给比值和通过票。',
    'test_missing_baseline_is_explicit_and_missing_metric_is_not_zero': '完全缺少基线和基线缺少单个指标分别保留原因；后者指标值保持 None，比较结果保持未知。',
    'test_completed_passed_trials_can_still_fail_relative_ten_percent_requirement': '两侧各自导航验收通过也可能不满足相对 10% 要求；此时属于可比但失败。',
    'test_exact_zero_baseline_has_absolute_tolerance_and_no_invented_ratio': '零基线用 1e-6 绝对容差并令比值为空；任何非零基线仍使用相对阈值，不能借此放宽。',
    'test_thruster_variation_uses_observed_changes_without_padding_zeros': '推力总变差只累计各轴已观测相邻指令的绝对变化，不在序列两端补零。',
    'test_missing_thruster_channel_cannot_be_imputed_as_zero': '缺失任一推力通道时总变差保持未知，不能假定未观测通道始终为零。',
    'test_cpu_delta_rss_and_final_status_mapping_counters': '以同进程 CPU 秒数差除单调时间差求占用率，保留峰值内存与最终队列计数；缺少映射 CPU 不补零。',
    'test_process_cpu_is_not_differenced_across_pids_or_clock_resets': '此样例重点验证 PID 切换后不跨进程差分 CPU，只用同一 PID 的有效相邻样本计算。',
    'test_duplicates_require_explicit_selection_and_keep_sources': '重复试验默认报错；显式 latest 与 provided-list 采用各自选择规则，并记录选中及丢弃来源。',
    'test_planned_but_missing_trial_and_faults_are_not_counted_as_comparable': '清单中缺失的计划试验仍进入缺失统计，故障试验单独计数，extra 来源里的普通试验不混入配对。',
    'test_cli_writes_new_reports_and_never_overwrites_raw_or_previous_report': '命令行仅写新报告目录；已有报告及原始运行目录内的输出目标被拒绝，原始摘要字节保持不变。',
    'test_missing_performance_cannot_report_all_passed': '导航配对通过不能替代性能证据；没有遥测时性能为 unknown，总体不通过。',
    'test_control_p99_strict_limit_and_cpp_without_telemetry': '控制 P99 必须严格小于 100 ms，等于阈值也失败；当前性能门槛只统计 Python 试验。',
    'test_slow_control_and_mapping_fail_despite_passing_navigation_ratios': '构造控制超时、映射计算超预算和连续溢出，即使导航比值合格也应报告性能失败。',
    'test_mapping_requires_integrated_computation_and_observed_period': '映射验收须同时具备实际积分计数、计算耗时和观测扫描周期，删除任一项都只能判未知。',
    'test_mapping_budget_uses_median_scan_period_not_long_gap_p95': '计算预算按扫描周期中位数判断，不能用长间隔抬高的 P95 掩盖处理速度不足。',
    'test_two_nonempty_gauges_do_not_prove_sustained_backlog': '仅两次队列快照只有一个完整区间，证据不足以确认持续积压，状态保持 unknown。',
    'test_balanced_progress_with_nonempty_queue_does_not_mean_backlog_failure': '队列可以持续非空，只要处理推进且计数守恒、队列有恢复，就不能仅凭非空判积压失败。',
    'test_recovered_small_endpoint_transient_is_not_sustained_backlog': '区分曾恢复的小幅末端瞬态与单向增长/较大增长，保留净队列变化并按证据判通过或未知。',
    'test_backlog_needs_two_consecutive_growth_intervals_to_fail': '连续两个区间溢出计数增长才判失败；只增长一次保持未知，不直接认定持续积压。',
    'test_occasional_tf_expiry_is_recorded_without_automatic_failure': '孤立的一次 TF 等待过期需要记录，但在其余吞吐证据完整时不自动判失败。',
    'test_incomplete_throughput_evidence_stays_unknown': '观测间隔过长、处理停滞、计数不守恒、窗口内重置或尾部缺失都不足以证明稳定吞吐。',
    'test_control_telemetry_must_cover_end_of_trial': '只有试验起点的控制遥测无法证明末端时效，即使映射吞吐通过，控制时序仍为未知。',
    'test_missing_status_timestamp_is_not_silently_discarded': '中间状态缺少时间戳不能静默丢弃后继续判通过，应使映射时序和积压证据均未知。',
    'test_acquisition_end_excludes_postprocessing_and_later_telemetry': '优先以 acquisition_finished 截止采集窗口，后处理期间的高延迟遥测不污染试验内性能。',
    'test_old_records_explicitly_keep_trial_finished_fallback': '旧日志缺采集结束事件时显式回退 trial_finished，并报告末端回退原因及证据不足。',
    'test_present_but_invalid_acquisition_end_is_not_silently_replaced': '采集结束事件已存在但时间戳无效时，不允许用旧结束事件替换，窗口及性能保持未知。',
    'post_drain_events': '在旧遥测夹具上补充出队后的密集窗口、等待耗时与最老待处理帧年龄，构造新观测契约。',
    'test_new_dense_latency_evidence_accepts_stable_one_frame_pipeline': '新密集等待和年龄证据可确认稳定的一帧流水线，即使净队列增长一且未观察到回空也能通过。',
    'test_old_one_frame_growth_still_unknown_without_new_measurements': '旧稀疏快照同样呈现一帧增长时仍保持未知，不能享受新观测契约的放宽条件。',
    'test_new_observed_empty_queue_needs_no_invented_age_samples': '真实观察到空队列时年龄样本计数为零即可，不需要伪造不存在待处理帧的年龄。',
    'test_new_window_allows_bounded_producer_summary_computation_delay': '允许状态生产端汇总计算带来 10 ms 的有限延迟，不把这种小偏差误判为陈旧观测。',
    'test_new_missing_inconsistent_or_unbounded_queue_evidence_is_unknown': '逐项破坏窗口完整性、阶段、覆盖、年龄上界、容量或身份一致性，新契约均须退回 unknown。',
    'test_new_fine_observations_do_not_replace_three_counter_snapshots': '密集窗口中的大量年龄样本不能替代三次累计计数快照；只有两次快照仍不足。',
    'test_new_small_survivor_ages_do_not_hide_overflow': '幸存待处理帧年龄较小不能掩盖已经发生的溢出，连续两段增长仍失败，单段增长仍未知。',
    'test_new_survivor_measurements_keep_isolated_and_repeated_tf_expiry_distinct': '新年龄观测仍区分孤立与连续 TF 过期：一次允许通过，连续增长不能证明稳定吞吐。',
    'test_new_latency_branch_never_relaxes_computation_limits': '等待/积压分支通过不放宽控制和映射计算时限，计算超预算仍阻止总体通过。',
    'test_new_contract_accepts_observations_produced_by_real_scan_queue': '用真实 ScanQueue 入队、延迟处理和观测接口生成遥测，验证新契约能接受实际生产数据结构。',
    'accounted_scan_events': '构造已接纳但因无效数据被消费的扫描；新旧版本分别用 invalid_consumed 与含无效项的 processed 表达。',
    'test_scan_counter_versions_preserve_successful_integration_accounting': '同时验证两版计数守恒：新版 processed 等于成功积分，旧版 processed 包含成功积分及无效消费。',
    'test_missing_or_negative_scan_counters_never_become_zero': '新旧版本任一必要计数缺失或为负都不能补零，应记录无效样本并使积压验收未知。',
    'test_new_invalid_consumed_must_be_present_and_a_nonnegative_integer': '新版无效消费计数须为非负整数，None/布尔也无效；缺字段造成版本混用时同样不能通过。',
    'test_balanced_interval_deltas_do_not_hide_invalid_lifetime_or_processing_counts': '所有快照加相同偏移会在区间差分中抵消，因此还须检查每个快照的生命周期守恒和处理关系。',
    'test_new_invalid_consumed_is_not_substituted_by_admission_invalid_count': '入队拒绝计数 invalid 与入队后无效消费不同，增加前者不应破坏后者的积分守恒关系。',
    'test_new_invalid_consumed_must_match_node_invalid_scans': '即使同步提高 accepted 维持队列守恒，invalid_consumed 与节点 invalid_scans 不一致仍应判未知。',
    'test_invalid_consumption_without_integrations_does_not_prove_throughput': '持续消费无效扫描但成功积分计数不变时，计数可守恒，仍不能据此证明建图吞吐正常。',
    'test_new_isolated_expired_scan_is_conserved_without_counting_as_processed': '一次过期可以通过并被单独计数，不得并入 processed 冒充成功积分。',
    'test_preexisting_reset_keeps_interval_balance_without_inventing_dropped_count': '采集前已发生的重置与遗留接纳数偏差不破坏窗口内守恒；不凭空推算一个丢弃计数。',
    'test_safety_stopped_trial_is_not_a_completed_comparable_navigation_result': '安全停止不同于成功完成，即使有完整性能遥测也不能进入可比导航配对。',
    'scenario_fixture': '为两种实现各构造固定第 1/2/3 次试验，并分别提供有效的源时间 RMSE 证据。',
    'refresh_scenario_fixture': '按试验键重新配对两侧记录，显式使用源时间 RMSE 及既定相对/零基线阈值。',
    'test_scenario_mean_is_arithmetic_of_three_rmse_values_and_retains_worst_pair': '三次 RMSE 取算术均值并报告样本标准差 ddof=1；均值通过仍保留逐次失败及最差比值，且不修改输入报告。',
    'test_scenario_mean_never_selects_only_the_best_valid_repeats': '固定三次必须齐全且证据有效；缺次、加次、基线失败、源时间未知或耗时缺失都不可挑最好几次替代。',
    'test_scenario_mean_duration_failure_and_zero_baseline_atol_are_independent': '均值 RMSE 合格不能抵消耗时超限；零基线仍独立采用绝对容差并保持 ratio=None。',
    'test_scenario_mean_requires_source_valid_even_when_wall_selected': '即使旧墙钟指标被选为逐次指标，场景三次均值仍要求有效源时间证据。',
    'test_default_and_explicit_per_repeat_reports_and_cli_stay_compatible': '默认和显式 per_repeat 路径应输出等价 JSON/Markdown，并不引入仅属于场景均值的新字段。',
    'test_scenario_mean_report_labels_and_unknown_performance_remain_independent': '场景均值通过仍不能盖过三次性能未知；报告标明算术均值、标准差及逐次结果，CLI 保持失败退出码。',
    'test_scenario_mean_two_eligible_four_incomparable_preserves_all_individuals': '六组中两组可比通过、四组不可比应分别统计；不可比组仍保留三次数据和全部 18 条逐次比较。',
    'test_scenario_mean_invalid_values_remain_unknown': '负值、NaN、无穷、布尔和 None 都不是有效源时间 RMSE，输出 None/未知并保证严格 JSON 可序列化。',
}
raw = TARGET.read_bytes()
source = raw.decode('utf-8')
lines = source.splitlines(keepends=True)
tree = ast.parse(source)
functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
assert set(comments) == {f.name for f in functions}
insertions = {}
for f in functions:
    pos = min([f.lineno] + [d.lineno for d in f.decorator_list]) - 1
    insertions[pos] = '# ' + comments[f.name]
nl = '\r\n' if b'\r\n' in raw else '\n'
after = ''.join((insertions[i] + nl if i in insertions else '') + s for i, s in enumerate(lines)).encode('utf-8')
assert ast.dump(ast.parse(after)) == ast.dump(tree)
def tokens(data):
    return [(t.type, t.string) for t in tokenize.tokenize(io.BytesIO(data).readline) if t.type not in (tokenize.COMMENT, tokenize.NL, tokenize.ENCODING)]
assert tokens(raw) == tokens(after)
for previous in (raw, BASE.read_bytes()):
    a, b = previous.splitlines(keepends=True), after.splitlines(keepends=True)
    additions = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        assert tag in ('equal', 'insert'), tag
        if tag == 'insert':
            assert all(line.lstrip().startswith(b'#') for line in b[j1:j2])
            additions.extend(range(j1, j2))
    assert b''.join(line for i, line in enumerate(b) if i not in additions) == previous
TARGET.write_bytes(after)
print(json.dumps({'top_level_functions_covered': len(functions), 'added_comment_lines': len(insertions), 'AST_and_tokens': 'pass', 'original_and_current_lines_preserved': True}))
