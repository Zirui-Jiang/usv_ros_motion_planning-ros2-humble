"""Add local callback and regression-intent comments after the first pass."""
import ast
from collections import defaultdict
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[2]
CALLBACKS={
 'legacy_reference.py':{
  'now_seconds':'读取 ROS 时钟秒数；状态和轨迹的新鲜度都需要使用同一时间基准。',
  '_validate_parameters':'先验证参数更新候选值；失败时拒绝整批更新，防止留下部分生效配置。',
  '_apply_parameters':'在控制调度中应用已确认参数，必要时使旧规划结果失效。',
  '_pose':'用消息对应时刻的 TF 转换位置与艏向，不把不同坐标系的数据直接拼接。',
  '_on_odom':'收到里程计后交给统一转换逻辑，保持订阅入口和状态解析分开。',
  '_consume_odom':'构造带时间和坐标系的 VesselState，并拒绝无效状态输入。',
  '_on_map':'将收到的地图转换为内部快照；新障碍可能立即使当前参考失效并触发停推。',
  '_trajectory_clear':'同时检查当前船体与参考扫掠区域，不只检测轨迹中心线。',
  '_current_footprint_clear':'按当前姿态检查完整矩形船体占用，覆盖船首、船尾和四角。',
  '_reset_task':'任务边界清除旧目标、轨迹和规划请求，同时重置控制状态并发布零输出。',
  '_on_goal':'转换外部目标到导航坐标系，建立新任务并触发规划。',
  '_cancel':'取消服务进入任务重置流程；零推力不代表船体已物理停止。',
  '_export':'导出节点当前有效参数，便于核对运行状态而不是只看启动文件。',
  '_planning':'仅在状态和地图有效时提交带快照的规划请求，避免让异步工作进程读取可变输入。',
  '_consume_plan':'接收最新有效规划结果并再次检查地图，过期结果不能恢复旧任务。',
  '_publish_command':'将数值控制输出转换为推进器话题消息，三个通道保持一致的发布流程。',
  '_control':'控制周期依次处理参数、状态与规划结果，校验输入后跟踪轨迹，失效时停推。',
  '_publish_status':'发布任务状态、计算耗时和过程统计，供监视与验收工具读取。',
  'destroy_node':'退出时先请求零推力并关闭工作进程，再释放 ROS 节点资源。',
 },
 'ros_acceptance.py':{
  'sim_time':'读取 ROS 仿真时间，与用于超时监视的墙钟单调时间分别使用。',
  'message_age':'按 ROS 时钟计算消息源时间年龄，接收回调频繁不等于观测内容新鲜。',
  'on_odom':'保存真实船位与源时间，供到达判定、横向误差和相邻位姿扫掠计算使用。',
  'on_scan':'记录扫描接收时刻、源时间年龄与间隔，不在验收端重建地图。',
  'on_path':'记录有效路径；横向误差参考与任务边界关联，避免替换目标后继续沿用旧路径。',
  'on_trajectory':'记录用于显示的局部轨迹及其源时戳，不把它代替模块间的完整执行参考。',
  'on_status':'保存导航状态；进入参考或地图失败状态时捕获相关输入，便于离线复查。',
  'on_mapping_status':'原样记录建图节点报告的性能和队列证据，避免在验收端猜测缺失数据。',
  'on_parameter_event':'记录运行中参数事件，区分从启动固定的配置与中途调参诊断。',
  'on_goal':'记录实际目标消息和源时间，便于追溯任务何时被替换。',
  'on_command':'保存每路真实推力值及接收时刻，统计首命令延迟、输出间隔与停推窗口。',
  'discover_costmap':'优先发现 raw costmap，必要时兼容 OccupancyGrid，并明确记录实际采用的话题类型。',
  'on_costmap':'保存地图数据、原点、分辨率和时间信息，保留 raw 与占据栅格的类型区别。',
  'capture_inputs':'将当前可见的地图、状态和轨迹分别落盘；异步订阅快照不是规划线程的原子输入。',
  'publish_goal':'先保存发布前输入快照，再发布目标并重置该目标的观测统计。',
  'zero_received_since':'检查指定时刻之后各通道是否收到零值；完整持续停推还需通过窗口覆盖检查。',
 },
 'distributed_smoke.py':{
  'on_command':'保存每个推进器通道的实际输出和接收时间，用于判定运动及停推。',
  'on_status':'记录决策公开状态，供目标取消和任务失效的后续断言读取。',
  'on_report':'记录航控回报，区分决策请求与控制端实际处理结果。',
  'on_heartbeat':'记录模块会话存活信息，用于识别进程重启后的身份变化。',
  'on_reference':'深拷贝已收到的完整轨迹参考，便于测试迟到消息和旧版本重放。',
  'start':'以独立子进程启动指定模块，保存日志与进程所有权。',
  'stop':'只停止测试自己创建的指定模块，模拟单模块退出与重启。',
  'inputs':'按测试开关发送静止自船状态和空地图，可单独停止某一路输入。',
  'step':'执行一次受控输入、进程健康检查和 ROS 回调处理。',
  'until':'等待可观测条件成立，超时则报告阻塞，不修改原有等待阈值。',
  'pump':'在给定时长内持续推进输入与回调，为稳态观察提供时间窗口。',
  'ready':'等待订阅连接和模块就绪后再下发任务，减少启动阶段对协议断言的干扰。',
  'goal':'发布带当前 ROS 时间的目标点，建立下一次目标处理观察。',
  'moving':'等待实际收到非零命令，证明控制端进入输出阶段后再开展故障测试。',
  'zero_window':'收集观察窗口并调用持续零输出判据，拒绝仅有一条历史零命令的假停推。',
  'close':'依次回收本测试的子进程并销毁观测节点。',
 },
}
TESTS={
 'test_original_configuration_loads_and_aliases_work':'验证既有项目配置可加载，旧别名与规范参数保持一致。',
 'test_removed_sensor_parameters_are_rejected_instead_of_silently_accepted':'已移除的传感器参数必须明确拒绝，不能表面接受但实际不生效。',
 'test_alias_conflicts_rejected':'同一含义的别名给出冲突值时拒绝启动或更新。',
 'test_invalid_update_is_transactional':'一项参数非法时整批更新失败，原有效参数不发生部分变化。',
 'test_invalid_startup':'检查非法启动配置在进入运行流程前被拒绝。',
 'test_current_time_lqr_reference_is_valid_but_not_hot_mutable':'当前时刻参考可用于 LQR，但该配置不允许按热参数随意切换。',
 'test_startup_goal_distance_tolerance_rejects_unresolvable_los_radius':'到点半径不能小于 LOS 可分辨的精度，否则启动校验必须失败。',
 'test_startup_preserves_smallest_admitted_goal_radius':'最小可接受到点半径应原值保留，不能被静默放大。',
 'test_project_yaml_rejects_goal_radius_below_los_precision':'直接从项目 YAML 读取时也必须执行同一到点半径精度检查。',
 'test_standalone_navigation_starts_only_its_business_module':'独立入口只包含自己的业务节点，不偷偷启动另一个业务模块。',
 'test_perception_entry_has_only_its_mapping_costmap_and_restart_monitor':'感知入口只拥有建图、代价地图和身份监视所需节点。',
 'test_main_keeps_legacy_command_and_composes_four_independent_entries':'主入口保留原命令形式，并组合仿真支撑及三个独立业务入口。',
 'test_simulation_support_owns_gui_policy_without_business_nodes':'无界面开关由仿真支撑管理，该入口不创建业务节点。',
 'test_custom_project_yaml_and_launch_precedence_match_across_business_entries':'自定义 YAML 和启动覆盖在决策、航控入口得到一致有效参数。',
 'test_invalid_project_yaml_fails_before_launch_actions_are_returned':'对应入口解析到非法 YAML 时，不返回可执行的节点动作。',
 'test_main_forwards_perception_config_paths_to_owning_package':'主入口把感知配置路径传给拥有该配置的感知包。',
 'test_all_entries_reject_empty_namespace_before_starting_nodes':'空命名空间在所有入口中均被拒绝，防止话题和坐标系身份含糊。',
 'test_transient_controller_timeout_is_counted_and_publishes_zero':'控制计算超时既要计入诊断，也要实际发布零推力。',
 'test_current_hull_blocks_control_even_when_reference_and_centre_are_clear':'轨迹和中心点可通行时，当前船角占据障碍仍必须阻止控制输出。',
 'test_process_planning_parameters_stale_inputs_cancel_and_clock_reset':'通过真实工作进程覆盖规划、参数更新、输入过期、取消和时钟回退的连续过程。',
 'test_complete_offline_chain_and_route_reuse':'完整离线规划链可产生参考，并在条件允许时复用已有路径。',
 'test_legal_soft_escape_route_is_reused_without_a_second_global_search':'合法软膨胀区逃逸路径复用时不应重复调用全局搜索。',
 'test_reused_escape_route_rechecks_latest_map_plateau_uphill_hard_and_unknown':'复用逃逸路径也必须重新检查最新地图的平台、上坡、硬障碍与未知格。',
 'test_reused_escape_route_checks_connection_from_the_latest_vessel_position':'重新规划的连接段必须从最新船位检查，不能只验证历史起点。',
 'test_failure_is_explicit_and_has_no_reference':'失败结果必须显式说明状态，且不附带可继续执行的参考轨迹。',
 'test_default_pid_keeps_straight_grid_endpoint_without_a_goal_kink':'默认 PID 的格心终点策略不应在直线路径尾部制造突兀折点。',
 'test_los_stop_disk_does_not_stop_lqr_outside_the_requested_goal':'LOS 停止圆盘不能使 LQR 在尚未满足最终目标容差时提前完成。',
 'test_small_or_equal_tolerance_appends_goal_without_losing_grid_endpoint':'较小或相等容差下追加精确目标，同时保留搜索得到的格心终点。',
 'test_required_goal_connection_checks_soft_hard_and_unknown_cells':'末端补接目标也必须通过软代价、硬障碍和未知区域检查。',
 'test_freshness':'检查时间边界上的新鲜度判定，防止将过期输入视为有效。',
 'test_input_gate_does_not_reuse_stale_trajectory':'输入门控拒绝过期轨迹，不能因仍有缓存对象就继续执行。',
 'test_worker_keeps_only_latest_pending_and_invalidates':'有界工作队列只保留最新待处理请求，失效后旧结果不得重新进入控制。',
 'test_first_search_refreshes_pending_route_but_keeps_latest_inputs':'首次搜索结果可供后续请求复用，但不得把新请求的状态和地图回退到旧值。',
 'test_pending_route_is_rechecked_against_latest_obstacles':'待处理请求复用路径前要检查最新障碍，不能直接信任先前通过的结果。',
 'test_rectangle_corner_collision_detected_when_centre_circle_would_miss':'构造中心圆近似会漏判的船角碰撞，验证完整矩形几何检查。',
 'test_oriented_footprint_changes_clearance':'只改变艏向也会改变矩形船体间隙，不能把船体当作方向无关的圆。',
 'test_swept_collision_detects_obstacle_between_disjoint_samples':'两个端点都安全时，仍应发现位姿间运动穿过的障碍。',
 'test_polyline_handles_corner_duplicate_and_missing_points':'折线距离计算需处理拐角、重复点和空路径。',
 'test_verdict_never_passes_unobserved_arrival_or_missing_path':'未观测到到达或没有参考路径时，验收不能默认为通过。',
 'test_supervision_stops_on_boundary_without_hiding_positive_margin_failures':'区分触发停止的几何边界和仍为正但不足验收裕量的间隙。',
 'test_regular_finite_zero_stream_covers_entire_window':'各路有限零命令按正常节奏覆盖整个窗口，才满足持续停推。',
 'test_nonfinite_or_nonzero_sample_cannot_pass':'任一 NaN、无穷或非零样本都会使停推窗口失败。',
 'test_one_cached_zero_per_axis_cannot_pass':'每路只有一条缓存零值不能证明随后持续停推。',
 'test_missing_or_incomplete_stream_cannot_pass':'命令通道缺失或首尾覆盖不足时不能通过。',
 'test_nonmonotonic_receipts_are_rejected':'接收时刻倒退会破坏窗口证据，必须拒绝。',
 'test_decision_readback_cannot_substitute_for_actual_control':'决策端参数一致不代表控制端实际一致，必须读取航控自己的参数。',
 'test_nonuniform_squared_error_trapezoids_not_equal_sample_rms':'非均匀源采样应按时间积分平方误差，不能对样本简单等权求均方根。',
 'test_same_pose_stamp_deduplicated_and_wall_pause_cannot_change_integral':'重复源时间去重，墙钟暂停不能凭空增加同一状态的积分权重。',
 'test_first_path_uses_receipt_order_and_all_prepath_motion_is_retained':'首条路径按接收顺序选取，同时保留路径到来前已经观测到的运动。',
 'test_bad_or_incomplete_evidence_is_unknown':'无效或覆盖不足的证据返回未知，不补零或外推。',
 'test_contract_is_never_implicitly_assumed':'未显式选择坐标系契约时，不能自动假定 map 与 odom 相同。',
 'test_boundary_half_second_is_inclusive_and_no_extrapolation':'恰好半秒的覆盖边界按协议接纳，积分区间仍不超出实际观测。',
 'test_observations_after_acquisition_and_older_pregoal_are_excluded':'剔除采集结束后的观测和不属于本次目标窗口的旧样本。',
 'test_polyline_endpoint_distance_is_not_infinite_line_distance':'在线段延长线上的点应计算到端点的距离，而不是错误地得到零。',
 'test_linear_squared_error_has_known_integral_on_nonuniform_grid':'用有解析积分的平方误差函数检验非均匀梯形积分数值。',
 'test_declared_frame_contract_contains_both_audited_source_and_log_hashes':'坐标系契约要同时保留历史源码和运行日志的身份哈希。',
 'test_explicit_install_selection_overrides_existing_project_install':'显式指定外部安装区时，不应误用项目内已有 install。',
 'test_unset_install_selection_defaults_to_project_install':'没有指定安装区时使用项目默认 install。',
 'test_missing_selected_setup_fails_with_path_and_override_instructions':'选定安装区不存在时立即失败，并指出具体缺失路径。',
 'test_explicit_install_path_with_spaces_is_sourced_intact':'安装路径含空格时保持一个完整参数，不拆分加载。',
 'test_launch_arguments_are_forwarded_without_splitting_or_evaluation':'启动参数原样传递，不做额外拆词或执行参数中的 shell 文本。',
 'test_setup_failure_stops_before_ros2_launch':'环境加载失败后不得继续启动 ros2。',
 'test_module_directory_is_not_a_ros_package_and_children_have_declared_names':'业务分组目录不是包，真正的子包名由各 package.xml 声明。',
 'test_shared_package_does_not_depend_on_business_or_simulation':'公共包依赖闭包不能反向包含业务实现或仿真。',
 'test_business_can_be_built_without_simulation_or_peer_business_packages':'单个业务包的依赖闭包不包含其他业务包或 sim_env。',
 'test_local_package_dependency_graph_is_acyclic':'对本地包依赖图做环检测，防止目录整理引入循环依赖。',
 'test_complete_configuration_ast_does_not_exempt_radar_or_new_statements':'配置审计必须比较全部语句，旧传感器键或新增常量也不能被忽略。',
 'test_definition_audit_detects_constants_defaults_and_function_body':'分别改变常量、默认参数和函数体，审计都应发现差异。',
 'test_definition_audit_allows_only_module_import_relocation':'只调整导入位置而定义保持一致时，定义级比较应保持通过。',
 'test_old_report_uses_legacy_label_and_value':'旧报告缺少新指标字段时，仍明确使用原有墙钟 RMSE。',
 'test_source_plot_csv_labels_and_both_values':'选择源时间指标后，导出标签及两种指标值都要可追溯。',
}

def main():
    files=list((ROOT/'src/sim_env/test').glob('*.py'))+list((ROOT/'scripts/validation').glob('*.py'))+list((ROOT/'scripts/control').glob('test_*.py'))
    count=0
    for path in files:
        if path.name=='test_compare_results.py':continue
        before=path.read_bytes();text=before.decode('utf-8');lines=text.splitlines(keepends=True)
        tree=ast.parse(text);add=defaultdict(list)
        for node in ast.walk(tree):
            if not isinstance(node,ast.FunctionDef):continue
            description=CALLBACKS.get(path.name,{}).get(node.name) or TESTS.get(node.name)
            if not description:continue
            index=min([node.lineno]+[d.lineno for d in node.decorator_list])-1
            indent=re.match(r'\s*',lines[index]).group()
            add[index].append(indent+'# '+description+'\n');count+=1
        if not add:continue
        result=''.join(''.join(add[i])+line for i,line in enumerate(lines))
        assert ast.dump(tree,include_attributes=False)==ast.dump(ast.parse(result),include_attributes=False)
        assert path.read_bytes()==before,'Concurrent edit: '+str(path)
        path.write_bytes(result.encode('utf-8'))
    print('Added',count,'callback and regression-intent comments.')

if __name__=='__main__':main()
