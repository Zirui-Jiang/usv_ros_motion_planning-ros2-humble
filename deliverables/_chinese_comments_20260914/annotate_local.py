"""Insert reviewed Chinese comments; never rewrite an existing source line."""
import ast
from collections import defaultdict
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
WORK = Path(__file__).resolve().parent

HEADERS = {
 'configuration.py': '兼容旧的配置导入路径；实际参数解析与校验集中在公共 usv_navigation 包。',
 'runtime.py': '兼容旧的规划执行器导入路径；实际异步规划逻辑位于 usv_decision.runtime。',
 'simulation.py': '保留旧可执行入口的导入兼容性；普通导航实际运行在独立决策与航控进程中。',
 '__init__.py': '声明 sim_env 的 Python 包；仿真启动资源与业务算法实现分别组织。',
 'launch_support.py': '组合 Gazebo 仿真支撑和感知、决策、航控入口，并统一传递命名空间、时钟及配置路径。',
 'costmap_shutdown_regression.py': '单独启动真实 Nav2 代价地图支撑，区分激活失败、退出超时和异常退出；不启动 Gazebo。',
 'legacy_reference.py': '拆分前的单节点导航参照，仅用于回归对照；当前业务入口使用独立决策、航控节点。',
 'test_configuration.py': '验证项目 YAML 加载、旧参数别名、非法值拒绝和原子更新，避免部分参数生效。',
 'test_launch_modules.py': '展开真实 launch 动作检查模块归属与参数优先级；测试不执行节点启动动作。',
 'test_navigation_ros.py': '使用真实 ROS 通信和规划子进程，验证旧参照的超时停推、船体检查、取消与时钟重置。',
 'test_runtime.py': '离线验证路径搜索到 LOS 的完整调用、路径复用、输入新鲜度和有界规划队列。',
 'test_visual_assets.py': '检查 Otter 网格引用的纹理是否可读，避免 RViz 载入船模时找不到材质。',
 'control_demo.py': '演示一次纯数值 PID/LQR 调用：构造状态与轨迹，打印归一化推进器命令；不发送实机报文。',
 'export_control_workspace.py': '从当前源码提取控制算法及其依赖，保留文件来源与哈希；不复制无关传感器资料或运行缓存。',
 'test_export_control_workspace.py': '用临时工程验证导出边界、拒绝覆盖、链接适配和归档可重复性，不修改真实源包。',
 'capture_provenance.py': '只读记录源码和安装产物的哈希及 Git 状态；采集时机决定这份记录能证明哪个阶段的版本。',
 'compare_results.py': '读取真实验收记录并比较 Python/C++ 的导航、性能与故障结果；缺失或不合格证据保持不可比。',
 'distributed_smoke.py': '用合成 odom 和空地图验证独立进程间的目标、停推和重启协议，不用于评价 Gazebo 导航精度。',
 'distributed_yaml_probe.py': '通过实际启动入口加载独立 YAML，读取节点参数服务，核对决策与航控采用的有效配置。',
 'gui_smoke.py': '短时启动图形界面并保存候选证据；不下发目标，截图是否显示正确仍需人工检查。',
 'localization_smoke.py': '给独立定位进程发送受控状态与时钟，检查畸形输入、坐标转换、时间回退及退出边界。',
 'mapping_throughput.py': '每帧扫描前确认对应 TF 可查，再测源码建图节点吞吐；统计包含同进程驱动与观测开销。',
 'metrics.py': '不依赖 ROS 的验收几何计算；长度用米、角度用弧度，并按完整矩形船体评价圆柱间隙。',
 'plot_acceptance.py': '从已完成试验的比较报告导出图表和 CSV，保留所选指标定义、原始来源与失败记录。',
 'process_sampler.py': '只读 Linux /proc 采样验收进程及其后代的 CPU/RSS；不发送信号，也不估计漏采进程。',
 'replay_los_snapshot.py': '从记录的 raw 地图、自船状态和首条路径离线重放 LOS；各输入时间单独保留。',
 'ros_acceptance.py': '启动和观测真实 ROS/Gazebo 试验，保存原始事件、几何间隙和停推反馈；只清理自身创建的进程。',
 'test_captured_footprint.py': '用已捕获的 raw 地图与轨迹检查完整船体：允许合法软膨胀区情况，仍拒绝实体障碍切角。',
 'test_compare_results.py': '以真实日志字段结构构造测试数据，覆盖缺失基线、不可比指标、队列证据和固定三次均值验收。',
 'test_distributed_install.py': '验证安装区审计确实检查包依赖、实际入口及消息资源；空目录与错误路径必须留下失败结果。',
 'test_distributed_validation.py': '验证停推必须有连续且新鲜的有限零输出，单次缓存零值或错误参数回读不能代替执行证据。',
 'test_harness_options.py': '检查验收命令行参数、配置哈希和安全停止边界；dry-run 只生成计划，不冒充实际试验。',
 'test_killpro_patterns.py': '只检查清理脚本的命令行匹配范围，不实际执行 kill 或清理用户进程。',
 'test_main_launcher.py': '用临时 Bash 环境替代外部安装区和 ros2 命令，验证路径选择、带空格参数与失败传播。',
 'test_metrics.py': '验证矩形船角、姿态变化、采样间扫掠碰撞和折线路径距离，防止中心点近似漏判。',
 'test_module_package_layout.py': '检查业务目录下的 ROS 包声明、依赖闭包与无环性，确保业务包不反向依赖仿真。',
 'test_plot_acceptance.py': '验证图表与 CSV 明确标注所选 RMSE，同时保留旧指标和源时间指标。',
 'test_preservation_audit.py': '用有意变更的临时源码验证保留审计：移动目录可接受，算法常量、默认值和函数体变化必须被发现。',
 'test_tracking_metrics.py': '验证源时间误差积分、重复时间戳去重与覆盖边界；观测不完整时必须返回未知。',
 'tracking_metrics.py': '按真实源时间积分平方横向误差；不外推、不平滑，也不以重复墙钟采样代替新观测。',
 'verify_distributed_install.py': '检查已加载的独立安装区：包依赖、可执行入口、消息序列化及 launch 参数；不启动仿真。',
 'verify_install.py': '审计当前加载的隔离安装布局，验证模块与资源解析路径；结果不代表 ROS 闭环已运行通过。',
 'verify_navigation_preservation.py': '将迁移前快照与当前目录比较：算法/YAML 逐字节、配置 AST 和指定函数定义分别核对。',
 'render_footprint_comparison.py': '根据已有地图与离线轨迹绘制船体轮廓，帮助定位膨胀层拒绝或实体切角；不生成新试验结果。',
 'run.py': '调用原 C++ 动态库生成数值参照，再与 Python API 对比；同时记录库和输入来源。',
 'conftest.py': '为源码测试加入各 Python 功能包的导入根目录，使测试无需先安装这些源码模块。',
}

FUNCTIONS = {
 'load_user_config': '读取可选的用户场景配置；文件不存在时使用默认启动参数。',
 'common_arguments': '声明各入口共用的参数，感知配置默认值来自感知包自己的安装资源。',
 '_value': '在实际 launch 上下文中求值，不能把 LaunchConfiguration 对象直接当作参数字符串。',
 '_boolean': '严格接受 true/false，避免非空字符串 false 被 Python 当成真。',
 'LaunchSettings': '保存已解析的仿真资源路径、命名空间、时钟和初始位姿，供组合入口共用。',
 '_settings': '先校验导航参数，再将初始位置和艏向转换为浮点数。',
 '_simulation_nodes': '创建仿真器、船模、状态发布和真值定位；感知、决策、航控由各自入口创建。',
 '_include_module': '按模块所属包加载 launch，并转发同一组启动参数。',
 '_launch_setup': 'main 组合全部入口，simulation_support 只创建仿真支撑，其余名称转发到所属业务包。',
 'launch_description': '用 OpaqueFunction 延迟到参数可求值时再创建动作，确保配置错误在节点启动前暴露。',
 'run_once': '只运行一次受控代价地图进程，记录生命周期激活和 SIGINT 退出的实际结果。',
 'Simulator': '保留旧单节点的感知输入、异步规划和控制调度，用于与拆分后的实现做回归对照。',
 'seconds': '将 ROS 时间戳的秒与纳秒合并为浮点秒。',
 'yaw': '从消息四元数提取平面艏向角，供平面状态模型使用。',
 'portable_interface_document': '将导出包内可用的源码链接改为相对路径，未导出的依赖明确标注仅在原仓库中。',
 'eligible': '按文件类型和目录过滤复制对象，防止缓存、私有环境配置与采集数据混入导出物。',
 'git_provenance': '记录源包所在工作树的版本与修改状态，不用 Git HEAD 替代尚未提交的文件内容。',
 'export_workspace': '先检查输出边界，再复制当前文件并记录哈希；归档已有时拒绝覆盖。',
 'files_under': '跟随安装区的包符号链接遍历文件，同时用真实路径去重，避免目录环。',
 'fingerprint': '计算文件哈希并比较读取前后的状态，识别采集过程中内容变化的情况。',
 'capture': '汇总源码、安装产物和验收脚本的版本信息，用于追溯本次试验究竟使用了哪些文件。',
 'verdict': '将单项结论和说明组成统一结构，保留通过、失败与未知的区别。',
 'combined_status': '汇总多个独立证据的状态，不能把未知项简单当作通过。',
 'post_drain_queue_evidence': '核对出队后的有限观测窗口；幸存扫描等待时间小，不等于被丢弃或过期的扫描也正常。',
 'backlog_performance': '联合队列长度、计数增量和观测时长判断积压，两个非空队列快照不能证明持续积压。',
 'performance_verdict': '分别评价计算时间、吞吐与观测覆盖；导航到达不能替代实时性证据。',
 'number': '只接受有限数值，并排除 Python 中作为整数子类的布尔值。',
 'finite': '区分有效有限数值与缺失、NaN、无穷和布尔值，避免污染统计结果。',
 'nested': '安全读取嵌套结果字段；缺少字段时保留 None，而不补造零值。',
 'read_json': '读取 JSON 并拒绝非有限数字标记，保持后续统计与序列化的输入契约。',
 'metric_summary': '仅汇总实际存在的有效数值，无观测时保留空结果。',
 'trial_key': '使用实现、控制器、场景、重复次数与故障类型共同标识试验。',
 'comparison_key': '配对同控制器、同场景、同重复次数的 Python/C++ 普通导航结果。',
 'ratio_comparison': '仅比较双方有效指标；基线为零时使用绝对误差阈值，不能虚构除零比例。',
 'process_metrics': 'CPU 增量只能在同一进程、单调时间和未重置的累计计数之间计算。',
 'event_metrics': '从逐条事件提取命令、状态、进程资源与轨迹观测，同时保留无效样本和证据缺口。',
 'load_run': '读取一个试验目录的清单、汇总和原始事件，检查来源及完成状态。',
 'select_duplicates': '按明确选择的策略处理重复试验，记录选取依据而不悄悄挑最好结果。',
 'completed_pass': '只有真实完成且通过的试验可进入有效配对，dry-run 计划不计作运行结果。',
 'selected_rmse': '按报告指定的定义取 RMSE；源时间证据无效时不回退到墙钟指标。',
 'compare_pair': '逐对比较导航指标，并把缺失、失败或不合格来源解释为不可比。',
 'group_results': '按实现、控制器和场景分组，结合计划中的重复次数保留缺失试验。',
 'scenario_mean_acceptance': '要求双方固定三次试验均合格后比较算术均值，同时保留逐次值、标准差与最差情况。',
 'build_report': '将配对导航、独立性能和故障结果合成报告；各项通过条件分别保留。',
 'markdown': '把已计算的报告转换为可读表格，不改变数值或验收结论。',
 'Smoke': '持有测试节点、受控输入和自建业务子进程，观测会话、参考与真实推进器输出。',
 'wrap_angle': '将角差约束到 [-π, π)，避免跨越正负 π 时产生接近一整圈的虚假转角。',
 'point_to_polyline_distance': '投影到每条有限线段并取最近距离，端点之外不能按无限直线计算。',
 'rectangle_circle_clearance': '先把圆心变换到船体系，再求圆与有向矩形的有符号间隙：正为分离、负为重叠。',
 'swept_clearance': '在相邻位姿间插值并扣除未采样区间的运动上界，覆盖船角转动造成的间隙变化。',
 'distribution': '过滤无效观测后计算均值、均方根及分位数；空数据不填零。',
 'trial_verdict': '联合到达、推力合法性、横向误差与船体间隙判断试验，而不是只看是否到点。',
 'supervision_stop_reason': '几何边界触发停止采集；仍为正但小于验收裕量的间隙，会验收失败但不触发此边界。',
 'sha256': '流式计算输入文件 SHA-256，核对绘图数据仍是报告选中的同一份来源。',
 'map_path': '按明确的路径前缀映射定位归档数据，不修改原始记录中的路径。',
 'successful': '仅把已完成且通过的记录识别为成功，保留其余记录用于解释失败。',
 'identity': '提取试验身份字段，让图表和导出数据可以追溯到具体重复试验。',
 'style': '按重复序号稳定分配颜色、标记与线型，便于跨图比较同一次试验。',
 'load_trial': '核对来源哈希后载入轨迹与事件；观测间隔过大时分段显示，避免补画未观测运动。',
 'rmse_metric_name': '兼容旧报告的指标字段，同时明确返回当前采用的 RMSE 定义。',
 'rmse_axis_label': '在坐标轴标出误差单位和指标定义，避免混淆两种 RMSE。',
 'rmse_export_values': '同时导出所选指标、墙钟指标和源时间指标，并保留源时间有效性。',
 'write_csv': '按固定字段顺序写出 CSV，便于表格软件读取与后续复核。',
 'prepare': '筛选规定的 PID/LQR 场景与重复试验，显式记录跳过、缺失和来源问题。',
 'selected': '选择同场景同控制器的试验，并按实现和重复序号稳定排序。',
 'figures': '使用无界面绘图后端导出轨迹及指标图；只消费已有观测，不重跑仿真。',
 'utc_now': '用 UTC 时间标记采样记录，便于与其他观测文件对照。',
 'read_process': '读取一个进程的 /proc 计数，保留采样时刻、父进程和命令行身份。',
 'census': '逐个读取当前可见进程；读取期间已退出的进程只计为观测缺口。',
 'matching_harness': '用验收脚本名和精确输出目录识别目标任务，避免把别的试验合并进来。',
 'descendants': '沿父子关系寻找目标验收进程的后代，限制资源统计范围。',
 'direct_role': '依据进程命令行识别导航、建图、仿真器等角色。',
 'classify': '对无法直接识别的子进程追溯已知祖先，归入相应模块的资源消耗。',
 'read_inputs': '恢复捕获的 raw 地图、状态和首条有效路径；OccupancyGrid 有损转换不满足此重放要求。',
 'TrialBlocked': '表示环境或先决条件不足，不能把未执行的试验记为通过。',
 'EmptyParameterResponse': '单独区分旧同名节点参数服务的空响应，以便受限重试。',
 'sanitized': '把输出中的非有限浮点数转为可序列化的缺失标记，保留数据有效性边界。',
 'zero_output_window': '要求整个观察窗口都被各路新鲜有限零命令覆盖，不能只检查最后一条缓存值。',
 'Jsonl': '逐事件写入并刷新 JSONL，保留试验运行中已经观测到的数据。',
 'OwnedProcess': '为本次试验创建独立进程组并保存输出；关闭时仅管理自己创建的进程。',
 'OwnedProcessGroup': '集中管理本次创建的多个业务进程，任一异常退出都能被试验驱动发现。',
 'free_port': '向系统申请一个当前空闲端口，用于隔离本次 Gazebo master。',
 'stamp_seconds': '将 ROS 消息时间戳转换为浮点秒，与接收端单调时间分别记录。',
 'parameter_value': '依据 Python 值类型构造 ROS 参数，区分布尔、整数和浮点数。',
 'make_probe': '在需要实际运行时才加载 ROS，构造订阅状态、地图、轨迹和推进器的观测节点。',
 'wait_until': '持续处理 ROS 回调并检查自建进程，直到条件成立或等待超时。',
 'call_service': '在限定时间内发现并调用服务，区分可选服务不存在与实际调用失败。',
 'verify_controller': '只对已知空参数回复重试；明确读回了另一控制器时立即拒绝。',
 'apply_start_parameters': '在发布目标前原子设置并读回指定参数，将实际生效配置写入证据。',
 '_verify_controller_once': '读取运行中节点的实际控制器、容差和参数，不能仅凭启动命令认定配置生效。',
 'preflight_corridor': '生成受控障碍前先检查走廊地图，避免把原场景障碍造成的干扰算成待测障碍效果。',
 'setup_world': '复位本次仿真的船舶状态并生成场景指定障碍，然后确认障碍已进入代价地图。',
 'navigation_trial': '完成一次普通导航或目标/取消故障试验，分别记录观测结束和后处理结束。',
 'odom_dropout_trial': '持续发送空地图，只停止 odom 输入，检验状态过期后是否及时且持续停推。',
 'arguments': '解析运行模式、场景、隔离域和输出选项，在启动进程前检查参数合法性。',
 'frame_contract_description': '明确选择经过审计的 map/odom 恒等变换前提；仅帧名相同不足以证明坐标一致。',
 'source_time_trapezoid': '先按事件接收顺序选取观测，再按唯一源时间积分误差平方，并检查首尾与相邻覆盖。',
 'tracking_scores': '并列保留旧墙钟和源时间误差结果，避免修改统计定义后覆盖历史值。',
 'within': '将实际路径解析后检查是否位于指定安装区，防止源码或旧 overlay 冒充当前安装结果。',
 'removed_package_discovery': '检查已移除包是否仍能通过任何 ament overlay 被发现，排查旧环境残留。',
 'installed_dependencies': '沿安装包声明追踪依赖闭包，检查业务层是否错误依赖仿真或其他业务实现。',
 'wrapper_calls_entrypoint': '分析包装脚本的 AST，要求实际导入并调用指定入口；仅注释提到入口不算通过。',
 'launch_nodes': '只展开声明与纯配置动作来检查节点参数，不执行启动进程的 Node 动作。',
 'launch_parameters': '提取展开后的有效节点参数，验证启动配置实际传入了目标节点。',
 'absolute_path': '要求调用者显式选择绝对安装路径，避免工作目录改变造成误审计。',
 'sha': '计算当前文件哈希，用于对应基线快照或 C++ 动态库的精确来源。',
 'packages': '从 package.xml 发现包名与目录，兼容目录迁移并拒绝同名重复包。',
 'definitions': '比较函数、类和常量定义，保留默认参数、装饰器及 docstring 的语义。',
 'configuration_ast': '比较完整配置语法树，不豁免 DEFAULTS 或其他新增配置语句。',
 'package_paths': '按声明解析功能包位置，兼容历史 C++ 布局与当前按业务分组的 Python 布局。',
 'fixtures': '建立确定性的数值输入夹具，确保两种实现接受同一组待比较数据。',
 'python_case': '用 Python 数值接口运行一个夹具，并转换成与 C++ 参照一致的结果结构。',
 'rectangle': '根据位置和艏向构造完整矩形船体顶点，用于地图上显示船首、船尾和四角。',
}

SPECIAL_FUNCTIONS = {
 ('verify_distributed_install.py','audit'): '逐项检查指定安装区的包、入口、消息与配置解析，把失败原因保留在结构化结果中。',
 ('verify_install.py','audit'): '验证安装路径、包发现和资源位置，避免误用当前源码目录满足安装检查。',
 ('verify_navigation_preservation.py','audit'): '按文件身份、配置 AST 和指定迁移定义逐层对照，区分路径变化与行为变化。',
 ('distributed_smoke.py','run'): '顺序执行进程所有权、取消、断流和模块重启用例，每项都依据真实订阅结果判定。',
}

def insert_python(path, original):
    text=original.decode('utf-8')
    lines=text.splitlines(keepends=True)
    tree=ast.parse(text)
    additions=defaultdict(list)
    if path.parent.name=='launch':
        role={'main':'组合仿真支撑、感知、决策和航控的主入口',
              'simulation_support':'仅启动 Gazebo、船模、定位与显示资源',
              'perception':'兼容旧命令，转发到独立感知包',
              'decision':'兼容旧命令，转发到独立决策包',
              'control':'兼容旧命令，转发到独立航控包'}[path.name.split('.')[0]]
        header=role+'；具体动作由 sim_env.launch_support 统一构建。'
    elif path.name=='sim_env_node':
        header='ROS 安装的兼容可执行包装脚本，将调用转交给 sim_env.simulation.main。'
    else:
        header=HEADERS[path.name]
    start=1 if lines and lines[0].startswith('#!') else 0
    if start<len(lines) and re.search(r'coding[:=]',lines[start]):start+=1
    additions[start].append('# '+header+'\n')
    for node in ast.walk(tree):
        if not isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef)):continue
        description=SPECIAL_FUNCTIONS.get((path.name,node.name),FUNCTIONS.get(node.name))
        if node.name=='generate_launch_description':description='向 ROS launch 返回启动描述对象，由启动系统解释执行。'
        if node.name=='main':description='命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。' if 'scripts' in path.parts or path.name=='costmap_shutdown_regression.py' else None
        if description:
            index=min([node.lineno]+[d.lineno for d in node.decorator_list])-1
            indent=re.match(r'\s*',lines[index]).group()
            additions[index].append(indent+'# '+description+'\n')
    # 只在已经逐段核对过的明确语句前补充局部说明，避免插入字符串内部。
    anchors={
      'launch_support.py':{
       '    headless = _boolean(context, "headless")':'headless 优先关闭两个 GUI；各业务节点仍照常运行。',
       '    description = Command([':'通过 xacro 展开当前船模；此入口明确关闭 IMU/GPS 并启用 LiDAR。',
      },
      'control_demo.py':{
       '    dt = 0.1':'示例仅计算一个 0.1 秒控制步，不积分真实船舶运动。',
       '    trajectory = Trajectory([':'八列依次为相对时间、弧长、x、y、艏向、曲率、参考速度与参考加速度。',
      },
      'metrics.py':{
       '    fraction =':'投影比例限制在线段两端之间；相邻重复点按端点处理。',
       '    qx =':'减去矩形半长和半宽，把四角与边界统一为有符号距离问题。',
       '    return minimum -':'扣除插值步内平移和船角转动的保守上界，不代表重建了未知真实动力学。',
      },
      'tracking_metrics.py':{
       'FRAME_EVIDENCE = [':'以下哈希和行号属于列出的历史提交与日志，不应随当前文件新增注释而改写。',
      },
      'ros_acceptance.py':{
       '        apply_start_parameters(probe, args.parameter_overrides)':'先设置并读回参数，再发布目标，保证整次试验使用可追溯的固定起始配置。',
       '        threshold = {**config["thresholds"]':'以运行中控制器的真实到点容差替代配置中的预备阈值。',
       '            if cancelled:':'取消完成需观测各路及时归零且持续为零；服务成功不能代替后续命令观测。',
       '            while history_index < len(probe.odom_history):':'逐条处理新到的 odom，检查相邻观测之间的船体扫掠间隙。',
      },
      'replay_los_snapshot.py':{
       '    route[-1] = state_data["goal"]':'此历史重放脚本会把末点换成精确目标；不要把它等同于所有后续运行时版本的私有输入。',
      },
    }.get(path.name,{})
    for anchor,description in anchors.items():
        indexes=[i for i,line in enumerate(lines) if line.startswith(anchor)]
        for i in indexes[:1]:
            indent=re.match(r'\s*',lines[i]).group()
            additions[i].append(indent+'# '+description+'\n')
    output=''.join(''.join(additions[i])+line for i,line in enumerate(lines))
    output+=''.join(additions[len(lines)])
    assert ast.dump(tree,include_attributes=False)==ast.dump(ast.parse(output),include_attributes=False),str(path)
    return output.encode('utf-8')

def insert_lines(original, header, prefix='#', anchors=None, xml=False):
    text=original.decode('utf-8')
    lines=text.splitlines(keepends=True)
    additions=defaultdict(list)
    index=1 if lines and (lines[0].startswith('#!') or lines[0].startswith('<?xml ')) else 0
    make=lambda s,indent='':indent+('<!-- '+s+' -->' if xml else prefix+' '+s)+'\n'
    additions[index].append(make(header))
    comment_spans=list(re.finditer(r'<!--.*?-->',text,re.S)) if xml else []
    starts=[];at=0
    for line in lines:starts.append(at);at+=len(line)
    for anchor,description in (anchors or {}).items():
        for i,line in enumerate(lines):
            if not line.lstrip().startswith(anchor):continue
            if xml and any(m.start()<=starts[i]<m.end() for m in comment_spans):continue
            indent=re.match(r'\s*',line).group()
            additions[i].append(make(description,indent));break
    return ''.join(''.join(additions[i])+line for i,line in enumerate(lines)).encode('utf-8')

def non_python(path, original):
    relative=path.relative_to(ROOT).as_posix()
    if path.suffix in {'.xml','.sdf','.xacro','.world'}:
        if path.name=='package.xml':header='声明 sim_env 的构建、安装与运行依赖；仿真资源包与独立业务节点通过这些包名关联。'
        elif path.suffix=='.sdf':header=f'Gazebo 模型 {path.parent.name} 的结构描述；碰撞体、外观和惯性分别定义，网格纹理仍由资源文件提供。'
        elif path.suffix=='.world':header=f'仿真场景 {path.stem}：定义环境、物理步进、模型实例及场景插件；不是规划算法输出。'
        else:
            labels={
             'materials.urdf.xacro':'船模使用的命名材质与颜色，供 visual 外观引用。',
             'otter.urdf.xacro':'Otter 船模组合入口，展开艇体、推进器和按开关启用的传感器及物理插件。',
             'otter_base.urdf.xacro':'船体基础连杆、质量惯量与外观/碰撞模型；这些决定 Gazebo 刚体结构。',
             'parts.mininertia.urdf.xacro':'为艇体附属部件定义小惯量宏，供主船模组合时引用。',
             'propeller.xacro':'推进器连杆与关节的 xacro 模板，描述安装位置及其机械连接。',
             'otter_gazebo_dynamics_plugin.xacro':'向 Gazebo 水动力插件传入船型、附加质量与阻尼配置。',
             'otter_gazebo_wind_plugin.xacro':'定义风载插件的配置宏；是否生效取决于上层船模是否展开调用。',
             'otter_gps.xacro':'GPS 传感器模板，描述安装位姿和仿真输出；普通导航入口当前关闭该传感器。',
             'otter_imu.xacro':'IMU 传感器模板，描述安装位姿和输出配置；普通导航入口当前关闭该传感器。',
             'otter_lidar.xacro':'LiDAR 传感器模板，配置扫描几何、量程和发布接口，作为当前障碍感知输入。',
             'otter_gazebo_thruster_config.xacro':'将推进器名称及模型配置展开为插件可读取的单个推进器条目。',
             'otter_gazebo_thruster_plugin.xacro':'向船模挂载推进器插件，由插件把接收的命令施加到仿真刚体。',
            }
            header=labels[path.name]
        anchors={'<inertial>':'质量和惯性张量影响动力学；与仅用于显示的网格外观分别配置。',
                 '<collision':'碰撞几何参与物理接触检测，不一定与显示网格完全相同。',
                 '<visual':'外观几何与材质只负责显示，不能据此认定物理碰撞尺寸。',
                 '<sensor':'传感器更新率、测量范围与插件输出在此配置。',
                 '<plugin':'加载指定动态库并传入参数；具体数值处理由对应插件源码完成。',
                 '<physics':'物理仿真的步长与更新设置影响仿真时间推进，不等于航控算法的控制频率。',
                 '<spherical_coordinates>':'定义场景地理参考；局部米制坐标与经纬度转换应采用同一参考。',
                 '<xacro:macro':'定义可复用宏；调用时替换参数并展开为实际 URDF/SDF 元素。'}
        return insert_lines(original,header,xml=True,anchors=anchors)
    if path.suffix in {'.yaml','.rviz'}:
        if path.suffix=='.rviz':
            header=('当前 ROS 2 RViz 显示布局，配置固定坐标系、话题显示和交互工具。' if path.name=='sim_env.rviz' else '保留的 RViz 布局资源，含历史插件标识；当前主入口使用 sim_env.rviz。')
        elif path.name=='user_config.yaml':header='场景选择与生成器输入；当前导航算法选择以共享算法 YAML 和 launch 覆盖为准。'
        elif path.name=='obstacles_config.yaml':header='障碍物生成清单：描述形状、位姿与尺寸，由动态 XML 工具转换为场景元素。'
        elif 'scripts/validation/config/' in relative:header=f'独立对比试验配置 {path.stem}，用于显式选择的参数诊断；不会自动替代主线默认配置。'
        elif path.name=='ekf_params_otter.yaml':header='保留的 robot_localization EKF 配置；普通导航当前使用仿真真值定位，不能仅因文件存在认定 EKF 已启用。'
        elif path.name=='gps_params_otter.yaml':header='保留的 GPS 到局部里程计转换配置；普通导航当前没有接入真实 GPS 定位链。'
        elif path.name=='trajectory_params_otter.yaml':header='历史轨迹时域配置；当前 LOS 主线参数位于 shared/usv_navigation/config/trajectory_planner。'
        else:header='历史算法配置模板；当前主线仅接入 A*/Dijkstra/GBFS、LOS、PID/LQR，实际有效参数以共享配置为准。'
        return insert_lines(original,header,anchors={
         'controller:':'以下层级沿用项目 YAML 格式；参数键的历史拼写保持兼容。',
         'control_frequncy:':'控制频率，单位 Hz；保留原参数名的历史拼写。',
         'lookahead_time:':'控制参考的时间前视量，单位秒，不能与 LOS 的米制前视距离混用。',
         'Q_matrix_diag:':'LQR 状态误差权重对角项，排列必须与控制器的状态向量一致。',
         'R_matrix_diag:':'LQR 输入代价权重对角项，影响控制输出大小与跟踪误差之间的折中。',
         'pose:':'位姿字段按生成器约定解释；改变场景后还需核对起点与目标是否处于可通行水面。',
         'Panels:':'界面面板布局，与实际导航计算和控制参数无关。',
         'Visualization Manager:':'显示对象、固定坐标系与工具设置；话题需与运行中的命名空间一致。'})
    if path.name=='CMakeLists.txt':
        header='配置仿真资源及定位节点的构建与安装；运行时通过安装区查找 launch、world 和船模。' if 'sim_env' in path.parts else '构建调用原 C++ 算法库的数值对照程序，供 Python/C++ 一致性检查使用。'
        return insert_lines(original,header,anchors={'find_package(':'查找构建所需依赖，名称与 package.xml 中的声明对应。','install(':'将构建产物或资源放入 ROS 包安装目录，源码修改后需重新安装才能被 launch 读取。'})
    if path.suffix in {'.sh','.bash'}:
        headers={
         'build.sh':'构建工程并汇总编译数据库，便于 clangd 解析 C++ 包含路径；使用前先加载 ROS 环境。',
         'main.sh':'加载 ROS 与指定安装区，准备 Gazebo 资源后启动普通导航主入口。',
         'killpro.sh':'按完整命令行匹配并强制结束仿真相关进程；匹配范围包含通用 Gazebo/RViz 名称。',
         'build_clean.bash':'在独立输出目录构建源码并记录过程，避免把旧安装产物误当作新构建结果。',
         'run_distributed_source_tests.bash':'加载选定安装区并运行完整源码测试，保存命令、日志及 JUnit 结果。',
         'run_source_tests.bash':'加载 ROS 并配置源码导入路径，执行 pytest；附加命令行参数直接转交 pytest。',
        }
        return insert_lines(original,headers[path.name],anchors={
         'set -eo pipefail':'让失败命令或管道失败传播为非零退出，避免后续步骤掩盖前一步错误。',
         'colcon build':'真正执行包构建；后续 Python 段只汇总编译数据库，不修改算法。',
         'source /opt/ros/humble/setup.bash':'先加载系统 ROS，再加载工作区 overlay，使包与消息类型按选定环境解析。',
         'USV_SELECTED_INSTALL=':'允许通过环境变量选择外部安装区，未指定时使用项目内 install。',
         'if [ -f "src/user_config/user_config.yaml" ]':'此步骤会按场景配置尝试生成障碍；生成失败沿用现有脚本的继续启动行为。',
         'exec ros2 launch':'用 launch 替换当前 shell，并原样传递所有参数，保留含空格的参数边界。',
         'PATTERNS=(':'以下是子串匹配规则而非本次任务的进程组标识，可能命中其他同名仿真实例。',
         'for _ in 1 2; do':'执行两轮清理以处理启动器退出时残留的子进程；此脚本使用 SIGKILL 强制结束。',
         'export PYTEST_ADDOPTS=':'该默认发现路径用于导出工作区的子测试；外层显式收集 src/scripts，不缩减全量范围。'})
    if path.suffix=='.ps1':
        return insert_lines(original,'按标题定位唯一 RViz 窗口并保存截图；只在用户明确运行脚本时捕获，不覆盖已有文件。',anchors={
         "Add-Type @'":'下面的 C# here-string 封装 Win32 窗口查询接口，其正文保持原样。',
         '$rvizWindows =':'要求唯一匹配，避免把其他应用或另一仿真窗口保存为本次证据。',
         '$rvizBitmap =':'按实际窗口矩形分配图像；后续 finally 无论成功失败都释放图形资源。'})
    if path.suffix=='.cpp':
        if path.name=='golden.cpp':
            return insert_lines(original,'数值对照可执行程序：直接调用既有 C++ 算法库，不在此重新抄写算法公式。',prefix='//',anchors={'int main(':'读取确定性的测试输入并输出结果，供 Python 对照脚本进行误差比较。'})
        return insert_lines(original,'仿真真值定位：将 Gazebo 船模状态转换为平面 odom 与 TF，供导航节点使用。',prefix='//',anchors={
         'class FakeLocalizationNode':'订阅模型状态并发布定位；这是仿真真值适配，不包含真实传感器融合或定位滤波。',
         'const std::string prefix':'坐标系名称随节点命名空间统一加前缀，保持 map、odom、base_link 的对应关系。',
         'if (msg->name.size()':'消息三组数组长度必须一致，先检查边界再按同一下标取模型状态。',
         'if (have_last_ && stamp == last_stamp_)':'同一仿真时刻不重复发布；时间回退视为新纪元，沿用现有逻辑允许重新开始。',
         'geometry_msgs::msg::TransformStamped map_to_odom;':'此工程令 map 到 odom 为恒等变换；不能据此假定其他定位系统也如此。',
         'const double yaw =':'提取艏向，把 Gazebo 的世界系线速度旋转到 Odometry 要求的船体系。',
         'odom.twist.twist.linear.x =':'船体系 x 向前、y 向左；投影后的纵横向速度分别进入控制状态 u、v。',
         'int main(':'初始化 ROS，持续处理定位回调，退出时释放节点与通信上下文。'})
    if path.name=='pytest.ini':return insert_lines(original,'限定主线源码与验证脚本的测试入口；importlib 模式避免同名测试文件相互覆盖。')
    raise ValueError('Unhandled local file: '+relative)

def main():
    rows=json.loads((WORK/'inventory.json').read_text(encoding='utf-8'))
    done=[]
    for row in rows:
        rel=row['path']
        if not (rel.startswith(('src/sim_env/','src/user_config/','scripts/')) or rel in {'conftest.py','pytest.ini'}):continue
        path=ROOT/rel;before=(WORK/'before'/rel).read_bytes()
        assert path.read_bytes()==before,'Concurrent edit: '+rel
        python=path.suffix=='.py' or path.name=='sim_env_node'
        after=insert_python(path,before) if python else non_python(path,before)
        path.write_bytes(after)
        done.append({'path':rel,'added_lines':len(after.splitlines())-len(before.splitlines())})
    (WORK/'local-changes.json').write_text(json.dumps(done,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Annotated',len(done),'files;',sum(x['added_lines'] for x in done),'Chinese comment lines added.')

if __name__=='__main__':main()
