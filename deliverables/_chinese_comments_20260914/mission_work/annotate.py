import ast
import difflib
import json
from pathlib import Path

ROOT = Path.cwd()
BASE = ROOT / 'deliverables/_chinese_comments_20260914'
M = 'src/core/mission_module/usv_mission/'
I = 'src/core/mission_module/usv_mission_interfaces/'
P = 'src/core/perception_module/perception/'
U = 'src/core/perception_module/usv_perception/'
plans = {}

def add(path, header, functions=None, anchors=None):
    plans[path] = (header, functions or {}, anchors or {})

add(M+'usv_mission/configuration.py', '任务应用的配置入口：合并工程 YAML、兼容旧参数名并校验启动及在线更新参数。', {
    'merge_explicit': '合并显式来源；同名参数值不一致时拒绝，避免由加载顺序悄悄决定配置。',
    'flatten': '把嵌套字典展开为 ROS 点分参数名，同时检查键类型和重复定义。',
    'normalize_parameters': '统一历史拼写别名；LQR 矩阵转浮点列表以匹配 ROS double array 声明。',
    'validate_parameters': '同时检查数据类型、有限值、可用算法及模型约束，返回前不写入节点状态。',
    'resolve_parameters': '从完整默认值生成已验证快照；未知参数直接报错以暴露配置拼写错误。',
    'section': '按前缀提取某个算法的配置，并去掉上层命名空间。',
    'load_yaml': '使用拒绝重复键的安全加载器读取工程配置；ROS 参数导出快照不能直接混用。',
    'load_project_parameters': '依次合并动力学、控制、轨迹、路径与可选任务参数，地理原点由启动入口单独提供。',
    'validate_update': '仅允许白名单增益在线修改；先验证候选副本，失败时保留当前有效配置。',
}, {'DEFAULTS = {': '时间参数采用秒，频率采用 Hz；平面位置及船体尺寸采用米，ROS 航向与角速度采用弧度及弧度/秒。',
    'HOT_PARAMETERS = frozenset(': '在线更新限于不改变算法实例、定时器频率和几何契约的参数。'})
add(M+'usv_mission/control_authority.py', '控制发布权保护：逐个检查实际推进器话题，发现竞争发布者后锁存冲突直至节点重启。', {
    'ControlAuthority': '本地 ROS 图观测门控，不是分布式执行器租约；未知图状态也禁止输出。',
    'observe': '每个输出必须恰有一个发布者；数量大于一时永久锁存本次进程的冲突。',
    'unavailable': '图查询异常只更新等待原因，不清除已确认的发布权冲突。',
    'status': '导出发布者数量、拒绝原因及是否需要重启，供状态话题诊断。'})
add(M+'usv_mission/runtime.py', '规划运行时：按任务意图分派规划，并以一个执行任务加一个可替换待办限制积压。', {
    'fresh': '时间戳必须有限且落在允许时窗；明显来自未来的数据也视为失效。',
    'readiness': '控制前依次检查状态、地图、参考轨迹的新鲜度及坐标系一致性。',
    'plan_navigation': '普通目标进入基础导航链，携带任务意图时进入任务专用规划。',
    'LatestPlanWorker': '单进程执行较重搜索；主 ROS 回调只提交快照和轮询结果。',
    'offer': '新请求覆盖尚未执行的待办，优先处理最新状态而非排长队。',
    '_start': '只有当前任务空闲时才提交待办，保证同时最多一个规划搜索。',
    'poll': '取回已完成结果；同版本普通航路可转交给待办，待办仍保留其最新状态和地图。',
    'invalidate': '丢弃待办并尝试取消未启动任务；已经执行的结果仍须由节点版本号过滤。',
    'close': '关闭工作进程前清理待办，并等待正在执行的任务结束。'})
add(M+'usv_mission/navigation.py', '普通目标导航链：校验或重搜全局路径，再生成 LOS 轨迹并检查整船扫掠区域。', {
    'plan_navigation': '读取规划快照并返回携带请求版本、搜索统计和耗时的结果，位置为米、航向为弧度。',
}, {'        connected = np.vstack((request.state.pose, route)) if len(route) else route': '复用航路前，把当前实测船位连接到剩余路径，避免只检查旧航路而漏掉新连接段。',
    '        endpoint_offset = float(np.linalg.norm(route[-1, :2] - request.goal[:2]))': '栅格终点可能偏离精确目标；需使 LOS 停止圆完全落在控制器的真实目标容差圆内。',
    '        if not footprint_path_is_clear(request.grid, trajectory.points[:, [2, 3, 4]],': '轨迹第 2、3、4 列分别为 x、y、yaw；使用船体矩形检查致命和未知栅格。'})
add(M+'usv_mission/planning_types.py', '规划进程交接数据：请求携带状态、地图、参数与任务意图，响应携带参考轨迹及诊断。', {
    'PlanningRequest': '冻结字段绑定形成交接对象；内部数组和字典的隔离依赖调用方复制快照。',
    'PlanningResponse': 'version 用于拒绝过期结果；elapsed 为秒，selected_goal 保存实际选中的任务候选点。'})
add(M+'usv_mission/missions/models.py', '任务数据模型：对外接收经纬度任务，对内生成可跨进程传递的局部规划意图。', {
    'MissionSpec': '纬经度和真航向用度；速度为米/秒，半径与精度为米，持续时间为秒。',
    'validate': '检查任务类型、数值范围和船速限制；绕圈还必须满足速度/半径不超过最大偏航角速度。',
    'MissionIntent': '保存本轮意图类型、阶段、版本及目标或圆轨迹，候选点由规划进程进一步筛选。',
    'signature': '修订号、意图类型和目标共同决定参考是否已切换。'})
add(M+'usv_mission/missions/geometry.py', '任务几何筛选：检查船体及转向包络，生成绕圈切入点和近场安全退让候选点。', {
    'pose_is_clear': '在船长和船宽两侧增加 margin 后检查矩形足迹，单位均为米。',
    'turning_envelope_is_clear': '用正方形包住扩展矩形任意旋转及到达误差圆，预留退让终点的转向空间。',
    'entry_candidates': '外部船位增加两个精确切点，并按 15 度采样圆周；恢复模式沿指定方向排除当前径向点。',
    'escape_candidates': '在 4 米、8 米两圈各取 16 个方向，只保留转向包络安全的点，再按转角与距离排序。'})
add(M+'usv_mission/missions/manager.py', '无 ROS 依赖的任务状态机：负责待命、绕圈、定点保持及安全抢占，重型搜索交给规划进程。', {
    'MissionManager': '保留原任务目标与圈数，临时退让目标和起步前序单独管理。',
    '__init__': '校验任务和局部目标，建立初始停止意图以及驻留、圈数、退让恢复计时状态。',
    '_emit': '仅当意图类型、阶段或目标改变时递增修订号，通知适配器作废旧参考。',
    '_fresh_inputs': '先确认状态和地图新鲜且同坐标系，失效时返回可诊断的等待原因。',
    '_reset_progress_anchor': '取消上一次圆周积分锚点，防止跨越脱轨或避障阶段补计圈数。',
    '_clear_departure': '清除制动、倒退及转向前序；可选择保留已完成前序的去重记录。',
    '_prepare_departure': '按实测停止、可选倒退、原地对准顺序执行；完成后仍要重新规划到原安全目标。',
    '_departure_settled': '位置、5 度航向误差、平移速度和角速度均达标，并持续 0.5 秒不同时间戳里程计才放行。',
    '_at_goal': '用平面欧氏距离判断是否进入任务要求的到达圆。',
    '_settled_at_center': '中心到达且平面速度不超过 0.1 米/秒保持一秒，才认定进入待命。',
    '_resume_nominal': '解除安全抢占并清除临时目标，重置积分锚点后恢复原任务。',
    '_safety': '安全检查优先于正常任务：船体碰撞即停止，近场入侵触发退让，持续净空后才恢复。',
    'tick': '每次使用当前仿真时间和最新观测推进状态，先处理时钟、数据失效与安全抢占。',
    'on_plan': '仅接收适配器已核对版本的响应；成功保存候选目标，失败按当前意图选择阻塞或退让。',
    'request_escape': '控制器预测到危险时可主动抢占保持或绕圈，重新选择安全出口。',
    'on_arrival': '控制器到达提示不直接结束任务，实际完成仍由下一次 tick 的观测条件决定。',
    'status': '输出原目标、临时目标、位置/航向误差、净圈数和阻塞原因，供任务监控使用。',
}, {'        if self._clock_invalid:': '时钟回退会破坏已累计驻留及任务时间，因此本实例锁定停止状态。',
    '        invaded = not pose_is_clear(grid, state.pose, self.config, margin=3.)': '3 米扩展矩形用于触发近场避让；恢复采用更大的 5 米包络形成滞回。',
    '                elif self._safe_stamp is None or grid.stamp > self._safe_stamp:': '恢复净空必须由不同时间戳的地图连续证实，重复发布同一地图不增加证据。',
    '        if self.spec.mode == \'station_keep\':': '定点保持先用导航接近目标，进入近区后锁存保持模式以避免边界来回切换。',
    '                if elapsed > 0 and abs(increment)*self.spec.orbit_radius <= allowed:': '仅累计物理速度允许范围内的有符号圆周进度；反向抖动会抵消，瞬移不计数。',
    '            if (isinstance(backoff, bool) or not isinstance(backoff, (int, float))': '倒退距离属于条件规划证据，必须有限且在 0 至 3 米范围内，并与起步朝向一起提供。'})
add(M+'usv_mission/missions/planning.py', '任务规划进程：在统一搜索时限内验证圆轨迹、切向入圈和退让候选，并返回执行前置条件。', {
    'mission_parameters': '复制任务专用速度与精度参数，避免改写普通导航或调用方快照。',
    '_route_cost': '路径长度加最小转弯半径乘累计转角，兼顾绕路距离与朝向调整代价。',
    '_route_after_backoff': '倒退后从新的停止位置重新连接原目标；绕圈退让保留切向接近几何。',
    '_departure_alignment': '验证制动扫掠、可选倒退走廊、原地旋转及后续导航，返回有条件的可行起步方案。',
    '_is_los_collision': '仅识别 LOS 整船扫掠碰撞，以触发专门的停止和对准候选搜索。',
    'plan_mission': '按意图生成参考；全体候选共享一个截止时刻，失败响应不携带可执行轨迹。',
}, {'    for backoff in (0., .5, 1., 2., 3.):': '先尝试原位对准，仅在继续航行不可行时按由短到长顺序验证倒退距离。',
    '            metadata = {\'departure_pose\': aligned.pose, \'departure_requires_replan\': True,': '这里的轨迹起点是假设停止并对准后的状态，元数据明确要求实测完成前序后重新规划。',
    '        if intent.kind == \'circle\':': '绕圈使用连续圆参考，没有有限航路终点；仍需检查预测轨迹整船扫掠。',
    '        if intent.kind == \'escape\':': '退让最多验证前四个几何候选，限制紧急响应阶段的搜索负担。',
    '            if intent.kind == \'orbit_entry\' and lower_bound >= best_cost:': '入圈候选按直线距离排序，距离下界已超过最好总代价时可提前结束。'})
add(M+'usv_mission/node.py', '任务 ROS 主节点：接收目标、里程计和代价地图，协调异步规划、控制发布与任务服务。', {
    'seconds': '把 ROS 秒/纳秒时间戳换算为浮点秒，统一新鲜度检查的单位。',
    'yaw': '由四元数提取平面航向，统一复用公共角度校验逻辑。',
    'MissionNode': '持有控制器、规划工作进程及任务适配器；本节点是任务链推进器命令发布入口。',
    '__init__': '读取有效配置，建立带船舶命名空间的话题/坐标系，并分别启动控制与规划定时器。',
    'now_seconds': '读取节点 ROS 时钟；仿真模式下由 /clock 驱动。',
    'command_authority_ready': '对重映射后的真实推进器话题检查发布权，首次竞争同时作废在途参考。',
    '_validate_parameters': '参数提交前只做候选校验；活动任务还需满足新参数下的任务可行性约束。',
    '_apply_parameters': '在控制周期应用已提交增益；轨迹参数变化时作废旧版本参考。',
    '_pose': '按消息时刻的 TF 将平面位姿转入 map；旋转先作用于位置，再加平移。',
    '_on_odom': '仅保留最新待处理里程计，并立即尝试坐标变换。',
    '_consume_odom': '位姿统一到 map，速度要求在 base_link 船体系；TF 暂缺可重试，非法数据直接丢弃。',
    '_on_map': '把 Nav2 无符号代价数组转换为二维快照；最新地图一旦否决轨迹就清空参考并请求零推力。',
    'trajectory_clear': '同时验证实测船体足迹和参考轨迹，避免只看规划中心线。',
    'current_footprint_clear': '用当前实测位置与航向检查整个矩形船体。',
    'reset_task': '递增版本、清理任务与控制器并发布空路径；必要时连同状态、地图和 TF 一起清空。',
    '_on_goal': '接收普通目标时先结束旧任务；新目标坐标变换失败也不会继续执行旧任务。',
    '_cancel': '取消任务后返回实际发布权状态，冲突时不会对竞争控制器发布命令。',
    '_export': '导出当前 ROS 参数快照，便于核对实际使用的数值。',
    '_planning': '仅在输入新鲜且发布权明确时向工作进程提交普通目标或任务意图。',
    '_consume_plan': '先过滤旧版本，再用当前地图复核返回轨迹，成功后发布全局路径及局部参考。',
    '_publish_command': '所有正常、取消、复位和关闭路径统一经过发布权门控。',
    '_control': '稳态时钟驱动控制回调；使用 ROS 时间计算 dt，暂停或输入失效时保持零命令。',
    '_publish_status': '以 JSON 发布状态、计数、耗时分位数和进程资源；毫秒用于耗时统计。',
    'destroy_node': '在上下文仍有效时请求零输出，再关闭规划进程和 ROS 节点。',
    'main': '自行处理退出信号，使最后一次零命令有机会在 ROS 上下文关闭前发布。',
}, {'            data = np.asarray(message.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)': '数组按行存储为 [y, x]；此处需要地图宽高与消息长度严格匹配。',
    '        dt = now - self.last_control': '定时器墙钟仍在前进而 ROS 仿真时钟暂停时，dt 可为零，不能推进控制积分。'})
add(M+'usv_mission/mission_adapter.py', '任务与 ROS/控制器之间的适配层：把地理请求变成本地状态机，并管理导航、保持及安全制动交接。', {
    'MissionAdapter': '任务服务、标记可视化和任务控制均依附同一个 MissionNode。',
    '__init__': '创建 SetMission 服务及持久化任务标记发布器，初始没有活动任务。',
    'active': '以状态机实例是否存在判断任务是否活动，停止意图仍可能属于活动任务。',
    'clear': '清除活动任务和控制器积累状态，同时删除 RViz 任务标记。',
    'set_mission': '先完成任务、地理转换和控制器构造；验证全部成功且发布权仍有效后才替换旧任务。',
    'input_state': '任务控制前检查状态和地图新鲜度及 frame_id 一致性。',
    'prepare': '推进状态机；意图签名变化时提升参考版本、取消待办并重置控制器。',
    'planning': '停止、保持和制动由控制侧处理；其余意图带任务专用快照提交规划进程。',
    'consume_plan': '确认版本和最新地图安全性后交接参考；条件起步方案不能直接当作可执行轨迹。',
    'safe_braking_command': '按候选次序选择已通过执行与停止扫掠校验的制动命令，全部失败则返回零命令。',
    'control': '按意图选择制动、停止、定点保持或导航控制，危险预测可再次触发安全抢占。',
    'status': '合并状态机报告、保持控制阶段以及控制侧安全拒绝原因。',
    'publish_markers': '在 map 中画原目标、观察/任务半径、目标朝向和当前阶段标签。',
}, {'            target = (x, y, heading_to_yaw(spec.heading_deg))': '地理坐标转换为 ENU 米制位置；真北为零、顺时针为正的航向转换为 ROS 平面 yaw。',
    '            self.holding.observe_command(getattr(n, \'last_published_command\', ControlCommand()))': '交接时继承真正已发布命令，避免保持控制器从虚构零值计算推力变化率。',
    '        if metadata.get(\'departure_requires_replan\') or metadata.get(\'departure_pose\') is not None:': '预测停止位置只证明后续可能可行；实测制动、倒退与转向完成后必须重新规划。',
    '            if not clear_control(n.state, command, n.grid, n.model, dt, **options):': '定点保持也会平移和旋转，因此每条命令必须通过完整船体运动及停止预测。'})
add(M+'usv_mission/mission_cli.py', '任务命令行客户端：本地校验 YAML 后执行 start、status 或 cancel，并输出机器可读结果。', {
    'mission_request_values': '检查必填键、类型和基本范围；与实际船舶模型有关的速度/转弯限制仍由服务端校验。',
    'load_mission': '读取拒绝重复键的任务 YAML，再补充接口默认值。',
    'make_parser': '命令行命名空间与响应超时放在子命令前；start 接收任务 YAML 路径。',
    'main': '先完成无需 ROS 的参数检查，再按统一墙钟截止时刻等待服务或状态消息。'})
add(M+'usv_mission/launch_support.py', '任务启动组合：复用只读算法配置及仿真资源，明确单独任务入口与完整仿真入口。', {
    'geographic_parameters': '显式地理参考 YAML 与 world 文件二选一；缺省时禁用地理任务，显式零原点仍合法。',
    '_value': '在 launch 上下文中求出参数最终字符串值。',
    '_boolean': '只接收 true/false 字符串，避免非空字符串误当布尔真。',
    '_namespace': '去掉命名空间两端斜线，并拒绝空船舶名称。',
    '_parameters': '读取工程算法配置、合并地理参考及启动覆盖值，然后再次统一校验。',
    '_mission_node': '用当前 Python 解释器创建唯一任务节点，并传入明确的时钟模式。',
    'mission_setup': '只组合任务节点，状态、TF 与地图由外部 ROS 图提供。',
    'simulation_setup': '组合 Gazebo、机器人状态、定位、建图、代价地图及任务节点，headless 同时关闭图形界面。',
    'common_arguments': '声明任务与仿真共用的算法选择、船舶命名空间和配置文件路径。',
    'simulation_arguments': '读取用户世界偏好，并声明出生位姿与 GUI/RViz 参数。',
}, {'    plugins = costmap_params.get("plugins")': '只检查显式列出的有效图层，启动契约要求静态层不参与本组合。'})
add(P+'perception/mapping.py', '纯数值激光建图内核：按光束顺序累积整数占用证据，可在没有 ROS 的环境中测试。', {
    'GridConfig': '正方形栅格宽度按格计，分辨率为米/格，偏移量为地图中心在世界坐标中的米制位置。',
    '__post_init__': '构造后校验正整数宽度、正分辨率和有限偏移，提前阻止非法地图几何。',
    'UpdateStats': '返回输入、有效、命中和无效光束数量，以及实际执行的栅格更新次数。',
    '_trace_rays': '先裁剪连续栅格射线，再用 Bresenham 顺序遍历；重复经过的格子必须逐次更新。',
    'GridMapper': '持有有符号 int8 证据网格及世界原点，当前只安装 Python 参考内核。',
    '__init__': '地图以偏移量为中心展开，左下角原点减去半幅物理宽度，未知栅格初值为 -1。',
    'reset': '将全部证据恢复为未知，供时钟回退后的新一轮观测使用。',
    'world_to_grid': '先减世界原点再除分辨率，向下取整得到整数列和行。',
    'integrate_scan': '验证扫描元数据，按传感器位姿将极坐标回波转成地图射线后调用顺序积分内核。',
}, {'        real_hit = hits[index] and 0.0 <= ex < width and 0.0 <= ey < width': '只有真实命中且原始端点在图内才标障；裁剪产生的边界端点不能变成假障碍。',
    '            value = int(data[cell]) + 55': '命中加 55 并饱和到 100；未知格 -1 首次命中得到 54，这不是标定后的概率。',
    '            value = int(data[cell]) - 25': '射线穿越减 25 并限制下界为 0，保留光束顺序对共享格子的影响。',
    '        hit = np.isfinite(values) & (values >= range_min) & (values < range_max)': '有限且位于量程内的是命中；正无穷及达到上限的返回只清空沿途，NaN/负值等被忽略。',
    '            lengths = np.where(hit, values, min(range_max, 30.0))': '无有效回波时清空距离最多 30 米，真实命中仍保留其测得距离。'})
add(P+'perception/scan_queue.py', '扫描等待队列：用传感器时戳去重、用单调墙钟限制 TF 等待，并单独统计真实队列观测。', {
    'PendingScan': 'stamp_ns 是 ROS 传感器纳秒时戳，received_at 是接收时的单调墙钟秒数。',
    'ScanResult': '回调显式区分等待、成功积分、超时和非法扫描消费，防止诊断计数混淆。',
    'QueueCounters': '记录接纳、处理、溢出、超时、重复、非法输入及复位，便于检查消费守恒。',
    'ScanQueue': '固定容量 FIFO；等待队首 TF 时不重排光束帧，满队列时丢掉最旧待办。',
    'enqueue': '拒绝零/旧时间戳和空坐标系；按 frame 分别去重，避免零时戳意外请求最新 TF。',
    'drain': '按到达顺序处理，在扫描之间和 TF 等待后重查截止时间；仅成功积分计入 completed。',
    'expire_pending': '在指定墙钟检查点真正移除过期项，随后可用同一时刻记录队列快照。',
    'observe_pending': '记录处理后的队长、最老扫描年龄和身份；诊断采样不改变接纳或超时规则。',
    '_prune_observations': '只保留最近五秒诊断窗口内的真实观测。',
    'observation_summary': '汇总真实观测的年龄分位数和扫描身份，区分空队列、无样本及同一扫描卡住。',
    'reset': '清空待办、去重历史和诊断窗口，允许新时钟纪元重新接纳较小扫描时戳。',
    'ClockGuard': '记住上一轮 ROS 时戳，暂停不算故障，任何回退都会通知上层复位。',
    'observe': '更新比较锚点并报告本次是否发生时间回退。'})
add(U+'usv_perception/mapping_node.py', '激光建图 ROS 适配器：严格按扫描时刻查询 TF，把结果交给数值内核并发布 OccupancyGrid。', {
    'MappingNode': '节点组织扫描队列、TF、栅格发布与诊断；几何积分仍由 perception 包实现。',
    '__init__': '启动参数只读；稳态时钟负责队列期限，ROS 时钟负责地图发布时间。',
    'startup': '声明只在启动时允许设置的参数，并返回最终覆盖后的值。',
    '_observe_clock': '时钟回退后清空地图、队列、TF 缓存及耗时窗口，避免跨纪元混用观测。',
    '_on_scan': '接纳最新合法扫描并统计实际扫描周期，暂不在订阅回调中做阻塞 TF 等待。',
    '_drain': '处理待办后，在同一个墙钟检查点执行最终过期清理并记录诊断快照。',
    '_try_process': '以 timeout=0 查询精确传感器时刻的 TF，成功但已经超期的扫描也不能修改地图。',
    '_publish_map': '按行序列化有符号字节网格，避免为百万格地图创建百万个 Python 整数。',
    '_log_metrics': '发布处理、排队、序列化分位数和丢帧计数；这些墙钟诊断与 ROS 时间戳分别记录。',
    'main': '创建并运行节点，退出时清空待处理扫描并有序关闭 ROS。',
}, {'        if start - pending.received_at >= self.pending.tf_wait_timeout:': '即使非阻塞查询也可能被调度延迟跨过期限，修改网格前必须再次确认准入时间。',
    '            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm': '先归一化 TF 四元数再求 yaw，避免缩放误差影响激光投影角度。'})
add(U+'usv_perception/perception_monitor.py', '感知模块进程身份监控：观察输出发布端点，发布模块心跳以通知下游感知链是否重启。', {
    'PerceptionMonitor': '发布者存在性决定 ready，端点身份变化切换 session；观测内容新鲜度由消费者另行检查。',
    '_poll': '短暂消失只撤销就绪状态；出现新的端点身份才生成新会话并重置序列号。',
    'main': '交给公共节点运行器处理感知监控节点的启动和退出。'})
add(U+'usv_perception/launch_support.py', '独立感知启动支持：读取本包建图和代价地图参数，组合建图、Nav2 地图和进程监控三个节点。', {
    '_load_yaml': '安全读取工程 YAML，拒绝重复键、非字典根以及 ROS 参数导出快照。',
    '_value': '在 launch 上下文中取得最终参数字符串。',
    '_launch_setup': '验证命名空间和图层配置，再将统一坐标系、扫描话题及出生偏移传入节点。',
    'launch_description': '声明感知所需的配置路径与时钟/命名空间参数，实际节点由 OpaqueFunction 延迟创建。',
}, {'    plugins = costmap.get("plugins")': 'plugins 才是实际装载清单；未列出的 static_layer 段不会启用，显式启用静态层则拒绝启动。'})
add(U+'scripts/usv_tf_expander.py', '仿真 TF 补充节点：周期发布带船舶命名空间的 map 到 odom 单位变换。', {
    'UsvTfExpander': '本仿真没有独立地图校正，map 与 odom 重合；定时发送使地图坐标系尽早可查询。',
    'publish_map_to_odom': '使用当前 ROS 时刻发布零平移、单位旋转，父子坐标系均带 robot_namespace。',
    'main': '运行 TF 广播节点，退出时释放节点与 ROS 上下文。'})

add(M+'validation/mission_acceptance.py', '任务闭环验收采集：运行真实 ROS/Gazebo 场景，以里程计、推进器命令和图状态形成测量证据。', {
    'TrialFailed': '验收要求未满足时使用的失败类型，与环境尚未准备好的 blocked 区分。',
    'scenario_config': '为定点保持、待命和顺/逆时针巡逻指定固定初始条件，局部目标采用米和弧度。',
    'sample_errors': '计算位置距离、最短航向误差及船体系平移速度，航向报告换算为度。',
    'fixed_window_metrics': '只评分第一次指定的固定时间窗，偏离与丢测都保留，不能重启窗口挑选较好片段。',
    'SettleDetector': '连续五秒满足收敛位置、航向及低速条件，才能开始正式保持评分窗口。',
    'DwellDetector': '通用观测驻留判据：数据缺口、非递增时间或不满足条件都会打断连续驻留。',
    'at_centre': '返回中心必须同时满足 STANDBY 标签、位置精度及低速观测。',
    'at_safe_waiting_point': '安全等待要证实船已到选定临时目标、速度低且整船与受控障碍至少间隔 0.2 米。',
    'independent_orbit_metrics': '从原始里程计独立积分有符号圈数，不跨状态切换、脱轨、缺测或非物理跳跃累计。',
    'orbit_rejoined_after_obstacle': '使用整船穿过障碍下游切向平面的证据及当前圆上运动，确认真正越障并恢复原圆。',
    'observed_lateral_impulse': '在服务请求到响应后短窗内寻找实测横向速度峰值，不能仅凭位置变化认定扰动已注入。',
    'preflight_failures': '试验包络内必须有可用地图且没有致命/未知格，避免环境障碍污染受控场景。',
    'MissionLog': '逐事件落盘并同步构建任务样本，发现非法推力、非有限状态或负扫掠间距即停止采集。',
    'advance': '推进 ROS 采集直到仿真时长或判据满足，同时用墙钟上限识别停滞。',
    'require_phase': '在限定仿真时长内要求观察到指定阶段，超时记录验收失败。',
    'measured_dwell': '按游标消费新增里程计样本，以独立驻留判据确认任务事件。',
    'measured_centre_return': '以实际中心位置和低速驻留核对返航完成。',
    'station_window': '先等待收敛，再采集首个完整 60 秒保持窗口。',
    'set_vessel_state': '经 Gazebo 服务设置受控位姿/世界系速度，并记录请求、响应和服务延迟。',
    'inject_lateral_impulse': '把船体系 0.4 米/秒横向速度转到世界系；服务延迟和实际速度观测分别验收。',
    'spawn_obstacle': '在指定相对位置生成受控圆柱障碍，生成前检查当前完整船体间距。',
    'obstacle_observed': '确认障碍附近地图中有致命占用；未知格不能冒充已经探测到障碍。',
    'remove_obstacle': '仅删除本次验收生成且持有名称的障碍，记录删除原因与服务结果。',
    'controlled_avoidance': '保持障碍存在直到实测确认退让恢复，再进入清理阶段。',
    'fingerprint': '对源码和安装资源分别计算 SHA-256，以核查测量前后使用的程序是否一致。',
    'run_trial': '单场景拥有独立进程组、ROS 域与 Gazebo 端口，预检后采集并在 finally 内清理。',
    'main': '验证试验矩阵并保存清单；dry-run 只记录计划，实际执行后才汇总测量通过与否。',
}, {'    allowance = min(0.15, 2.0*max_speed/radius*largest_boundary)': '状态约一秒发布一次，阶段边界存在采样不确定性；全局容差最多 0.15 弧度，不能靠丢帧放宽。',
    '    if args.dry_run:': '这里只生成预定场景，all_passed 保持 False，不能把调度成功解释为闭环实验通过。'})
add(M+'validation/task_ros_acceptance.py', '基础导航与故障验收工具：采集真实 ROS 数据，验证到达、航迹偏差、扫掠间距及取消/里程计失效停车。', {
    'TrialBlocked': '缺少安装、服务、输入或环境预条件时标记试验阻塞，避免混作算法测量结果。',
    'EmptyParameterResponse': '单独标记控制器名称暂未返回，可进行有界重试。',
    'sanitized': '递归将 NaN/无穷值换成 JSON null，保持严格 JSON 文件可解析。',
    'Jsonl': '逐行写事件并即时 flush，保留异常或中断之前已经收到的观测。',
    'OwnedProcess': '仅管理本工具创建的 launch 进程组，退出先交给 launch 转发信号。',
    'free_port': '临时绑定本机端口取得可用 Gazebo master 端口号。',
    'stamp_seconds': '把 ROS 秒与纳秒转为秒值，用于保留消息自身的采样时刻。',
    'parameter_value': '按 ROS ParameterValue 类型编号取出标量或数组。',
    'make_probe': '延迟导入 ROS 并创建订阅探针，记录状态、地图、路径、命令及参数事件。',
    'Probe': '把多个公开话题组成验收观测，不访问规划器内部私有状态。',
    'sim_time': '从探针 ROS 时钟取得仿真秒数。',
    'message_age': '报告消息头相对 ROS 时间的年龄，单位毫秒；无有效时戳返回缺失值。',
    'on_odom': '提取 map 平面位姿和船体系速度，保留源时戳与回调墙钟时刻。',
    'on_scan': '记录扫描回调时刻和消息头年龄，用于输入延迟诊断。',
    'on_path': '仅把接近当前目标的路径作为首次参考，记录目标到路径回调的等待时长。',
    'on_trajectory': '保存局部路径与航向，并保留最后一条非空轨迹供故障复现。',
    'on_status': '解析导航状态；首次出现参考失败时保存当时已收到的输入快照。',
    'on_mapping_status': '保存建图诊断消息，无法解析的原文也保留。',
    'on_parameter_event': '仅记录当前船舶任务节点的参数变化，便于追踪试验中实际配置。',
    'on_goal': '记录从公开目标话题观察到的请求及源时刻。',
    'on_command': '按推进器轴记录归一化推力，并统计左推进器回调间隔和首次非零输出时延。',
    'discover_costmap': '优先选择 Nav2 原始代价图，短暂等待其出现后才允许退回有损占用图表示。',
    'on_costmap': '兼容原始代价图和占用图消息，记录类型标签以使用相应障碍阈值。',
    'capture_inputs': '保存地图 NPZ 与状态/参考 JSON；各订阅源时戳分开保留，并非原子规划快照。',
    'publish_goal': '清理上次目标测量基准，保存输入后以 map 坐标系发布本次目标。',
    'zero_received_since': '三个推进器轴均须在指定墙钟时刻之后收到足够接近零的命令。',
    'wait_until': '短周期处理 ROS 回调，同时检查自有进程和控制发布权，直到谓词或墙钟期限。',
    'call_service': '按候选服务名查找并限时等待响应；必需服务不可用时显式阻塞试验。',
    'verify_controller': '只重试空参数响应；明确返回了错误控制器类型时立即拒绝试验。',
    'apply_start_parameters': '目标下发前原子设置试验覆盖参数，再读回校验并记录。',
    '_verify_controller_once': '从活动节点读取算法与动力学参数，用实际控制器到达半径替代假设阈值。',
    'preflight_corridor': '检查名义起终点之间胶囊形走廊，确保没有既有致命或未知单元干扰试验。',
    'setup_world': '在自有仿真内放置初始船位并生成场景障碍，要求新地图确实观测到它们。',
    'navigation_trial': '采集基础导航或目标替换/取消场景；几何越界立即停止采集并留下快照。',
    'odom_dropout_trial': '用受控 ROS 发布器维持新鲜地图并停止里程计，独立测量陈旧状态触发的零推力延迟。',
    'arguments': '解析工作区、场景、控制器及参数覆盖，拒绝重复覆盖键和非法 ROS 域。',
    'main': '保存试验计划和配置指纹，运行选定导航/故障组合并汇总真实采集结果。',
}, {'        # Close the acquisition interval before distribution calculations,': '正式采集结束点先落盘，后续统计、文件输出及清理耗时不应混入采集区间。',
    '        publish_odom = False': '故障注入只停里程计；地图继续更新，隔离状态超时这一原因。'})
add(M+'validation/mission_matrix.py', '任务验收矩阵调度器：限并发运行独立场景，保留子试验证据，失败后停止派发新任务。', {
    'write_json': '先写临时文件再原子替换，让外部进度读取者始终读到完整 JSON。',
    'plan_tasks': '按长场景优先排序，为每次重复分配独立 ROS 域、输出目录和子进程命令。',
    'read_child_result': '只有场景/域匹配、源码与安装指纹一致且测量及图证据通过，才提升为成功。',
    'graph_evidence_verified': '核对非空控制发布权观测、数量、规则及 manifest 中逐字一致的记录。',
    'progress': '汇总运行、通过、失败、阻塞和未派发状态；中间进度不能先宣称整组通过。',
    'interrupt_owned': '只中断持有的子试验 PID，并用全组共享截止时刻限制清理等待。',
    'run_matrix': '先回收所有完成结果再派发新任务，避免已失败时继续启动后续场景。',
    'main': '验证并发数、重复数、ROS 域范围及新输出目录，再保存矩阵清单并执行。'})
add(M+'validation/task_graph_guard.py', '验收图所有权检查：确保任务节点唯一且独占三个推进器命令话题，并记录每次检查证据。', {
    'GraphViolation': '发现图所有权违约时中止当前验收采集。',
    'node_path': '将节点名和命名空间拼成规范绝对路径，保留检查所需身份。',
    'evaluate_graph': '对公开图快照做纯函数判定，重复节点/端点、旧控制节点及额外竞争轴均可暴露。',
    'GraphGuard': '周期测量 ROS 图并累积历史；之后恢复正常也不能抹掉此前失败记录。',
    'check': '按最多每半秒一次的策略获取节点和推进器发布端点，异常判定立即抛出。',
    'evidence': '只有至少一次观测且所有历史检查通过，才返回整体 passed。',
    'activate_graph_guard': '正常启动发现完成后挂接保护并立即强制检查，失败不能靠重试隐藏。'})
add(M+'validation/task_metrics.py', '无 ROS 的验收几何与统计：米制航迹偏差、矩形船体与圆柱净距，以及边界停止判据。', {
    'wrap_angle': '把角度差包到 [-pi, pi)，用于最短方向插值。',
    'point_to_polyline_distance': '把点投影到每条有限线段并取最小距离；空路径用 None 表示缺少参考。',
    'rectangle_circle_clearance': '把障碍中心转进船体系，计算矩形到圆的有符号距离：正为分离、负为重叠。',
    'swept_clearance': '位置线性、航向最短角插值后采样，并扣除间隔误差上界；不重建未观测动力学。',
    'distribution': '过滤缺失和非有限观测后计算均值、RMS、95 分位和最大值，空样本保留缺失。',
    'trial_verdict': '综合实际到达、命令合法性、航迹误差及整船障碍间距形成带理由的判定。',
    'supervision_stop_reason': '负扫掠间距或航迹偏差越界时停止采集，与通过阈值的评分逻辑分开。'})
add(M+'validation/verify_install.py', '安装区审计：核对 ament 包发现、Python 模块来源、执行入口、接口类型支持及共享库依赖。', {
    'within': '解析真实路径后检查包含关系，防止导入落到选定安装区之外。',
    'audit': '使用已 source 的隔离安装环境读取真实模块和可执行文件，不通过追加源码路径掩盖安装问题。',
    'main': '只写新的审计 JSON 文件，并以全部检查是否通过决定退出码。'})
add(M+'validation/task_source_layout.py', '验证工具路径定位：从自身或指定位置向上寻找包含 src 的源码根目录。', {
    'find_source_root': '逐级检查父目录，避免任务目录迁移后固定 parents 层数失效。'})
add(M+'validation/plot_missions.py', '任务验收绘图：仅使用日志中真实样本绘制航迹、误差、推进器命令及独立圈数曲线。', {
    'plot_trial': '读取单场景指标和事件；缺少任务样本时不造图，命令曲线保留自身墙钟横轴。',
    'main': '接受单个场景或一组结果目录，逐场景输出 PNG/PDF 路径。'}, {
    '    progress = metrics.get("independent_orbit_progress", {})': '独立圈数曲线取自原始里程计积分记录，并显示有限的阶段采样容差。'})

small = {
    M+'package.xml': '任务应用 ROS 包清单：声明任务服务、算法包、TF、可视化和仿真启动依赖，构建方式为 ament_python。',
    M+'setup.cfg': '把任务命令行入口安装到 lib/usv_mission，使 ros2 run 能按包名找到脚本。',
    M+'setup.py': '安装任务 Python 包、launch/配置/RViz 资源，并注册 mission_node 和 mission_cli 两个命令。',
    I+'package.xml': '任务服务接口包清单：构建时生成 SetMission 类型，运行时依赖 rosidl 类型支持。',
    I+'CMakeLists.txt': '使用 ROS 接口生成器从 SetMission.srv 生成各语言服务类型，并导出运行时依赖。',
    I+'srv/SetMission.srv': '任务服务请求采用 WGS84 经纬度；响应说明是否接受、任务编号和拒绝原因。',
    M+'launch/mission.launch.py': '独立任务启动入口：仅创建任务节点，可用地理参考 YAML 或 world 文件提供显式原点。',
    M+'launch/mission_sim.launch.py': '完整任务仿真入口：用既有 Gazebo 资源和感知支持节点组合单一任务控制器。',
    M+'rviz/mission.rviz': '任务 RViz 布局：显示任务中心/半径/朝向标记，以及传感器、地图和全局/局部路径。',
    M+'test/conftest.py': '仅为任务包测试把本包源码目录加入 Python 搜索路径，便于测试当前工作区实现。',
    M+'usv_mission/__init__.py': '任务应用 Python 包入口，具体节点与数值运行时由各子模块提供。',
    M+'usv_mission/missions/__init__.py': '集中导出任务请求、规划意图和状态机，供任务适配器统一导入。',
    M+'validation/build_missions.bash': '从当前源码建立全新外部验证工作区，保留基线来源后构建、审计安装并运行源码测试。',
    M+'validation/refresh_missions.bash': '刷新验证采用新建工作区构建，复用 build_missions 的参数与既有结果保护。',
    M+'validation/run_mission_matrix.bash': '加载所选已构建工作区后运行任务矩阵调度器，其余参数原样传给 Python。',
    M+'validation/run_mission_validation.bash': '加载所选 ROS 安装环境后运行任务闭环验收脚本，可附带场景和重复数参数。',
    M+'validation/run_navigation_regression.bash': '选择任务或旧仿真导航回归入口；simulation 目标要求唯一匹配脚本，避免调用含糊路径。',
    M+'validation/run_source_tests.bash': '在已构建环境运行当前源码测试；未指定参数时显式覆盖 src 和 scripts。',
    M+'validation/_environment.bash': '供验收入口 source 的环境辅助脚本：向上定位源码根，再加载选定安装区和 ROS Humble。',
    M+'config/mission/mission_params.yaml': '任务控制调参：航向容差单位为度，船体安全扩展边距单位为米，地理原点由启动参数提供。',
    M+'config/mission/examples/orbit.yaml': '绕圈示例：在配套世界局部约 (23, -10) 米处以 0.5 米/秒沿 10 米半径顺时针巡逻三圈。',
    M+'config/mission/examples/standby.yaml': '待命示例：返回经纬度指定中心并低速停稳，随后越出 10 米观察半径才重新返中。',
    M+'config/mission/examples/station_keep.yaml': '定点保持示例：保持给定经纬度与真航向 90 度，即面向东方；其余值使用服务默认值。',
    P+'package.xml': '纯数值感知包清单：整数证据建图与扫描队列只依赖 NumPy，不依赖 ROS 节点运行。',
    P+'setup.cfg': '设置 perception 脚本安装目录，并将包内 test 目录作为默认 pytest 测试范围。',
    P+'setup.py': '安装纯 Python perception 包及 ament 索引资源，测试代码不打包进运行模块。',
    P+'perception/__init__.py': '纯感知算法包公共入口，导出栅格建图配置与实现。',
    P+'test/benchmark_mapping.py': '可复现建图内核基准：固定长射线输入，报告毫秒耗时及周期预算，不包含 ROS 序列化或 TF 查询。',
    U+'CMakeLists.txt': '混合感知包构建：安装 Python 适配器和脚本，同时编译负责 Nav2 生命周期的 C++ 代价地图节点。',
    U+'package.xml': '感知节点包清单：依赖 Nav2 代价地图、TF、激光/占用图消息及模块心跳接口。',
    U+'launch/perception.launch.py': '独立感知 launch 入口，具体参数读取与三个业务节点的组合由 launch_support 负责。',
    U+'scripts/perception_mapping_node': 'ros2 run 的激光建图可执行入口，转交 usv_perception.mapping_node.main。',
    U+'scripts/perception_monitor': 'ros2 run 的感知心跳监控可执行入口，转交 perception_monitor.main。',
    U+'test/run_ros_tests.bash': '加载 ROS Humble 并配置当前感知源码路径，在独立域 173 运行包内 ROS 测试。',
    U+'usv_perception/__init__.py': '感知 ROS 适配包入口，建图、启动配置和心跳监控在各自模块中实现。',
    U+'config/costmap/mapping.yaml': '累积激光证据地图：每边 1024 格、分辨率 0.5 米/格，总边长 512 米。',
    U+'config/robots/otter/global_costmap_params_otter.yaml': 'Otter 局部感知代价地图配置；独立感知启动入口读取 costmap 段并按船舶命名空间覆盖话题和坐标系。',
}
for path, header in small.items():
    add(path, header)

plans[I+'srv/SetMission.srv'][2].update({
    'float64 heading_deg 0.0': '真北为 0 度，顺时针增加；服务适配层会转换为 ROS yaw。',
    'float64 speed 0.5': '速度为米/秒，下面三个位置/观察/绕圈半径参数均为米。',
    'uint32 laps 0': '圈数和秒制时长中任一非零上限先到达即结束绕圈；零表示该项不限。',
    '---': '分隔符以上为请求，以下为响应；accepted=False 时阅读 reason 获取拒绝原因。'})
plans[U+'config/robots/otter/global_costmap_params_otter.yaml'][2].update({
    '  update_frequency: 2.0': '更新/发布频率为 Hz，TF 容忍时间为秒；Nav2 的 width/height 为米，resolution 为米/格。',
    '  rolling_window: true': '滚动窗口随船移动；track_unknown_space=false 表示本配置不保留未知区标签。',
    '  footprint: "[[-1.1, -0.6], [1.1, -0.6], [1.1, 0.6], [-1.1, 0.6]]"': '船体系多边形长 2.2 米、宽 1.2 米；padding 另加缓冲边距。',
    '  plugins: ["obstacle_layer", "inflation_layer"]': '以当前有效清单为准：只装载实时障碍层和膨胀层；下面的 static_layer 段未列入清单，当前不参与更新。',
    '      clearing: true': '射线穿越清空旧障碍，有效回波标记新障碍；标记/清空距离分别为 20 米和 22 米。',
    '  inflation_layer:': '围绕致命障碍生成随距离衰减的软代价，供路径搜索偏好更大净空。'})
plans[M+'setup.py'][2]['for directory in ("launch", "config", "rviz"):'] = '按资源原有子目录安装文件，并排除 Python 缓存，保证安装后启动入口能找到配置。'
plans[U+'CMakeLists.txt'][2]['add_executable(costmap_support_node src/costmap_support_node.cpp)'] = 'C++ 可执行程序只承接 Nav2 代价地图生命周期和线程管理，算法建图在 Python 包中。'
plans[M+'validation/build_missions.bash'][2]['if [[ -e "$mission_ws" || -L "$mission_ws" ]]; then'] = '拒绝已有文件、目录或符号链接，确保构建不会覆盖已验证的源码和证据。'
plans[M+'validation/_environment.bash'][2]['task_source_environment() {'] = '显式加载已构建工作区；单线程 BLAS/OMP 限制数值库隐式并发。'
add(U+'src/costmap_support_node.cpp', 'Nav2 代价地图生命周期宿主：启动更新线程，并在正常或中断退出时按依赖顺序释放资源。', anchors={
    'class CostmapSupportNode : public nav2_costmap_2d::Costmap2DROS': '继承 Costmap2DROS，补充 Humble 生命周期中断场景的析构清理。',
    '  ~CostmapSupportNode() override': '先结束地图更新，再释放执行器及图层；回调销毁时插件动态库必须仍然存活。',
    '    map_update_thread_shutdown_ = true;': '中断激活可能跳过正常 deactivate/cleanup，必须主动通知并 join 尚存线程。',
    '    executor_thread_.reset();': '先让内部执行器停止访问资源，再释放插件实例、TF 和回调组。',
    'int main(int argc, char ** argv)': '启动两线程执行器，依次 configure 与 activate；状态不符或异常返回失败退出码。'})

def test_file(path, header, descriptions):
    source = (BASE/'before'/path).read_text(encoding='utf-8-sig')
    names = [n.name for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name.startswith('test_')]
    texts = [line.strip() for line in descriptions.strip().splitlines() if line.strip()]
    assert len(names) == len(texts), (path, len(names), len(texts))
    add(path, header, dict(zip(names, ('验证：'+text+'。' for text in texts))))

test_file(M+'test/test_control_authority.py', '控制发布权回归：检查实际重映射话题的独占性、冲突锁存和任务/仿真导入边界。', '''
每个实际输出话题都已发现且恰有一个发布者才放行
第二发布者消失后冲突仍锁存，必须重启才能恢复
图查询异常阻止输出，但不能擦除已发现的冲突
任务运行时代码不反向导入仿真模块
''')
test_file(M+'test/test_missions.py', '任务状态机离线回归：用确定性地图和里程计样本覆盖待命、绕圈、保持、退让及倒退对准前序。', '''
非法任务类型、经纬度、速度、半径与计时参数均被拒绝
任务速度遵守船舶模型非零的最低速度限制
待命要求低速驻留，越出外圈才返中并保持内外半径滞回
顺逆时针跨越角度包络仍按有符号进度累计并在规定圈数返中
反向抖动相互抵消，离开精度带期间不补计圈数
绕圈计时含切入阶段，时钟暂停不增加时间，回退触发停止
静止任务仍检测近场入侵，船体本身碰撞时禁止当成可退让起点
恢复需要不同的新鲜地图和原目标净空，保留原任务模式和中心
陈旧观测或不同坐标系使活动任务停止
远距离定点任务先导航，进入近区后保持模式锁存
小幅位置噪声不让已锁定的入圈对准目标来回变化
保持控制器的预测危险可以主动请求安全退让
退让候选的转向包络包含位置容差圆
超过物理速度限制的瞬移不能增加圈数
从保持阶段被抢占后，恢复使用导航返回原定点位置
未观测到地图的时间缺口不算连续净空驻留
已知区域过小时报告无安全出口，不向未知区域退让
平移或偏航尚未实测停止时只能制动，不能建立旋转锚点
对准锚点采用实测停止坐标并保持固定，忽略规划器假设位置
位置、航向、速度约束持续全部满足才释放对准前序
对准位置阈值随用户要求的任务精度收紧
重复里程计时间戳不能积累对准驻留，缺测后重新计时
状态过期清除驻留计时但保持已冻结的旋转锚点
重复起步元数据不会重启正在进行或已完成的前序
退让前序不改变原圆、原任务和此前有效净圈数
目标失效、恢复、新安全事件或规划失败都会清理旧起步状态
非有限起步位姿不能成为旋转目标
倒退目标依据实测停止朝向冻结，再以实测倒退终点对准
倒退到达要求位置、航向和低速全程持续达标
倒退位置精度取请求精度十分之一且必须有不同里程计样本
输入或参考失效后丢弃旧倒退证据和临时目标
非法类型、非有限或超范围倒退距离被拒绝
合法倒退距离只影响临时后退目标，不改前方安全目标
真实保持控制器与动力学迭代能完成安全倒退再对准前序
''')
test_file(M+'test/test_mission_adapter.py', '任务适配层离线回归：用轻量节点替身检验条件起步参考的隔离及新地图后的重试。', '''
重复条件方案不可执行，候选阻塞直到新地图到达后才重新派发
只有重新规划标志而没有起步位姿的条件结果也不能被执行
''')
test_file(M+'test/test_mission_cli.py', '任务 CLI 输入回归：确认默认值、YAML 结构、经纬度示例及错误报告均在连接 ROS 前处理。', '''
最简定点保持请求使用服务接口规定的默认速度、精度和航向
缺字段、错误类型和非法范围在本地被拒绝
任务 YAML 重复键与必填键缺失均能被发现
三个安装示例均有效且对应配套世界约 (23, -10) 米目标
start/status/cancel 和命名空间参数可明确解析
非法 YAML 在导入 ROS 之前产生标准 CLI 错误退出码
''')
test_file(M+'test/test_mission_launch.py', '任务启动回归：校验显式地理原点、依赖声明以及唯一任务控制节点的 launch 组合。', '''
缺少地理参考时不会默认启用经纬度零原点
显式 world 文件提供其中真实声明的地理原点
显式零原点有效，且区别于没有提供参考
非法显式地理参考文件在构造节点前被拒绝
同时提供 world 与参考 YAML 会因来源含糊而拒绝
不存在的显式 world 文件无法悄悄降级为有效参考
包清单声明运行时算法、消息及仿真启动依赖
独立入口只启动任务节点且不包含仿真 launch
完整仿真仅有一个任务控制器，并启动公开定位、建图和代价地图支持节点
独立启动在返回节点前拒绝无效地理原点
''')
test_file(M+'test/test_mission_planning.py', '任务规划回归：用合成障碍验证切点、候选筛选、整船扫掠、统一时限和条件起步几何。', '''
圆周入圈候选覆盖各象限并含船外两条精确切线的切点
恢复候选沿指定方向覆盖余下圆周并排除当前点
前半圆全被阻挡时仍能在后半圆找到安全恢复点
前半圆有安全候选时，不被更近的上游候选替换
顺逆时针主候选集都包含 pi 边界的对侧点
从圆心、圆内和圆外均可选出通过验证的切向入圈航路
绕圈参考不复用有限旧航路、不做终点减速
未来圆周扫掠出现障碍时规划失败并抢占为退让
退让最多验证四个候选，同时保留参与规划的地图时间戳
待办绕圈请求不继承普通工作结果中的有限航路
规划路径终点航向变化不会覆盖用户所需的圆周切向朝向
候选终点必须给任意航向保留足迹空间
转向包络包含扩展船体旋转后的角点
转向包络还覆盖到达位置容差引入的误差
到达圆不安全的退让候选在工作进程内再次被拒绝
前向 LOS 不安全时可提供已验证制动后对准的条件方案
动力学制动尾迹碰撞时不能提供旋转方案
已经对准的退让直接使用正常 LOS，不重复前序
实测对准后对同一目标重新规划，不继续携带旧条件起步标记
停止位置转向空间不足时先验证短距离倒退再转向
船后障碍阻挡时不能提供穿越障碍的倒退方案
原位可以安全对准时优先使用原位，不做多余倒退
''')
test_file(M+'test/test_mission_ros.py', '任务 ROS 适配回归：覆盖地理服务、替换事务、控制器交接、安全制动与发布权冲突。', '''
类型化 SetMission 服务接受地理目标并转换到正确局部坐标
非法替换保留旧任务及版本，成功替换才递增任务编号
定点保持持续活动且生成有限的推进器命令
保持任务也遵守状态和地图超时，失效返回零推力
导航交给保持时从实际已发命令限制推力变化率
等待退让参考期间，对运动船舶使用已验证的制动路径
没有可验证的制动命令时返回零推力
可选择更强的安全制动，但不得缩小安全边距
条件起步轨迹不能在实测停止前交给控制器
退让命令不安全时改用单独通过预测校验的制动命令
取消后旧任务或旧规划响应不会重新启动导航
当前船体占用阻止保持控制，不能豁免为退让
没有普通目标的待命任务仍主动监测近场入侵
选中切向航路在参考版本切换后仍能保留，旧版本结果被统计为过期
在线增益更新传递到任务控制器且保留任务精度及积分状态
竞争发布者阻止任务、控制和取消输出并锁存冲突
发布权门控检查真实重映射话题，未知 ROS 图状态禁止发布
交接零命令时新发现冲突也不能接受并安装新任务
''')
test_file(M+'test/test_task_configuration.py', '任务配置回归：校验旧参数兼容、事务式热更新、LOS 精度边界及地理原点来源。', '''
原工程 YAML 可加载，旧别名兼容且 LQR 矩阵转换为浮点数组
规范参数名与别名给出冲突值时明确拒绝
无效更新不改写当前参数，合法更新返回新的候选配置
不合法的启动参数组合被统一校验拒绝
LQR 可使用当前时刻参考，但 lookahead_time 不能在线修改
控制器目标半径过小、无法支持 LOS 有效精度时拒绝启动
最小允许的目标半径原值得到保留
工程 YAML 中低于 LOS 精度的目标半径同样被拒绝
地理坐标允许有符号数值，但拒绝范围外经纬度
任务调参 YAML 可省略，并且不能覆盖从世界读取的地理原点
''')
test_file(M+'test/test_task_navigation_ros.py', '任务节点 ROS 集成回归：使用受控输入测试异步规划、控制失效、参数更新、取消和时钟复位。', '''
短暂控制器超时会增加失败计数并发布零命令，同时保留目标
参考和中心点虽安全，实测船体角点碰撞仍阻止控制
完整节点链处理进程规划、参数事务、陈旧输入、暂停、取消和时间回退
''')
test_file(M+'test/test_task_runtime.py', '普通导航离线运行时回归：覆盖航路复用、软膨胀区退出、精确目标停止圆及有界规划队列。', '''
路径与 LOS 离线链生成有效参考，重复请求复用已验证航路
合法软膨胀区退出航路不会触发第二次无用全局搜索
复用航路仍检查最新地图的平台、上坡、致命和未知代价
航路复用检查从最新船位到旧航路的连接段
无路径结果返回明确失败状态和空参考轨迹
默认 PID 保留直线栅格终点，不为精确目标制造多余折角
LOS 停止圆不会让 LQR 在实际请求目标容差之外停止
目标容差较小时追加精确目标，同时保留原栅格终点
精确目标连接段也必须通过软代价、致命及未知单元检查
新鲜度边界对未来、陈旧和有效时戳作出预期判断
输入门控不复用过期状态、地图或轨迹
规划工作器只保留最新待办，作废不会误认为在途任务已消失
首次搜索结果可补充待办航路，但保留待办最新状态、地图和参数
传给待办的已搜索航路也要在最新障碍地图上重新验证
''')
test_file(M+'validation/test_migration_validation.py', '验收工具迁移回归：检查任务节点图证据、目录定位、源码指纹及本地验证模块来源。', '''
合法图中唯一任务节点独占三个命令话题
缺失、多余或错误控制发布者都成为显式失败
实时图守卫保存每次观测，后来恢复不能擦除历史失败
不同深度的验证目录都能向上找到源码根
任务验证源码只计入指纹一次，安装文件变化独立改变安装指纹
根级同名模块缓存不会重定向本任务包的验收工具
旧成功记录缺少实际 ROS 图所有权证据不能提升为成功
导航入口指向核心任务节点并保留公开参数命名契约
''')
test_file(M+'validation/test_mission_acceptance.py', '任务验收判据回归：用构造样本检查固定窗口、独立圈数、绕障恢复、扰动证据及只画实测数据。', '''
保持窗口发生偏离后不能重开较好窗口覆盖原结果
位置和航向联合达标率按同一时刻计算，不以两个边际比例代替
缺失里程计使固定窗口不完整，即使剩余样本均达标也失败
收敛必须连续五秒位置、航向和低速全部满足
收敛检测不会把缺测时段计入驻留
仅有 STANDBY 标签不足以证明实测返中驻留
安全等待要求选定目标、低速和完整船体净距同时满足
顺逆时针真实有符号里程计可独立证明完整一圈
相同阶段与时戳的重复状态不改变圈数评分或边界容差
重复状态快照不能让静止船舶凭空完成一圈
状态去重不能折叠掉倒退时间或同刻阶段切换的错误
提前返航、静止或反方向绕圈都不能通过规定圈数
里程计缺口使测量失效，不能扩大阶段边界容差补偿
位置瞬移与状态消息断流均导致独立圈数验证失败
避障阶段和径向超出精度带的运动不计入净圈数
仅前进一米不足以删除障碍，必须确认实际绕过并恢复圆周
靠近圆心反向极角变化不否定已经证实的下游整船绕过
刚生成障碍或上游小幅回退不能判为成功恢复
下游恢复仍要满足原圆径向、航向及运动要求
船中心过平面而船体仍跨越平面时，整船通过不成立
恢复原圆仍需要一秒不同的新鲜观测驻留
已证实的下游通过不会因继续绕到另一半圆而撤销
早先退让中存在危险或缺测记录不能在恢复时丢弃
横向扰动必须由新鲜横向速度观测证明，不能仅靠位移
延迟服务的观测窗口包含请求期间和响应后的样本
服务等待期间观测到有效横向脉冲可接受并记录延迟
延迟服务没有新脉冲时不能复用注入前的高速度
服务延迟超上限独立标记阻塞，即便存在运动观测
航向误差正确处理角度跨界且场景目标方向保持一致
预检拒绝致命格、未知格和超出观测地图的试验包络
非法推力先保留原始日志再停止采集
原始日志保存每个绕圈样本对应的阶段和径向误差
障碍必须被观测为致命占用，未知值不能代替探测成功
dry-run 只输出未运行计划，不能报告试验通过
阻塞且无测量的场景不会生成虚构图像
仅用日志样本导出 PNG/PDF，推进器曲线保留独立墙钟时间轴
''')
test_file(M+'validation/test_mission_matrix.py', '矩阵调度器回归：使用可控子进程测试并发上限、停止派发、结果身份和证据完整性。', '''
默认 dry-run 规划 21 个独立 ROS 域且不创建子试验目录
ROS 域编号溢出在创建输出之前被拒绝
已有子任务失败后停止派发，正在运行的独立任务仍可完成
安装指纹发生变化或无法验证时子任务不能被标成通过
成功矩阵遵守 jobs 上限并保留外层真实重复编号
缺失结果标记阻塞，中断只向本调度器持有的子进程发信号
格式错误的子任务 manifest 标记阻塞，不使调度器崩溃
''')
test_file(P+'test/test_mapping.py', '整数证据建图内核回归：使用小地图精确检查光束顺序、量程、边界裁剪和坐标变换。', '''
默认地图尺寸、int8 类型和世界原点正确，复位后全为未知
首个命中为 54，重复命中饱和到 100，穿越可逐步清空
共享栅格按光束输入顺序更新，因此交换次序产生不同证据
NaN、负无穷、负距离和小于最小量程的输入不清空地图
正无穷或超量程返回只清空沿途，不标记虚假障碍
无回波清空距离最多 30 米，图内未裁剪端点不被额外清空
二维边界裁剪不会跨行索引或把边界当命中
图外起点射线可进入地图，完全不相交射线保持地图不变
含小数原点偏移时坐标使用真实原点和 floor 得到行列
传感器 yaw 旋转与地图边界行为一致
极大有限量程先裁剪再转整数索引，避免越界溢出
非法地图宽度、分辨率和偏移被构造器拒绝
非法扫描元数据抛错，空扫描不改变网格
对角 Bresenham 只更新预期经过格与命中格
''')
test_file(P+'test/test_scan_queue.py', '扫描队列回归：使用确定性时钟覆盖去重、容量、精确截止时间及诊断观测守恒。', '''
FIFO 中每帧只积分一次且拒绝旧时戳和重复时戳
队列仅保留最新五帧，溢出数量正确累计
缺少 TF 时保持队首顺序，并按单调墙钟到期丢弃
零时戳和空坐标系不能触发隐式最新 TF 查询
回退复位清空等待项及去重历史，较小时戳可重新接纳
不同坐标系的同一时戳不算重复扫描
时钟暂停有效，任意向后跳变被识别
首帧处理耗时后下一帧重新检查截止时间
失败 TF 查询跨过期限后立即过期，不等待下一次轮询
显式超时、非法消费和成功积分计数互相独立且总量守恒
最终清理与诊断采用同一检查点，队列消费及观测计数一致
恰好到达截止时刻的帧不会调用处理函数
不断更新的单帧流水线与同一帧卡住能通过身份数量区分
处理后观测不隐藏过期计数，也不会改变原超时规则
五秒窗口移除旧诊断，复位不伪造零年龄样本
诊断时钟也拒绝非有限值与向后跳变
''')
test_file(U+'test/test_launch.py', '独立感知启动回归：解析 launch 节点和参数，检查解释器、命名空间、图层清单及 YAML 错误。', '''
独立入口只启动建图、代价地图与监控三个节点并传递默认参数
PATH 上存在不兼容 Python 时仍使用启动进程的正确解释器
自定义 YAML、出生偏移和命名空间正确到达建图和代价地图
空命名空间在生成节点前被拒绝
代价地图必须明确给出非空插件清单
启动配置不能实际启用静态地图层
重复 YAML 键和 ROS 参数导出快照被拒绝作为工程配置
''')
test_file(U+'test/test_mapping_deadline_ros.py', '建图适配器期限回归：使用真实消息类型和精确模拟墙钟，检查 TF 返回时刻对地图修改的准入。', '''
TF 虽查询成功但达到或超过期限时不得修改地图
非法旋转扫描被消费但不计入成功建图数量
及时 TF 仍保持原整数证据和准确排队毫秒值
节点最终过期清理与诊断观察使用完全相同的检查点
''')
test_file(U+'test/test_mapping_ros.py', '感知 ROS 集成回归：受控时钟、扫描及 TF 驱动真实建图节点，不需要 Gazebo。', '''
精确时刻 TF、去重、墙钟超时、时钟回退、静态外参存活及地图字节发布共同符合预期
''')
test_file(U+'test/test_perception_monitor_ros.py', '感知身份 ROS 回归：创建、移除并重建真实发布端点，检查模块会话更新规则。', '''
发布者短暂消失只取消 ready，新发布端点出现才切换 session
''')

plans[U+'test/test_mapping_ros.py'][2].update({
    '        set_clock(10.0)': '先建立扫描同刻 TF 并完成一次积分，再重复同一帧证明不会二次累积。',
    '        set_clock(10.1)': '推进扫描时刻但不补 TF，验证不能使用旧时刻位姿并最终按墙钟过期。',
    '        set_clock(1.0)': '向后跳时钟必须清空动态观测及去重历史，已有静态传感器外参仍应可用。'})

def perform():
    inventory = json.loads((BASE/'inventory.json').read_text(encoding='utf-8'))
    targets = {item['path'] for item in inventory if item['path'].startswith((M, I, P, U))}
    assert set(plans) == targets, (targets-set(plans), set(plans)-targets)
    reports = []
    for path in sorted(targets):
        original = (BASE/'before'/path).read_bytes()
        current = (ROOT/path).read_bytes()
        bom = b'\xef\xbb\xbf' if original.startswith(b'\xef\xbb\xbf') else b''
        source = original[len(bom):].decode('utf-8')
        lines = source.splitlines(keepends=True)
        header, definitions, anchors = plans[path]
        newline = '\r\n' if '\r\n' in source else '\n'
        prefix = '// ' if path.endswith('.cpp') else '# '
        is_xml = path.endswith('.xml')
        insertions = {}
        def insert(index, text, indent=''):
            line = indent + ('<!-- '+text+' -->' if is_xml else prefix+text) + newline
            insertions.setdefault(index, []).append(line)
        first = 0
        if lines and (lines[0].startswith('#!') or lines[0].startswith('<?xml')):
            first = 1
        if len(lines) > first and ('coding:' in lines[first] or 'coding=' in lines[first]):
            first += 1
        insert(first, header)
        is_python = path.endswith('.py') or path in (U+'scripts/perception_mapping_node', U+'scripts/perception_monitor')
        if is_python:
            tree = ast.parse(source)
            seen = set()
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.name in definitions:
                    start = min([node.lineno]+[d.lineno for d in node.decorator_list])-1
                    insert(start, definitions[node.name], lines[start][:len(lines[start])-len(lines[start].lstrip())])
                    seen.add(node.name)
            assert seen == set(definitions), (path, set(definitions)-seen)
        for anchor, text in anchors.items():
            matches = [i for i, line in enumerate(lines) if line.rstrip('\r\n') == anchor]
            assert matches, (path, 'missing anchor', anchor)
            for index in matches:
                indent = anchor[:len(anchor)-len(anchor.lstrip())]
                insert(index, text, indent)
        result = bom + ''.join(''.join(insertions.get(i, []))+line for i, line in enumerate(lines)).encode('utf-8')
        if not lines:
            result = bom + ''.join(insertions.get(0, [])).encode('utf-8')
        assert current in (original, result), ('concurrent change', path)
        before_lines, after_lines = original.splitlines(keepends=True), result.splitlines(keepends=True)
        matcher = difflib.SequenceMatcher(a=before_lines, b=after_lines, autojunk=False)
        added = 0
        for op, a0, a1, b0, b1 in matcher.get_opcodes():
            assert op in ('equal', 'insert'), (path, 'modified original lines', op)
            if op == 'insert':
                assert all(line.lstrip().startswith((b'#', b'//', b'<!--')) for line in after_lines[b0:b1])
                added += b1-b0
        if is_python:
            assert ast.dump(ast.parse(original)) == ast.dump(ast.parse(result)), ('AST changed', path)
        if is_xml:
            import xml.etree.ElementTree as ET
            assert ET.tostring(ET.fromstring(original)) == ET.tostring(ET.fromstring(result)), ('XML changed', path)
        if path.endswith(('.yaml', '.rviz')):
            import yaml
            assert yaml.safe_load(original) == yaml.safe_load(result), ('YAML changed', path)
        (ROOT/path).write_bytes(result)
        reports.append({'path': path, 'added_comment_lines': added})
    (BASE/'mission_work/report.json').write_text(json.dumps(reports, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'files': len(reports), 'comment_lines': sum(row['added_comment_lines'] for row in reports)}, ensure_ascii=False))

if __name__ == '__main__':
    perform()
