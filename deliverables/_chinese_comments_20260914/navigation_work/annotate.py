from pathlib import Path
import ast
import collections
import difflib
import io
import json
import re
import tokenize
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]
WORK = ROOT / 'deliverables/_chinese_comments_20260914'
items = [v for v in json.loads((WORK / 'inventory.json').read_text(encoding='utf-8-sig'))
         if re.match(r'src/core/(control_module|decision_module|shared)/', v['path'])]
notes = collections.defaultdict(lambda: collections.defaultdict(list))
headers = {}

def resolve(suffix):
    matches = [v['path'] for v in items if v['path'].endswith(suffix)]
    assert len(matches) == 1, (suffix, matches)
    return matches[0]

def source(path):
    return (WORK / 'before' / path).read_bytes().decode('utf-8-sig')

def describe(suffix, header, functions=None, anchors=None):
    path = resolve(suffix)
    headers[path] = header
    text = source(path)
    lines = text.splitlines()
    tree = ast.parse(text) if path.endswith('.py') else None
    for name, note in (functions or {}).items():
        matches = [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and n.name == name]
        assert matches, (path, name)
        for node in matches:
            line = min([node.lineno] + [d.lineno for d in node.decorator_list])
            indent = re.match(r'\s*', lines[line - 1]).group()
            notes[path][line - 1].append(indent + '# ' + note)
    for anchor, note in (anchors or {}).items():
        matches = [i for i, line in enumerate(lines) if line.strip() == anchor]
        assert len(matches) == 1, (path, anchor, matches)
        i = matches[0]
        indent = re.match(r'\s*', lines[i]).group()
        notes[path][i].append(indent + '# ' + note)

describe('controller/controller/controller.py', '控制器公共契约：验证状态与轨迹，选取随轨迹年龄推进的预瞄点，统一返回归一化双桨命令。', {
    'validate_goal_distance_tolerance': '目标距离容差单位为米，必须大于 LOS 去重尺度，保证控制器要求的位置精度可由参考轨迹表达。',
    'select_config': '兼容完整 controller 配置、指定算法子配置和直接参数字典三种输入层级。',
    'stop': '将失败、等待或完成状态附在零推力命令中，调用方可同时处理执行输出与诊断原因。',
    'reset': '新任务或确认到达时由具体算法清除积分、误差或求解诊断历史。',
    '_reference': '先判断真实船舶是否满足终点条件，再按状态时间与预瞄时间从轨迹插值，不能以预测到达代替真实到达。',
    '_command': '左右桨分别限幅到 [-1, 1]；任何非有限输出都转为 INVALID_INPUT 的零命令。',
}, {
    'age = max(0.0, state.stamp - trajectory.stamp)': '轨迹 stamp 是生成时使用的状态时间，重复传输不会使旧轨迹重新变新；预瞄时间单位为秒。',
    'if math.hypot(state.u, state.v) > self.config["goal_speed_tolerance"]:': '进入目标圆盘但仍有较大平面速度时返回 SETTLING；漂出圆盘后仍可恢复跟踪。',
    'terminal = trajectory[-1]': '只有预测终点位于最终目标盘内时才恢复接近速度，途中明确设置的停车点保持原意。',
})
describe('controller/controller/controller_factory.py', '按名称构造 Otter 的 PID 或 LQR 控制器；不支持的算法和船型在初始化时明确报错。')
describe('controller/controller/pid_controller.py', 'Otter 双环 PID：以平面速度模和 ROS 航向误差计算纵向及转向需求，再分配为左右桨归一化命令。', {
    'PIDController': '保留原工程增益标定；积分变量已经包含积分增益，分配时不再次除以最大物理推力。',
    'update_config': '先完整验证候选参数再替换配置；在线调参保留积分历史，失败更新不污染当前参数。',
    'reset': '任务切换时清除速度、航向的积分项与上一拍误差。',
    'compute': 'dt 为秒，u/v 为船体系带符号速度；速度环使用 hypot(u,v)，航向环使用最短弧误差。',
}, {
    'yaw = wrap_angle(yaw)': '保留历史航向控制量的角度规整规则，以兼容当前调好的增益。',
    'command = self._command(0.5 * (surge - yaw / self.model.D),': '差动分配满足左右和产生纵向需求、右减左乘力臂 D 产生转向需求。',
    'if command.status == AlgorithmStatus.SUCCESS:': '仅成功输出才提交积分和误差历史，避免数值异常影响后续控制。',
})
describe('controller/controller/lqr_controller.py', '保留六状态 Otter 离散 LQR 模型，通过有界 Riccati 迭代求双桨反馈增益。', {
    'LQRController': '状态顺序为世界坐标 x/y、航向 psi、船体系 u/v/r；Q 对应六状态，R 对应左右桨输入。',
    'discrete_model': '围绕参考航向和纵向速度线性化，用前向欧拉离散；discrete_dt 独立于本次控制回调 dt。',
    'solve_gain': '同时限制迭代次数与可选墙钟耗时，未收敛时抛出超时，禁止使用半成品增益。',
    'compute': '构造当前减参考的六维误差，航向规整后使用带负号的 K 反馈，异常统一产生零命令。',
}, {
    'B *= model.max_force * dt': 'B 将归一化双桨输入换算为物理力作用，再乘离散时间得到状态增量。',
    'feedback = np.linalg.solve(hessian, B.T @ P @ A)': '解线性方程而非显式求逆，计算离散代数 Riccati 更新中的反馈项。',
    'residual = float(np.linalg.norm(P_next - P))': '相邻 P 矩阵的范数差用于判断收敛，并暴露给数值回归测试。',
})
describe('controller/controller/station_keeping_controller.py', '欠驱动 Otter 定点定艏控制：通过 RECOVER、ALIGN、HOLD 三阶段完成位置恢复、艏向对准与持续保持。', {
    'StationKeepingController': '只有左右纵向桨；横向偏移通过转向和前后运动消除，不假定存在侧推执行器。',
    'reset': '从位置恢复阶段启动，并清空对准计时和上一条实际推力历史。',
    'observe_command': '记录实际发出的归一化推力，接管或安全替代后必须同步，否则变化率限制会从错误起点计算。',
    'compute': '目标是世界坐标 (x米, y米, yaw弧度)；保持成功仍为 SUCCESS，退出由运行层管理。',
}, {
    'ex, ey = c * dx + s * dy, -s * dx + c * dy': '将世界系位置误差转到船体系，ex 为纵向误差，ey 为横向误差。',
    'if self.phase != "RECOVER" and (distance > .7 * tolerance or abs(ey) > .5 * tolerance):': '进入对准使用更小距离门限，重新恢复使用较大门限，形成滞回以减少边界来回切换。',
    'course_error = forward_error if abs(forward_error) <= abs(reverse_error) else reverse_error': '比较正驶与倒驶所需转角，选择转动较少的接近方向。',
    'delta = cfg["command_slew_rate"] * dt': '变化率以归一化命令每秒表示；从接管时实际输出逐拍接近目标，允许过渡值暂高于保持上限。',
})
describe('usv_control/usv_control/control.py', '普通导航控制 ROS 节点：接收决策参考和感知快照，执行 PID/LQR，并通过唯一推进输出适配器发布三路桨命令。', {
    'ControlNode': '会话号区分进程重启，任务号区分目标，序号拒绝旧参考；控制循环独立于规划进程运行。',
    '_close_task': '登记已关闭任务的最大编号、重置算法并立即发零，阻止同一任务的迟到活动参考重新启动。',
    '_on_peer_restart': '决策或感知进程重启会关闭当前任务，要求上层重新下达目标。',
    '_reference_parameters': '参考必须携带完整有效参数快照；只允许 HOT_PARAMETERS 中的原有热参数与本地不同。',
    '_on_reference': '先核验三方会话与任务时序，再校验参数、目标与轨迹；无效参考立即停止输出。',
    '_apply_parameters': '在控制拍边界原子应用待处理参数，PID 在线增益更新保留积分状态。',
    '_publish_command': '按左桨、右桨、侧推顺序发布 Float32，同时保存实际发送值供报告使用。',
    '_control': '每拍默认零命令，只有数据新鲜、模块在线、船体无碰撞且 dt>0 时才运行跟踪算法。',
    '_publish_report': '回报实际推力、任务/参考身份与撤销标志，决策端据此确认取消或真实到达。',
    'destroy_node': 'ROS 上下文仍可用时先发布零推力，再销毁节点。',
}, {
    'if not message.active:': '非活动快照用于取消或结束任务，也可携带已接受的在线参数变更。',
    'if now < self.last_clock - 1e-6:': '仿真时间倒退时关闭任务并清除旧输入，避免跨时间轴继续使用原轨迹。',
    'if ready in (\'NAVIGATING\', \'WAITING_REFERENCE\') and not self._current_footprint_clear():': '即使参考尚未到来，也检查最新实测船体；碰撞撤销参考但不将任务标为完成。',
    'elif dt <= 0:': '稳态时钟仍会触发回调；仿真时钟暂停导致 dt 非正时保持零输出。',
})

describe('path_planner/path_planner/path_planner.py', '路径几何工具：地图坐标下的 N×3 位姿路径转换、二维栅格索引、按弧长重采样与最近路点剪枝。', {
    'as_path': '接收 N×2 或 N×3 数组，缺省航向补零并复制数据，后续处理不修改调用方输入。',
    'grid_to_index': '行优先展开为 x + width*y，先分别检查二维边界以防跨行索引别名。',
    'index_to_grid': '用取模恢复列、整除恢复行，并拒绝越界索引及非整数输入。',
    '_heading': '内部路点使用三点圆切线估计航向；近共线或重合时回退，并令切线指向后继点。',
    'resample': '按累计弧长以米为间距插值，保留首尾完整位姿；相邻重复坐标先合并。',
    'prune': '保留最近路点之后的路径，距离相等时选最后一点；可按路径距离向后多保留一段。',
}, {
    'if not math.isfinite(ratio) or ratio > 2_000_000:': '拒绝异常细采样导致的巨量分配，以免配置错误耗尽内存。',
})
describe('path_planner/path_planner/path_planner_core.py', '全局规划门面：执行栅格搜索，逐段验证路径，再尝试带碰撞复检的可选重采样。', {
    'path_is_clear_with_inflation_escape': '逐个穿越单元验证线段；仅允许起点膨胀区代价严格下降，离开后不再进入高代价区。',
    'hard': '致命障碍、按策略禁止的未知单元、启用描边的地图边界均不可逃逸。',
    'blocked': '软障碍由阈值定义；未知单元单独按策略处理，不直接以原始数值大小判断。',
    'make_plan': '搜索成功后仍检查完整线段；重采样若穿障则保留已验证的原栅格路径。',
}, {
    'while current != target:': '按下一个 x/y 栅格边界的穿越参数前进，捕捉固定间距采样可能漏掉的短暂单元交会。',
    'if cross_x and cross_y and not allow_corner_cutting:': '精确穿过角点时遵守与搜索器相同的对角切角策略。',
    'if blocked(following) and (not blocked(current)': '不同高代价单元之间必须严格下降；同一单元内重复点不视为代价平台。',
})
describe('graph_planner/astar_planner.py', '有界八邻域栅格搜索，统一实现 A*、Dijkstra 与贪心最佳优先，输出地图单元中心位姿序列。', {
    'AStarPathPlanner': '代价采用 Nav2 原始 uint8：254 为致命，255 为未知；软障碍阈值为 254×obstacle_factor。',
    'plan': '输入起终点均为 (x米, y米, yaw弧度)，搜索过程中同时限制耗时和扩展次数。',
}, {
    'hard_blocked = costs == 254': '硬障碍不允许逃逸；高于阈值的膨胀成本可按严格下降规则从起点退出。',
    'queue = [(0.0, 0.0, next(counter), source, 0.0)]': '堆元素保存优先级、启发值、稳定计数器、单元和累计距离；计数器打破同分并列。',
    'if current in closed or current_g > best.get(current, math.inf):': '跳过已经闭合或已被更短路径替代的旧堆条目。',
    'if current == target:': '沿父指针回溯并反转为起点到终点，坐标转换为单元中心且保留两端航向。',
    'heuristic = math.hypot(nx - target[0], ny - target[1]) if self.name != "dijkstra" else 0.0': 'Dijkstra 令启发值为零，A* 用 g+h，GBFS 只用 h；移动距离按栅格尺度计。',
})
describe('utils/path_planner_factory.py', '从配置显式选择三种已实现栅格搜索模式；未知 planner_name 在创建时拒绝。')
describe('path_processor/path_processor/path_processor.py', '可选路径处理器的数组校验与基类：保存障碍诊断数据，具体简化/平滑仍需调用方重新碰撞检查。', {
    'as_path': '将二维位置补航向为 N×3 副本，统一拒绝非法维度与非有限值。',
    'set_obstacles': '保存一份独立障碍数组；RDP 与 SG 当前不消费该数组来约束几何变换。',
    'process': '基类仅定义处理接口，实际实现由具体路径处理器提供。',
})
describe('path_processor/path_processor/ramer_douglas_peucker.py', '用显式栈实现 RDP 路径简化，并可按最大间隔重新加密；位置和距离均以米表示。', {
    'ramer_douglas_peucker': '每段保留偏离端点连线段最远且超过 epsilon 的点，重复分段直到满足误差阈值。',
    'RDPPathProcessor': '保存简化阈值和可选加密间隔，以统一 process 接口调用纯函数。',
}, {
    't = np.clip(offsets @ segment / square, 0.0, 1.0)': '投影参数限制在 [0,1]，计算点到有限线段的距离，避免删掉折返到线段外的路点。',
    'segments = int(math.floor(ratio)) + 1': '保留原工程 floor(长度/间隔)+1 的分段规则，整数倍长度也会多分一段。',
    'angular_delta = (last[2] - first[2] + math.pi) % (2 * math.pi) - math.pi': '加密航向沿最短角弧插值，避免跨 ±π 时绕远转向。',
})
describe('path_processor/path_processor/savitzky_golay.py', 'NumPy 最小二乘核实现 Savitzky–Golay 位置平滑，保留端点位姿并重算内部切线航向。', {
    'savitzky_golay': '窗口必须为不少于三的奇数，次数小于窗口；短于窗口的路径直接返回独立副本。',
    'SavitzkyGolayPathProcessor': '构造时先验证滤波配置，process 将固定窗口参数传给纯函数。',
}, {
    'kernel = np.linalg.pinv(np.vander(positions, polyorder + 1, increasing=True))[0]': 'Vandermonde 矩阵拟合局部多项式，伪逆首行得到中心位置估计的卷积权重。',
    'padded = np.pad(points[:, :2], ((half, half), (0, 0)), mode="edge")': '复制边缘坐标补齐窗口；平滑可改变可通行性，调用方必须复检结果。',
})
describe('trajectory_planner/trajectory_planner/circle.py', '不可变圆航线参数：局部地图圆心、半径米、航速米每秒与顺逆时针方向。', {
    'direction': 'ROS 航向逆时针为正，因此顺时针方向返回 -1。',
    'pose_at': '用圆周角求位置及沿航行方向的切线航向，输出 yaw 规整到 [-π,π)。',
})
describe('trajectory_planner/trajectory_planner/los_trajectory_planner.py', 'LOS 视线导引生成带时间戳的有限时域参考轨迹，普通路线按剩余长度减速，圆航线保持恒速。', {
    'LosTrajectoryPlanner': '模型提供巡航速度；配置规定输出间隔、预瞄距离、路点切换距离与偏航角速度上限。',
    'plan_circle': '圆形向量场叠加曲率前馈，采用中点积分减小半径偏差；圆航线没有最终停车目标。',
    'turn_rate': '径向误差修正切线方向，前馈包含圆周角变化与径向修正变化，再加航向误差反馈并限幅。',
    'plan': '输入地图 N×3 路点与当前船舶状态，积分位置和速度使用同一速度剖面以保持运动学一致。',
}, {
    'if circle.speed / circle.radius > cfg[\'yaw_rate_max\'] + 1e-12:': '匀速圆周运动需要角速度 v/R，超过配置上限时该参考不可行。',
    'keep = np.r_[True, np.linalg.norm(np.diff(waypoints[:, :2], axis=0), axis=1) > 1e-9]': '先删去相邻距离不大于 1e-9 米的重复路点，避免零长度航段。',
    'speed_denominator = max(initial_distance, cfg["slowdown_distance"])': '减速分母至少为 slowdown_distance，避免临近目标重规划时又恢复巡航速度。',
    'inner_dt = min(0.1, output_time - elapsed)': '内部积分步长不大于 0.1 秒且精确止于下一输出时间，输出间隔更小时也不越界积分。',
    'desired_yaw = segment_yaw - math.atan2(cross_track, cfg["lookahead_distance"])': '横向误差按航段左侧为正；从航段方向减去修正角，使参考朝路径收敛。',
    'allow_switch = False': '每个输出周期最多切换一次路点，保留原导引在密集转弯点处的推进顺序。',
    'curvature = yaw_rate / speed if speed > 1e-9 and not reached else 0.0': '曲率单位为 1/米，按偏航角速度除以线速度求得；停止时显式置零防止除零。',
})
describe('trajectory_planner/trajectory_planner/trajectory.py', '只读时间参数轨迹：八列依次是相对秒、弧长米、x米、y米、yaw弧度、曲率1/米、速度米/秒、加速度米/秒²。', {
    'TrajectoryCell': '单个插值参考点的具名元组，字段顺序与轨迹数组一致。',
    'Trajectory': '复制并冻结 N×8 数组，校验时间严格递增和弧长非递减，stamp 保存生成参考的绝对时间。',
    'reference_at_time': '按相对时间秒查询预瞄点，查询范围外取首尾参考。',
    'reference_at_distance': '按累计弧长米查询，可处理停车期间重复的弧长值。',
    '_reference': '搜索上下包围点后线性插值；航向独立按最短角弧插值，避免 ±π 跳变。',
    'TrajectoryResult': '将状态、轨迹、诊断信息和算法耗时一起返回调用者。',
})
describe('trajectory_planner/trajectory_planner/trajectory_planner_factory.py', '创建当前唯一支持的 LOS 轨迹规划器，拒绝未知名称以避免静默回退。')

describe('usv_decision/usv_decision/decision.py', '普通导航决策节点拥有目标任务和规划生命周期，发布路径/轨迹参考并依据控制报告确认停止与到达。', {
    'DecisionNode': '保留外部节点名 usv_simulation；后台规划进程与控制执行分离，ROS 回调只投递和验收快照。',
    '_apply_parameters': 'PID 热参数随参考同步；LOS 热参数另增规划版本并使旧轨迹失效，但不更换任务号。',
    '_trajectory_clear': '验收参考时同时检查最新实测船体和参考扫掠船体；逐拍执行校验由控制节点负责。',
    '_map_updated': '新地图阻断已有参考时立即清空路线与轨迹，并通知控制端等待新参考。',
    '_reset_task': '增加版本与任务号，失效化排队规划，清空可视化和运行状态，再发布当前任务快照。',
    '_on_goal': '替换目标先清理旧任务，新目标验证后一次性发布；同一任务号不能先发停止再发活动参考。',
    '_cancel': '取消服务等待对应任务和参考序号的实际零输出确认，墙钟两秒超时则返回失败。',
    '_export': '导出当前节点有效参数的 ROS YAML 快照，区别于项目启动配置文件格式。',
    '_planning': '只在目标、状态和地图都存在且时间新鲜时投递副本，执行中的旧工作不阻塞回调。',
    '_consume_plan': '拒绝过期版本结果，并用最新地图复检工作进程产生的扫掠轨迹后才发布。',
    '_publish_reference': '改变内容时递增序号，定时重传时保持序号和轨迹源时间，不能靠重发延长轨迹有效期。',
    '_on_control_report': '先验证会话并处理取消确认，再允许当前任务报告更新决策状态。',
    '_resolve_cancellations': '每个取消请求分别核对任务号、最低参考序号和三路零推力，不能用后续取消替代先前确认。',
    '_tick': '稳态定时回调处理时间倒退、参数、里程计、规划结果、参考重发与取消超时。',
    '_publish_status': '汇总自身和新鲜控制报告的统计信息，发布 JSON 导航状态便于运行观察。',
    'destroy_node': '退出时发停用参考、结束未完成取消请求，并关闭规划执行器。',
})
describe('usv_decision/usv_decision/runtime.py', '与 ROS 解耦的规划工作进程：复用可通行旧路线或重搜栅格，生成 LOS 参考并验证完整船体。', {
    'PlanningRequest': '携带任务版本、目标、状态/地图快照、旧路线和参数副本，隔离回调与工作进程的数据修改。',
    'PlanningResponse': '返回同一版本的规划状态、路线、轨迹、耗时及是否实际进行了全局搜索。',
    'plan_navigation': '先保证地图与状态同系；路线、终点连接和 LOS 扫掠检查逐级通过后才返回可用参考。',
    'LatestPlanWorker': '最多一个执行中任务和一个可替换待处理快照，避免规划积压成无界历史队列。',
    'offer': '新请求覆盖尚未开始的旧请求，并记录替换次数。',
    'poll': '只读取已完成结果；同版本新请求继承已找到路线，但仍用其最新地图重新校验。',
    'invalidate': '丢弃等待请求并尝试取消未运行 Future；运行中结果由上层版本号过滤。',
    'close': '清除待处理工作并等待执行器安全结束。',
}, {
    'connected = np.vstack((request.state.pose, route)) if len(route) else route': '复用路线前把船舶当前位置连到剩余路径，连接段也必须满足膨胀区逃逸规则。',
    'if endpoint_offset < controller_tolerance:': '若栅格终点落在真实目标盘内，按两圆盘中心偏差收缩 LOS 停止半径，确保预测停车盘被包含。',
    'connection = np.vstack((route[-1], request.goal))': '栅格中心在真实目标盘之外时追加真实目标，并先检查新增连接线段。',
    'trajectory.goal = tuple(request.goal[:2])': '控制器的最终到达判断始终使用用户请求坐标，不以量化后的栅格中心代替。',
})

describe('common/common/collision.py', '在原始代价地图上检验矩形船体与插值扫掠路径，物理碰撞只针对致命和未知单元的完整面积。', {
    '_pose_clear': '先用船体轴对齐包围盒筛选单元，再用分离轴定理检测旋转矩形与栅格方块是否相交。',
    'footprint_path_is_clear': '平移线性插值、航向最短弧插值，并按船角运动上界采样及补裕量；无效输入保守返回不可通行。',
}, {
    'x0 = math.floor(math.nextafter((x - extent_x - grid.origin_x) / resolution, -math.inf))': '左/下边界向负无穷取相邻浮点数，确保恰好接触栅格边界也纳入检测。',
    'iy, ix = np.nonzero(grid.data[y0:y1 + 1, x0:x1 + 1] >= 254)': '不把软膨胀带再扩一遍船体；代价 254/255 才进入物理占据检测。',
    'motion_bound = float(np.linalg.norm(delta)) + radius * abs(angle)': '船角最大位移上界由平移长度加外接圆半径乘转角组成，因此纯旋转也需要中间检查。',
    'pad = .5 * motion_bound / steps': '每个采样船体增加半步运动上界，覆盖相邻采样之间的插值运动。',
})
describe('common/common/dynamics.py', '低速平面三自由度 Otter 数值预测，用于命令筛查与回放；保留当前模型阻尼与刚体速度耦合。', {
    '_model_matrices': '组装有效惯性、线性阻尼和二次阻尼，要求惯性对称正定；相同不可变模型复用缓存。',
    'allocate_force': '输入纵向力牛顿与偏航力矩牛顿米，分配左右桨后统一缩放以保留两路比例。',
    'step_dynamics': '使用最大 50 毫秒的 RK4 子步推进带符号船体系 u/v/r 与世界系 x/y/yaw，拒绝侧推输入。',
    'derivative': '船体系动力学计算加速度，世界系运动学旋转 u/v 为位置导数；阻尼系数本身携带力的符号。',
    'braking_command': '对带符号纵向速度和偏航角速度施加阻尼；此处不限制变化率，发布前仍需安全检查。',
    'braking_candidates': '依次提供温和及更强制动候选，每个候选都必须分别通过完整停止尾迹的校验。',
    'predict_control_path': '先完整执行本拍候选，再逐步重新计算温和制动，返回世界位姿序列及是否在时域内停止。',
    'clear_control': '只有预测已停止且含两侧 margin 的完整扫掠船体无碰撞才放行；任何输入或数值异常均拒绝。',
}, {
    'tau = np.array([(command.left + command.right) * model.max_force, 0.,': '左右桨和产生纵向力，右减左乘力臂产生正偏航力矩，横向执行力固定为零。',
    'if not active and math.hypot(predicted.u, predicted.v) <= .02 and abs(predicted.r) <= .01:': '候选本拍执行完后，速度不超过 0.02 米/秒且角速度不超过 0.01 弧度/秒才判停止。',
})
describe('common/common/geodesy.py', 'WGS84 经纬高与局部 ENU 坐标相互转换；海事艏向为真北顺时针度，ROS yaw 为东向起算逆时针弧度。', {
    'heading_to_yaw': '将海事艏向换为 ROS yaw，并规整到 [-π,π)。',
    '_ecef': '先把角度换成弧度，利用椭球卯酉圈曲率半径计算地心地固坐标，单位米。',
    'GeoReference': '不可变地理原点，纬经度以度表示，高度以米表示。',
    'from_world': '从实际 SDF 世界读取参考，仅接受单个 WGS84、ENU 且 heading_deg 为零的世界。',
    '_basis': '构造原点处东、北、天三个单位基向量，以便在 ECEF 与局部 ENU 之间投影。',
    'to_local': '目标与原点 ECEF 做差，再投影到东、北、天轴，输出局部米制坐标。',
    'to_geographic': '局部坐标反投影到 ECEF 后迭代求纬度；极轴与地心退化位置单独处理。',
})
describe('common/common/math_helper.py', '纯数值公共几何工具：角度规整、限幅、四元数偏航提取和路径中心线采样碰撞检查。', {
    'wrap_angle': '将标量或数组角度规整到 [-π,π)，标量输入返回 Python float。',
    'clamp': '检查上下界顺序后把数值限制在闭区间。',
    'quaternion_yaw': '先验证并归一化四元数，再按 ROS 约定提取平面偏航角。',
    'path_is_clear': '中心线每段最多半格间距采样；该工具不覆盖船体尺寸，也不提供软膨胀区逃逸例外。',
})
describe('common/common/models.py', '跨模块纯数值数据契约：船舶状态、物理参数、只读代价地图、规划结果和归一化推进命令。', {
    'AlgorithmStatus': '区分成功、无效输入、无路、超时、真实到达与目标区内减速等待。',
    'VesselState': 'x/y 属于 frame_id 坐标系且单位米；psi 为弧度，u/v 为船体系米/秒，r 为弧度/秒。',
    'world_velocity': '将船体系纵横速度旋转到世界系；姿态改变不应重复旋转已有船体系测量。',
    'USVModel': 'Otter 差动推进参数集合；最大单桨力单位牛顿、质量千克、偏航惯量千克米²、尺寸和力臂为米。',
    'from_mapping': '拒绝未知字段、非有限数及不合理物理范围，当前只支持 Differential 类型的 otter。',
    'GridSnapshot': '地图 data 采用 [行y, 列x] 的 uint8 原始代价，分辨率与原点单位米。',
    '__reduce__': '跨进程 pickle 恢复时重新调用构造校验，重新建立只读数组，避免工作进程改写障碍。',
    'world_to_map': '相对地图原点除以分辨率并向下取整，负向越界不截断回地图。',
    'map_to_world': '返回栅格单元中心坐标，使用半格偏移而不是左下角。',
    'is_blocked': '越出地图视为阻塞；255 按未知策略处理，其余原始值与阈值比较。',
    'PlanResult': '路径为 N×3 位姿数组，携带算法耗时秒和扩展单元数。',
    'ControlCommand': '左右桨和侧推均为 [-1,1] 归一化命令；状态和消息供运行层诊断使用。',
})
describe('common/common/visualizer.py', '将数值路径转为 ROS nav_msgs/Path，延迟导入消息类型，使纯算法无需在导入时依赖 ROS。', {
    'make_path': '共享坐标系和时间戳，逐点把平面 yaw 转为绕 z 轴四元数；缺少航向列时使用零。',
})
describe('usv_navigation/usv_navigation/configuration.py', '统一项目 YAML 与 ROS 参数快照：展开层级、处理兼容别名、验证物理约束并限制在线可改项。', {
    'merge_explicit': '合并显式参数时若同名值不一致立即报错，避免后载入配置悄悄覆盖前者。',
    'flatten': '把嵌套映射转成点分隔参数键，同时校验键类型与重复参数。',
    'normalize_parameters': '统一历史拼写别名，LQR 整数列表转双精度数组，满足 ROS 参数声明类型。',
    'validate_parameters': '检查类型、有限性、枚举、正负范围、轨迹时间关系和 Q/R 维度，再验证船舶模型。',
    'resolve_parameters': '标准化输入后覆盖默认值并完整验证，最终按默认类型保持 ROS double 参数一致。',
    'section': '按前缀选参数并剥离前缀，得到单个纯算法可直接消费的配置。',
    'load_yaml': '使用禁止重复键的安全 YAML 加载器；ROS 导出快照不接受为项目原始配置。',
    'load_project_parameters': '按动力学、控制、轨迹、路径配置的约定层级合并，再形成完整有效参数快照。',
    'validate_update': '只接受既有热参数键，先验证整个候选配置再返回，其他改动要求重启。',
}, {
    'HOT_PARAMETERS = frozenset(': '允许在线改动的是 PID 六个增益与 LOS 四个导引参数；模型、频率、容差等保持启动时值。',
})
describe('usv_navigation/usv_navigation/heartbeat.py', '轻量进程存活与会话跟踪，不导入导航算法；心跳周期和超时均按墙钟秒计。', {
    'PeerSession': '保存当前会话、递增序号、最后接收时刻及已退休会话，防止重放恢复旧进程身份。',
    'observe': '仅接受新会话或当前会话递增序号，重复乱序消息不刷新在线时间。',
    'fresh': '必须已发现会话且墙钟年龄非负、不超过超时，未来或非有限时间均判不新鲜。',
})
describe('usv_navigation/usv_navigation/launch_support.py', '独立业务节点共享的启动配置工具：定位四类项目 YAML、处理 launch 覆盖并构造一致命名空间节点。', {
    'navigation_arguments': '声明命名空间、仿真时间、可选算法覆盖和四份配置路径；空算法覆盖保留 YAML 设置。',
    '_boolean': '只把 true/false 文本解释为布尔值，拒绝其它拼写。',
    'NavigationSettings': '一次解析后的启动快照，包含命名空间、时钟选项、有效参数与来源日志。',
    'navigation_settings': '按项目 YAML、非空算法覆盖、命名空间修正、完整验证的顺序生成启动设置。',
    'navigation_node': '使用当前 Python 解释器启动业务节点，注入有效参数、仿真时间和附加配置。',
})
describe('usv_navigation/usv_navigation/node_runner.py', '统一 ROS 节点初始化、spin 与退出清理；将 SIGTERM 转为正常退出路径，保证业务 destroy_node 得到调用。', {
    'run_node': '关闭 rclpy 默认信号处理，自行保存并恢复 SIGTERM 处理器，finally 中清理节点和上下文。',
})
describe('usv_navigation/usv_navigation/protocol.py', '分布式导航轨迹编解码与心跳接口转出，保留原始轨迹时间、坐标系、八列数值和最终目标。', {
    'trajectory_to_values': '将轨迹快照转为可序列化值；None 显式编码为空点集。',
    'trajectory_from_values': '空点集表示暂无参考；非空跟踪参考必须具有坐标系和最终目标，并复用轨迹构造校验。',
    'trajectory_from_message': '按 POINT_FIELDS 固定顺序读取消息，曲率字段 kappa 对应数组第六列。',
    'set_message_trajectory': '填充轨迹源时间、坐标系与点序列；header 的重传时间不代替 trajectory_stamp。',
})
describe('usv_navigation/usv_navigation/ros_support.py', '决策与控制节点共用的 ROS 输入适配层；每个节点独立保存里程计、代价地图和模块心跳快照。', {
    'seconds': '将 ROS 秒与纳秒时间字段合成浮点秒。',
    'yaw': '通过已验证的四元数工具提取 ROS 平面航向。',
    'NavigationInputs': '声明参数、TF 与输入订阅，按命名空间构造 map/base_link，采用稳态时钟发送心跳。',
    '_validate_parameters': 'ROS 参数写入回调只验证候选值；业务循环在拍边界应用通过的热更新。',
    '_on_heartbeat': '按墙钟接收时间更新对端会话，分别调用首次可用或重启钩子。',
    'peers_ready': '指定角色均需心跳新鲜且 ready 标志为真。',
    '_pose': '按消息时间查询 TF，把来源平面位置与航向变换到本节点 map 坐标系。',
    '_on_odom': '暂存最新里程计并尝试立即转换；TF 未到时留待后续循环重试。',
    '_consume_odom': '要求 twist 已属于本船 base_link，位置经 TF 转换后直接保留船体系 u/v/r 的符号。',
    '_on_map': '只接受同 map 坐标系且原点未旋转的 Nav2 原始地图，按 size_y×size_x 重建只读快照。',
    '_current_footprint_clear': '用最新状态的单个位姿检查完整矩形船体，而非仅检查船舶中心点。',
    '_clear_inputs': '清除状态、地图、待转换里程计与 TF，防止时间重置后继续使用旧输入。',
    'statistics': '输出每类计时样本的分位数以及本进程 CPU/RSS 统计，供诊断报告汇总。',
}, {
    'except TransformException:': '缺少 TF 时保留待处理里程计；格式错误则丢弃，避免反复消费坏数据。',
})
describe('usv_navigation/usv_navigation/state_rules.py', '普通导航就绪状态判定：依次检查状态、地图、参考的新鲜度与坐标系一致性。', {
    'fresh': '时间单位秒；容忍最多 1e-6 秒的微小未来偏差，拒绝明显未来、过期及非有限时间。',
    'readiness': '返回第一个未通过的门限状态；所有输入可用且同系时才进入 NAVIGATING。',
})

TESTS = {
 'test_controllers.py': ('验证 PID/LQR 的历史数值契约、预瞄终点语义、非法输入零输出、在线更新与求解边界。', [
 '核对 PID 差动分配公式及积分项的逐拍累积。', '验证成功调参保留积分，失败更新不改已有配置。', '空轨迹、非有限状态、非正时间步、异系和倒车参考均返回零命令。',
 '真实全局目标决定到达状态，局部时域终点不代替任务目标。', '高速入目标盘只进入 SETTLING，漂出后仍允许继续控制。', '验证命令限幅及船体系速度模不随世界航向改变。', '验证预瞄点随状态时间相对轨迹源时间推进。',
 '逐项核对保留的 LQR 离散 A/B 系数。', '验证收敛增益使离散闭环特征值模小于一，零跟踪误差产生零输入。', '迭代不足或求解超时必须返回零命令，不使用部分增益。', '直接增益暴露 u/v，检查船体系速度不被重复旋转且保留负号。',
 '拒绝 Q/R 维度、正定性和求解参数错误。', '拒绝未实现算法和不支持船型。', '预测参考到达不能使真实船舶提前停车。', '途中停车参考不被目标接近逻辑擅自恢复速度。', '目标半径小于或等于 LOS 去重尺度时必须拒绝。',
 '拒绝不合法目标半径更新时保留 PID 参数及历史。', '验证最小允许半径仍覆盖被 LOS 去重合并的目标位置差。']),
 'test_station_keeping.py': ('验证定点定艏控制的欠驱动约束、阶段滞回、推力接管连续性和确定性闭环收敛。', [
 '正后方目标分别正驶/倒驶，纯横向目标用转向而非侧推。', '验证命令幅值、每拍变化率限制及非法输入立即归零。', 'HOLD 持续有效，偏移超出按精度缩放的滞回门限后恢复。', '安全替代后变化率从实际发布命令开始计算。',
 '导航转定点接管时保持已发布推力的连续性。', '接管值高于定点上限时按变化率渐降，同时始终满足硬件范围。', '多个初始状态收敛后，在固定六十秒窗口同时检查位置和航向合格率。', '施加横向与转动脉冲后不重置控制器也应回到目标。', '拒绝无效精度、非有限角度、过大命令限幅和未知配置。']),
 'test_control_ros.py': ('使用真实 ROS 消息和人工传感器输入验证独立控制节点的会话、停车、参数与时钟边界。', [
 '验证跟踪后取消归零，旧序号或错误控制会话不能恢复任务。', '热参数在控制拍应用，PID 积分历史保持不变。', '当前船体碰撞撤销参考但保留任务，安全后新参考可恢复。', '完成任务与对端重启后旧任务号不能重新激活。', '临时状态过期和仿真暂停归零，时钟倒退清理任务。', '对端发现前的空会话参考不能在之后自动生效。', '不可热改参数不一致导致停止，不能继续旧参考。', '通过 ROS 初始参数选择 LQR 并保留指定预瞄值。']),
 'test_launch_control.py': ('解析控制模块 launch 动作，验证只启动其业务节点并正确合并共享参数，不实际启动子进程。', ['验证节点包/入口/命名空间、仿真时间和默认控制参数。']),
 'test_grid_planning.py': ('验证三类栅格搜索、边界、路径处理、软膨胀起点逃逸及真实快照回放的数值契约。', [
 '空地图路径长度与三次对角加两次轴向步的解析结果一致。', '贯通障碍墙必须无路。', '二维邻接不能把线性索引跨行误当相邻。', '起点或终点占据时明确失败。', '非法或越界端点不能被静默投影回地图。',
 '验证未知策略与原有 254×factor 障碍阈值。', '对角切角是否允许由显式选项控制。', '地图描边使用独立掩码，不改只读快照。', '非零原点和小数分辨率正确转换坐标。', '同格起终点返回单元中心并采用目标航向。',
 '扩展预算用尽只返回超时，不泄露部分路径。', '替换计时器以确定性检验规划时间预算。', '配置错误在构造时立即抛出。', '二维索引可往返且各轴边界独立验证。', '剪枝沿用最后最近路点规则并返回独立副本。',
 '重采样保留端点完整位姿和直线内部切线。', '验证圆弧切线估计及重复位置数值稳定性。', '拒绝零、负数与非有限采样间距。', '门面确实应用启用的重采样间距。', '粗重采样穿障时保留原安全栅格路径。',
 '软膨胀起点可沿严格下降代价退出，且兼容重采样。', '逃逸不能走代价平台、上坡或离开自由区后重新进入。', '逃逸例外绝不允许致命或默认禁止的未知单元。', '即使起点可逃逸，高代价终点仍拒绝。',
 '显式可通行未知格不成为重新进入膨胀区的入口。', '软起点规则同样不能绕过跨行二维边界。', '地图描边上的软起点仍视为硬边界。', '软逃逸保留对角切角开关。', '粗重采样不能跳过必须逐格下降的逃逸前缀。',
 '逐段校验捕捉短暂穿过高代价格和离开地图。', '先校验保存快照哈希，再检验真实 factor=0.25 软起点场景恢复合法路线。']),
 'test_processing.py': ('验证 RDP 简化和 SG 平滑的几何效果、端点保持、短路径及参数异常处理。', [
 '空路径与单点路径保留约定形状和数值。', 'RDP 保留转折并按误差阈值移除小偏移。', '折返点到有限线段的距离不能误用无限直线距离。', '加密保持原分段规则，航向沿最短角弧插值。', '拒绝非法简化阈值与加密间隔。',
 '首尾重合仍保持有限值与端点。', 'SG 短路径原样保留，不产生空输出。', 'SG 默认核匹配原七点权重并保留两个端点。', 'SG 精确保留内部二次曲线且不修改输入。', '拒绝非奇数窗口及非法多项式阶数。']),
 'test_circle.py': ('验证圆航线连续恒速、径向收敛与偏航角速度可行性，覆盖顺逆时针。', ['多圈参考不停车且半径误差和转速受限。', '圆内外初始位置均降低径向误差，速度不附加减速坡道。', '拒绝所需角速度过大及非有限状态。']),
 'test_los.py': ('验证 LOS 导引方向、精确输出时间、非负速度、近目标减速与密集路点切换顺序。', [
 '远离目标时横向误差仍产生正确导引转向。', '各种输出间隔均与实际积分时间匹配，不超过速度允许行程。', '弯曲长路线不产生负参考速度或非有限字段。', '已在目标且路点重复时生成有限静止轨迹。', '空、畸形和非有限输入返回明确失败状态。',
 '未知算法和无效输出时间配置提前拒绝。', '目标外静止船仍得到正初始接近速度。', '临近目标反复规划应继续减速，不恢复巡航速度。', '减速距离下限不改变远场原有速度比例。', '密集路点每输出周期最多推进一次，先使用当前段导引。']),
 'test_trajectory.py': ('验证八字段轨迹插值、角度跨界、重复弧长、只读快照与输入约束。', ['时间和弧长查询用正确下包围点插值全部八列。', '跨 ±π 的航向沿短弧插值。', '停车造成重复弧长时查询仍有定义。', '源数组和回调不能修改已建立轨迹快照。', '拒绝负数及非有限查询值。', '拒绝错误形状、非有限字段、非递增时间和空轨迹查询。']),
 'test_decision_ros.py': ('验证决策目标生命周期，确保旧完成报告和旧规划版本不影响替换后的任务。', ['非法替换目标清掉当前目标并增加任务号。', '旧任务完成报告不能清除新目标，当前任务完成可正常结束。', 'LOS 热更新增加版本并失效化参考，同时保留任务号。', '模块重启清理任务且要求新目标。']),
 'test_distributed_ros.py': ('真实 ROS 消息下的决策/控制集成测试，人工提供时钟、地图、里程计和感知心跳。', [
 '新目标可运行且推进只有一个发布者，取消必须等待控制端真实零输出确认。', '重叠取消各自等待匹配任务和参考序号的零确认。', '旧取消的迟到确认可结束旧服务请求，但不改替换后的当前任务状态。', '新任务替换后，未得到零确认的旧取消仍需超时失败。',
 '目标替换、SETTLING 与真实完成在两节点间一致传播。', 'LOS 和 PID 热更新保留同一任务和 PID 积分。', '控制节点重启后必须等待新目标才能恢复运行。', '非法替换目标使两个模块都停止并归零。']),
 'test_launch_decision.py': ('解析决策模块 launch 动作，验证业务节点、兼容节点名与共享参数，不实际启动子进程。', ['验证只拥有决策节点、保留 usv_simulation 名称并传递命名空间与时钟参数。']),
 'test_collision.py': ('验证完整矩形船体与地图单元面积、平移/旋转扫掠、角度跨界及地图越界的碰撞判定。', ['软膨胀成本不被重复当作物理障碍。', '中心可通行但船艏接触致命或未知单元时必须拒绝。', '侧向间距随船体航向变化，检测完整单元方块。', '平移端点均安全时仍检测中间障碍。', '原地转动端点均安全时仍检测中间船角碰撞。', '跨 ±π 使用短弧，不扫过无关障碍。', '畸形、非有限及部分船体越界均拒绝。']),
 'test_dynamics.py': ('验证带符号三自由度动力学、制动预测与完整船体停止尾迹安全检查。', ['船体系倒驶速度正确旋转到世界系，差动推力产生正确转向。', '制动反向抵消有符号运动且预测最终停止。', '倒驶船艉碰撞及当前重叠均拒绝。', '旋转扫掠检测端点之外的船角障碍。', '非法候选、未建模侧推和时域内不停均保守拒绝。', '零推力自由衰减匹配解析解，不能当作瞬时停车。', '未知格与状态/地图异系均不安全。', '回放已保存的状态和障碍，验证温和制动失败而有界强制动通过。', '前进后退的强制动候选均反向阻尼且没有侧推。', '当前船体已占据时所有制动候选仍必须拒绝。']),
 'test_geodesy.py': ('验证 WGS84/ENU 解析基准、跨半球极区往返、实际世界原点以及海事艏向约定。', ['赤道格林尼治原点与已知 ECEF 轴关系一致。', '不同半球和极区局部坐标往返误差满足容限。', '实际世界地理参考与已知示例点转换一致。', '真北顺时针度正确转换为 ROS 东向逆时针弧度。', '越界、非有限或布尔经纬高必须拒绝。', '世界缺参考、错误坐标系/表面/航向或缺字段时明确报错。']),
 'test_models.py': ('验证状态坐标约定、地图独立只读快照、进程序列化与路径线段检查。', ['地图复制源数组且保持只读，坐标转换严格按二维边界。', '船体系速度旋转及角度规整符合约定。', '跨规划进程序列化后地图仍为独立只读数组。', '路径中间线段穿障不能因路点安全而漏检。', '非法地图数据类型和船舶质量参数必须拒绝。']),
 'test_lightweight_support.py': ('验证感知可用的轻量心跳/生命周期工具不会导入业务算法，并保留运行器兼容别名。', ['在独立解释器导入工具，检查 sys.modules 未加载业务或导航算法包。', 'ROS 支持模块导出的 run_node 与共享运行器为同一个对象。']),
 'test_protocol.py': ('验证轨迹字段无损传输、心跳拒旧及退休进程不可重放恢复。', ['完整轨迹往返保留八列、源时间、坐标系、目标及只读标志。', '空参考只编码为 None，不在编解码层改变任务。', '非有限轨迹消息必须被构造校验拒绝。', '已退休进程的迟到心跳不能恢复旧会话。', '重复乱序心跳不刷新存活时间，新序号可刷新。', '未来或非有限心跳时间不能视为新鲜。', '公开协议中的心跳类型和周期/超时别名与轻量模块一致。']),
}
for filename, (header, descriptions) in TESTS.items():
    path = resolve('/test/' + filename)
    headers[path] = header
    text = source(path)
    lines = text.splitlines()
    nodes = sorted([n for n in ast.walk(ast.parse(text)) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name.startswith('test_')], key=lambda n: n.lineno)
    assert len(nodes) == len(descriptions), (filename, len(nodes), len(descriptions))
    for n, comment in zip(nodes, descriptions):
        line = min([n.lineno] + [d.lineno for d in n.decorator_list]) - 1
        notes[path][line].append(re.match(r'\s*', lines[line]).group() + '# ' + comment)

PACKAGE_DESCRIPTIONS = {
    'controller': 'Otter PID、LQR 与定点保持纯 Python 控制算法',
    'usv_control': '普通导航控制节点及推进输出适配器',
    'path_planner': 'A*/Dijkstra/GBFS 栅格规划和路径几何工具',
    'path_processor': 'RDP 路径简化与 SG 几何平滑',
    'trajectory_planner': 'LOS 轨迹生成、圆航线与八字段插值',
    'usv_decision': '普通导航任务管理和异步路径/轨迹规划节点',
    'common': '船舶状态、地图、动力学和公共数值工具',
    'usv_navigation': '共享导航参数、ROS 输入与会话协议支持',
    'usv_navigation_interfaces': '决策、控制、感知间的导航传输消息',
}
for item in items:
    path = item['path']
    parts = Path(path).parts
    package = parts[3]
    name = parts[-1]
    desc = PACKAGE_DESCRIPTIONS[package]
    if name == 'package.xml':
        headers[path] = f'{package} 包清单：声明{desc}所需的构建、运行及测试依赖。'
    elif name == 'setup.cfg':
        headers[path] = f'将 {package} 的开发和安装脚本定位到 lib/{package}，符合 ROS 2 包可执行文件查找约定。'
    elif name == 'setup.py':
        headers[path] = f'安装 {package}：打包{desc}，并注册 ament 资源索引和包元数据。'
        text = source(path)
        for i, line in enumerate(text.splitlines()):
            if 'entry_points=' in line:
                notes[path][i].append('    # console_scripts 将 ROS 可执行入口映射到对应节点模块的 main 函数。')
            elif 'config_files = ' in line:
                notes[path][i].append('# 递归安装四类 YAML 并保留配置子目录，启动器据此定位实际算法配置。')
    elif name == '__init__.py':
        if '/graph_planner/' in path:
            headers[path] = '导出八邻域搜索器 AStarPathPlanner，供路径规划工厂统一引用。'
        elif '/utils/' in path:
            headers[path] = '导出 create_planner 工厂，统一路径规划器的创建入口。'
        else:
            headers[path] = f'{package} Python 包入口：提供{desc}的模块边界。'
    elif name.endswith('.launch.py'):
        headers[path] = f'独立启动{desc}，复用共享的项目 YAML、命名空间与仿真时钟参数。'
        describe(path, headers[path], {'_launch': '在 launch 上下文可用后解析参数，返回配置来源日志及本业务节点动作。',
                                     'generate_launch_description': '先声明共享参数，再用 OpaqueFunction 延迟求值，确保覆盖参数已经生效。'})

describe('usv_navigation_interfaces/CMakeLists.txt', '生成四类导航 ROS 消息的语言绑定，并导出运行时依赖供决策、控制及感知包使用。', anchors={
    'rosidl_generate_interfaces(${PROJECT_NAME}': '声明消息文件和被引用的标准头/二维目标位姿类型。',
})
describe('config/path_planner/common_planner_params.yaml', '共享全局规划配置：选择栅格算法、设置规划预算/输入超时，并配置膨胀阈值及重采样。', anchors={
    'planning_frequency: 5.0': '规划投递频率为 Hz；其后的 state/map/trajectory/planning_timeout 单位均为秒。',
    'max_expansions: 200000': '搜索扩展单元数硬上限，和耗时预算共同限制一次搜索。',
    'obstacle_factor: 0.25': '高代价阻塞阈值为 254×factor；起点软膨胀区仍可按实现中的严格下降规则退出。',
    'resample_ratio: 0.5': '历史键名 ratio 在当前实现中表示弧长采样间距，单位米。',
    'convert_offset: 0.0': '保留历史兼容参数；当前坐标转换使用 GridSnapshot 的原点与分辨率。',
})
describe('config/trajectory_planner/trajectory_planner_params.yaml', '共享 LOS 导引参数：时间单位秒，几何距离单位米，偏航角速度单位弧度/秒。', anchors={
    'trajectory_time_interval: 0.5': '输出采样间隔与下一项预测总时域一起决定轨迹长度。',
    'switch_dist: 5.0': '接近当前航段终点到此距离后可推进路点；每个输出周期最多推进一次。',
    'lookahead_distance: 1.0': '横向偏差经 atan2(偏差,预瞄距离) 转为航向修正，较小距离使修正更敏感。',
    'slowdown_distance: 3.0': '接近终点速度剖面的最小分母尺度，防止短路线重规划恢复满巡航速度。',
})
describe('config/robots/otter/controller_params_otter.yaml', 'Otter 普通导航控制参数；PID 与 LQR 共用真实目标距离/速度条件，预瞄时间以秒表示。', anchors={
    'control_frequncy: 10.0': '历史拼写由 configuration.normalize_parameters 映射到 control_frequency，单位 Hz。',
    'pid_controller:': '速度环使用平面速度模，航向环使用最短角误差；这组增益直接标定为归一化双桨命令。',
    'lqr_controller:': 'LQR 离散时间和求解超时单位秒；迭代上限与残差容差限制每拍 Riccati 求解。',
    'Q_matrix_diag: [1, 1, 100, 10, 10, 10]': 'Q 依次惩罚 x、y、psi、u、v、r 误差，R 对应左右两路输入。',
})
describe('config/robots/otter/dynamics_otter.yaml', 'Otter 差动船模参数：质量千克、惯量千克米²、尺寸米、速度米/秒；阻尼系数已经带作用力符号。', anchors={
    'max_force: 50.0': '单个纵向推进器满归一化命令对应的最大力，单位牛顿。',
    'D: 0.395': '纵向推力相对船体中心线的横向力臂，偏航力矩由右减左推力乘 D 得到。',
    'x_dot_u: 5.0': '附加质量/惯量参与有效惯性，纵横向使用质量项，偏航使用惯量项。',
    'x_u: -20.0': '线性阻尼带负号，对正向速度产生反向阻力；勿在使用处再次翻转符号。',
    'jerk_min: -2.0': '加速度变化率约束的单位为米/秒³。',
})

MESSAGE_NOTES = {
 'ControlReport.msg': ('控制执行报告：携带实际桨命令、当前任务身份和撤销/完成信号，用于决策状态与取消确认。', {
     'string control_session': '控制和规划会话标识进程实例，拒绝重启前的报告。',
     'uint64 task_id': '任务号和所执行参考序号共同关联一次取消或到达事件。',
     'bool revoke_reference': '请求决策端撤销当前参考；不同于任务已经完成。',
     'float64 left': 'left/right/lateral 为实际发布的归一化命令，合法范围 [-1,1]。',
     'string metrics_json': 'JSON 字符串携带计时分位数、事件计数与进程资源统计。'}),
 'ModuleHeartbeat.msg': ('模块心跳：以进程会话与单调序号识别在线状态和重启，接收端使用本地墙钟计算新鲜度。', {
     'string module': '角色名用于路由到 decision、control 或 perception 对端记录。',
     'uint64 sequence': '同一 session 的序号必须递增；重放或乱序不刷新存活时间。',
     'bool ready': '进程是否准备就绪，reason 可说明未就绪原因。'}),
 'NavigationReference.msg': ('决策发送的导航快照：绑定三方进程会话与任务版本，空轨迹点可暂停仍活动的任务。', {
     'std_msgs/Header header': 'header 表示本次发送时间/坐标系；轨迹年龄单独由 trajectory_stamp 计算。',
     'string planner_session': '规划、控制、感知三方会话必须与当前发现的实例匹配。',
     'uint64 task_id': 'task_id 标识目标生命周期，revision 区分规划版本，sequence 标识快照内容更新。',
     'geometry_msgs/Pose2D goal': '真实导航目标 (x米, y米, theta弧度)，完成判定不使用量化后的栅格终点。',
     'string parameters_json': '完整有效参数快照，控制端只允许既有 HOT 参数与启动配置发生差异。',
     'float64 trajectory_stamp': '轨迹生成所依据的源时间秒，重传不得刷新。'}),
 'TrajectoryPoint.msg': ('一个时间参数轨迹点，字段顺序与纯 Python N×8 轨迹数组保持一致。', {
     'float64 time': '相对 trajectory_stamp 的时间，单位秒。',
     'float64 s': '累计路径弧长，单位米，允许停车段重复。',
     'float64 x': 'x/y 为轨迹坐标系位置，单位米；theta 为 ROS 平面航向弧度。',
     'float64 kappa': '曲率单位 1/米，velocity 为米/秒，acceleration 为米/秒²。'}),
}
for filename, (header, anchors) in MESSAGE_NOTES.items():
    describe('/msg/' + filename, header, anchors=anchors)

assert len(headers) == len(items), [v['path'] for v in items if v['path'] not in headers]
counts = collections.Counter()
for item in items:
    path = item['path']
    original = (WORK / 'before' / path).read_bytes()
    current = (ROOT / path).read_bytes()
    assert current == original, f'File changed since snapshot: {path}'
    bom = b'\xef\xbb\xbf' if original.startswith(b'\xef\xbb\xbf') else b''
    raw = original[len(bom):]
    lines = raw.splitlines(keepends=True)
    newline = b'\r\n' if b'\r\n' in raw else b'\n'
    index = 0
    if path.endswith('.xml'):
        while index < len(lines) and lines[index].lstrip().startswith(b'<?'):
            index += 1
        header = '<!-- ' + headers[path] + ' -->'
    else:
        while index < min(2, len(lines)) and (lines[index].startswith(b'#!') or re.search(br'coding[:=]', lines[index])):
            index += 1
        header = '# ' + headers[path]
    notes[path][index].insert(0, header)
    # Existing lines are copied as bytes; no normalization, replacement or new empty lines.
    output = bytearray(bom)
    for i, line in enumerate(lines):
        for note in notes[path].get(i, []):
            output.extend(note.encode('utf-8') + newline)
            counts['comment_lines'] += 1
        output.extend(line)
    assert all(i < len(lines) for i in notes[path]), path
    output = bytes(output)
    old_text = original.decode('utf-8-sig')
    new_text = output.decode('utf-8-sig')
    if path.endswith('.py'):
        assert ast.dump(ast.parse(old_text), include_attributes=False) == ast.dump(ast.parse(new_text), include_attributes=False), path
        def tokens(text):
            return [(t.type, t.string) for t in tokenize.generate_tokens(io.StringIO(text).readline)
                    if t.type not in (tokenize.COMMENT, tokenize.NL)]
        assert tokens(old_text) == tokens(new_text), path
        counts['python_ast_and_tokens'] += 1
    elif path.endswith('.yaml'):
        assert yaml.safe_load(old_text) == yaml.safe_load(new_text), path
        counts['yaml_values'] += 1
    elif path.endswith('.xml'):
        assert ET.tostring(ET.fromstring(old_text)) == ET.tostring(ET.fromstring(new_text)), path
        counts['xml_structure'] += 1
    before_lines, after_lines = original.splitlines(keepends=True), output.splitlines(keepends=True)
    matcher = difflib.SequenceMatcher(a=before_lines, b=after_lines, autojunk=False)
    for operation, a, b, c, d in matcher.get_opcodes():
        assert operation in ('equal', 'insert'), (path, operation)
        if operation == 'insert':
            assert all(line.strip().startswith((b'#', b'<!--')) for line in after_lines[c:d]), path
    (ROOT / path).write_bytes(output)
    counts['files'] += 1
(WORK / 'navigation_work/result.json').write_text(json.dumps(dict(counts), indent=2), encoding='utf-8')
print(json.dumps(dict(counts), indent=2))
