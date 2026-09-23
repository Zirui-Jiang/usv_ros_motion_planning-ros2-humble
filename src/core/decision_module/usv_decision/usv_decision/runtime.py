# 与 ROS 解耦的规划工作进程：复用可通行旧路线或重搜栅格，生成 LOS 参考并验证完整船体。
"""ROS-independent planning worker and navigation freshness rules."""
from dataclasses import dataclass, replace
import time
import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, USVModel, VesselState
from common.collision import footprint_path_is_clear
from path_planner.path_planner_core import PathPlannerCore, path_is_clear_with_inflation_escape
from path_processor import ShortcutPathProcessor
from trajectory_planner.trajectory import Trajectory
from trajectory_planner.trajectory_planner_factory import create_trajectory_planner
from trajectory_planner.route_trajectory import RouteTrajectoryPlanner
from usv_navigation.configuration import C, T, section
from usv_navigation.state_rules import fresh, readiness


def _warmup_worker():
    """Start the spawned planner process and load its module graph early."""
    return None


# 携带任务版本、目标、状态/地图快照、旧路线和参数副本，隔离回调与工作进程的数据修改。
@dataclass(frozen=True)
class PlanningRequest:
    version: int
    goal: tuple
    state: VesselState
    grid: GridSnapshot
    route: np.ndarray
    parameters: dict
    desired_speed_mps: float | None = None
    waypoints: tuple = ()  # remaining ordered (x, y, theta, speed_mps) anchors
    waypoint_radius: float = 1.5


# 返回同一版本的规划状态、路线、轨迹、耗时及是否实际进行了全局搜索。
@dataclass(frozen=True)
class PlanningResponse:
    version: int
    status: AlgorithmStatus
    route: np.ndarray
    trajectory: Trajectory
    elapsed: float
    message: str = ''
    searched: bool = False
    expanded: int = 0


# 先保证地图与状态同系；路线、终点连接和 LOS 扫掠检查逐级通过后才返回可用参考。
def plan_navigation(request):
    """Runs in one spawned process; consumes only immutable/copy snapshots."""
    if request.waypoints:
        return plan_route_navigation(request)
    begun = time.perf_counter()
    cfg = request.parameters
    route = np.empty((0, 3))
    searched, expanded = False, 0

    def result(status, trajectory=None, message=''):
        return PlanningResponse(request.version, status, route,
                                trajectory if trajectory is not None else Trajectory(),
                                time.perf_counter() - begun, message, searched, expanded)

    try:
        if request.state.frame_id != request.grid.frame_id:
            return result(AlgorithmStatus.INVALID_INPUT, message='state/map frame mismatch')
        planner_cfg = section(cfg, 'path_planner.')
        planner_cfg.update(planner_name=cfg['planner_name'], obstacle_factor=cfg['obstacle_factor'],
                           timeout=cfg['navigation.planning_timeout'],
                           max_expansions=cfg['navigation.max_expansions'])
        if cfg['planner_name'] == 'hybrid_astar':
            planner_cfg.update(footprint_length=cfg['navigation.footprint_length'],
                               footprint_width=cfg['navigation.footprint_width'])
        planner = PathPlannerCore(planner_cfg)
        preserves_kinematics = getattr(planner.planner, 'preserves_kinematics', False)
        deadline = begun + cfg['navigation.planning_timeout']
        if preserves_kinematics:
            from path_planner.graph_planner.hybrid_path_reuse import prepare_cached_path, shortcut_straight_connections
            # 全局路径保持原几何，仅裁掉已走部分；局部轨迹才从最新实测位姿出发。
            cached = prepare_cached_path(request.route, request.state.pose, request.goal,
                                         request.grid, planner_cfg, deadline=deadline)
            route = np.empty((0, 3)) if cached is None else cached
        else:
            route = planner.prune(request.route, request.state)
        threshold = 254.0 * cfg['obstacle_factor']
        # Check the vessel-to-route connection and remaining segments on the
        # latest map with the same policy that accepted the global route.
        # A legal soft descent must not become a false invalidation merely
        # because the vessel is still in its original inflated start region.
        # 复用路线前把船舶当前位置连到剩余路径，连接段也必须满足膨胀区逃逸规则。
        connected = np.vstack((request.state.pose, route)) if len(route) else route
        if not len(route) or (not preserves_kinematics and not path_is_clear_with_inflation_escape(
                request.grid, connected, threshold, planner.planner.unknown_is_obstacle,
                outline_map=planner.planner.outline_map,
                allow_corner_cutting=planner.planner.allow_corner_cutting)):
            searched = True
            found = planner.make_plan(request.state.pose, request.goal, request.grid)
            expanded = found.expanded
            route = found.path
            if not found.success:
                return result(found.status, message=found.message)
            if preserves_kinematics:
                route = shortcut_straight_connections(route, request.grid, planner_cfg, deadline=deadline)
        # Build a request-local model so a mission speed never mutates the
        # shared ROS parameter snapshot used by later goals.
        dynamics = section(cfg, 'dynamics.')
        if request.desired_speed_mps is not None:
            dynamics['cruising_speed'] = request.desired_speed_mps
        model = USVModel.from_mapping(dynamics)
        los_config = section(cfg, T + 'los.')
        if preserves_kinematics:
            los_config['follow_dense_path'] = True
        controller_tolerance = cfg[C + cfg[C + 'controller_name'] + '_controller.goal_distance_tolerance']
        endpoint_offset = float(np.linalg.norm(route[-1, :2] - request.goal[:2]))
        # 若栅格终点落在真实目标盘内，按两圆盘中心偏差收缩 LOS 停止半径，确保预测停车盘被包含。
        if endpoint_offset < controller_tolerance:
            # Preserve the grid endpoint and its final approach direction.
            # By the triangle inequality, this LOS stop disk lies entirely
            # within the controller's disk around the actual requested goal.
            los_config['goal_tolerance'] = min(
                los_config['goal_tolerance'], controller_tolerance - endpoint_offset)
        else:
            if preserves_kinematics:
                return result(AlgorithmStatus.NO_PATH, message='Hybrid A* did not connect the exact goal')
            # A centre outside/on the actual-goal disk cannot support a
            # positive LOS stop radius. Keep it and validate a new connection.
            # 栅格中心在真实目标盘之外时追加真实目标，并先检查新增连接线段。
            connection = np.vstack((route[-1], request.goal))
            if not path_is_clear_with_inflation_escape(
                    request.grid, connection, threshold, planner.planner.unknown_is_obstacle,
                    outline_map=planner.planner.outline_map,
                    allow_corner_cutting=planner.planner.allow_corner_cutting):
                return result(AlgorithmStatus.NO_PATH, message='goal connection failed grid collision validation')
            route = np.vstack((route, request.goal))
            los_config['goal_tolerance'] = min(los_config['goal_tolerance'], controller_tolerance)
        guidance = create_trajectory_planner(cfg[T + 'planner_name'], model, los_config)
        generated = guidance.plan(route, request.state)
        if generated.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            return result(generated.status, message=generated.message)
        trajectory = generated.trajectory
        # The exact requested goal, not a grid-cell centre, defines completion.
        # 控制器的最终到达判断始终使用用户请求坐标，不以量化后的栅格中心代替。
        trajectory.goal = tuple(request.goal[:2])
        # A* retains its original soft-cost threshold. Local collision safety
        # checks the complete hull against lethal/unknown cell areas instead:
        # rejecting the centre at every soft inflated cost can deadlock a
        # physically clear turn, while centre-only checks miss bow collisions.
        if not footprint_path_is_clear(request.grid, trajectory.points[:, [2, 3, 4]],
                                       cfg['navigation.footprint_length'], cfg['navigation.footprint_width']):
            return result(AlgorithmStatus.NO_PATH, message='LOS swept footprint intersects lethal or unknown map cells')
        return result(generated.status, trajectory)
    except TimeoutError as error:
        return result(AlgorithmStatus.TIMEOUT, message=str(error))
    except (ValueError, TypeError, ArithmeticError) as error:
        return result(AlgorithmStatus.INVALID_INPUT, message=str(error))


def plan_route_navigation(request):
    """Validate exact anchor connections, then predict a bounded safe prefix."""
    begun = time.perf_counter()
    route, legs, speeds = np.empty((0, 3)), [], []
    expanded, searched = 0, False
    def result(status, trajectory=None, message=''):
        return PlanningResponse(request.version, status, route,
                                trajectory if trajectory is not None else Trajectory(),
                                time.perf_counter()-begun, message, searched, expanded)
    try:
        cfg = request.parameters
        deadline = begun+cfg['navigation.planning_timeout']
        points = np.asarray(request.waypoints, dtype=float)
        model = USVModel.from_mapping(section(cfg, 'dynamics.'))
        if (points.ndim != 2 or points.shape[1] != 4 or not len(points)
                or not np.isfinite(points).all() or np.any(points[:, 3] <= 0)
                or np.any(points[:, 3] > model.velocity_max)):
            return result(AlgorithmStatus.INVALID_INPUT, message='invalid ordered waypoints')
        if request.state.frame_id != request.grid.frame_id:
            return result(AlgorithmStatus.INVALID_INPUT, message='state/map frame mismatch')
        planner_cfg = section(cfg, 'path_planner.')
        planner_cfg.update(planner_name=cfg['planner_name'], obstacle_factor=cfg['obstacle_factor'],
                           max_expansions=cfg['navigation.max_expansions'])
        if cfg['planner_name'] == 'hybrid_astar':
            planner_cfg.update(footprint_length=cfg['navigation.footprint_length'],
                               footprint_width=cfg['navigation.footprint_width'])
        # 从上次验证的前缀按必经点切回单航段；每段单独复用，快捷连接不能跨必经点。
        cached_route = np.asarray(request.route, dtype=float)
        if (cached_route.ndim != 2 or cached_route.shape[1] != 3
                or not np.isfinite(cached_route).all()):
            cached_route = np.empty((0, 3))
        elif len(cached_route) > 1:
            # 相邻航段各自保留端点，堆叠后只合并完全相同的共享位姿，绝不合并原地转向。
            same_position = np.linalg.norm(np.diff(cached_route[:, :2], axis=0), axis=1) <= 1e-10
            same_heading = np.abs((np.diff(cached_route[:, 2])+np.pi) % (2*np.pi)-np.pi) <= 1e-10
            cached_route = cached_route[np.r_[True, ~(same_position & same_heading)]]
        cache_cursor = 0
        controller = C+cfg[C+'controller_name']+'_controller.'
        guidance = RouteTrajectoryPlanner(model, section(cfg, T+'los.'), request.waypoint_radius,
                                          cfg[controller+'lookahead_time'])
        start = np.asarray(request.state.pose)
        prefix_message = ''
        # 当前点之后不再按预瞄距离截断，后面的航点这次都搜索。
        # 仍用规划时间和最多 32 段挡住过长的航路。
        for anchor in points:
            remaining = deadline-time.perf_counter()
            if remaining <= 0:
                return result(AlgorithmStatus.TIMEOUT, message='route search exceeded total planning budget')
            # Optional lookahead search must leave time to generate and check
            # a stopping reference on the already validated current prefix.
            planner_cfg['timeout'] = .7*remaining if legs else remaining
            planner = PathPlannerCore(planner_cfg)
            preserves_kinematics = getattr(planner.planner, 'preserves_kinematics', False)
            cached_leg = None
            if preserves_kinematics:
                from path_planner.graph_planner.hybrid_path_reuse import prepare_cached_path, shortcut_straight_connections
                candidates = cached_route[cache_cursor:]
                if len(candidates):
                    angles = (candidates[:, 2]-anchor[2]+np.pi) % (2*np.pi)-np.pi
                    matches = np.flatnonzero((np.linalg.norm(candidates[:, :2]-anchor[:2], axis=1) < 1e-7)
                                             & (np.abs(angles) < 1e-7))
                    if len(matches):
                        end = cache_cursor + int(matches[0])
                        cached_leg = prepare_cached_path(cached_route[cache_cursor:end+1], start,
                            anchor[:3], request.grid, planner_cfg, deadline=deadline)
                        cache_cursor = end
                    else:
                        # 旧前缀不包含当前点时，后面几段也不能越过它抢先复用。
                        cache_cursor = len(cached_route)
            if cached_leg is None:
                searched = True
                # A* 剪枝推迟到精确端点接入之后；不要先剪格中心再重采样再剪一次。
                found = planner.make_plan(start, anchor[:3], request.grid,
                                          postprocess=not planner.shortcut_enabled)
                expanded += found.expanded
                if not found.success:
                    if legs:
                        prefix_message = f'safe prefix; next mandatory anchor pending ({found.status.value}): {found.message}'
                        break
                    return result(found.status, message=found.message)
                if preserves_kinematics:
                    leg = shortcut_straight_connections(found.path, request.grid, planner_cfg, deadline=deadline)
            else:
                leg = cached_leg
            def valid(candidate):
                return path_is_clear_with_inflation_escape(
                    request.grid, candidate, 254.*cfg['obstacle_factor'],
                    planner.planner.unknown_is_obstacle, outline_map=planner.planner.outline_map,
                    allow_corner_cutting=planner.planner.allow_corner_cutting, deadline=deadline)
            if not preserves_kinematics:
                # Keep requested endpoints rather than cell centers. Simplify only
                # inside this leg; crossing a mandatory anchor is never an option.
                connected = np.vstack((start, found.path, anchor[:3]))
                if not valid(connected):
                    if legs:
                        prefix_message = 'safe prefix; next mandatory anchor connection is blocked'
                        break
                    return result(AlgorithmStatus.NO_PATH, message='exact mandatory anchor connection is blocked')
                # Dijkstra/GBFS 的多点链原本就做直连简化，保留其行为；
                # 普通 A* 可显式关闭，以比较原始路径和剪枝路径。
                if planner.shortcut_enabled or cfg['planner_name'] != 'astar':
                    leg = ShortcutPathProcessor(valid).process(connected, deadline=deadline)
                else:
                    leg = connected
            legs.append(leg)
            speeds.append(float(anchor[3]))
            route = np.vstack(legs)
            start = anchor[:3]
            if len(legs) >= 32:
                break
        generated = guidance.plan(legs, speeds, request.state,
                                  goal=tuple(points[-1, :2]) if len(points) == 1 else None,
                                  deadline=deadline)
        if generated.status != AlgorithmStatus.SUCCESS:
            return result(generated.status, message=generated.message)
        if not footprint_path_is_clear(request.grid, generated.trajectory.points[:, [2, 3, 4]],
                                       cfg['navigation.footprint_length'], cfg['navigation.footprint_width']):
            return result(AlgorithmStatus.NO_PATH, message='route swept footprint intersects lethal or unknown map cells')
        if time.perf_counter() >= deadline:
            return result(AlgorithmStatus.TIMEOUT, message='route validation exceeded total planning budget')
        return result(AlgorithmStatus.SUCCESS, generated.trajectory, prefix_message)
    except TimeoutError as error:
        return result(AlgorithmStatus.TIMEOUT, message=str(error))
    except (ValueError, TypeError, ArithmeticError) as error:
        return result(AlgorithmStatus.INVALID_INPUT, message=str(error))


# 最多一个执行中任务和一个可替换待处理快照，避免规划积压成无界历史队列。
class LatestPlanWorker:
    """One executing job and one replaceable pending snapshot, no unbounded queue."""
    def __init__(self, executor=None):
        if executor is None:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor
            executor = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn'))
            self._warmup_future = executor.submit(_warmup_worker)
            self._warmup_future.result()
        else:
            self._warmup_future = None
        self.executor = executor
        self.future = None
        self.pending = None
        self.replaced = 0

    # 新请求覆盖尚未开始的旧请求，并记录替换次数。
    def offer(self, request):
        if self.pending is not None:
            self.replaced += 1
        self.pending = request
        self._start()

    def _start(self):
        if self.future is None and self.pending is not None:
            request, self.pending = self.pending, None
            self.future = self.executor.submit(plan_navigation, request)

    # 只读取已完成结果；同版本新请求继承已找到路线，但仍用其最新地图重新校验。
    def poll(self):
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        try:
            result = future.result()
            # A request queued while the first global search is running still
            # contains an empty route. Carry the completed route into that
            # request before dispatch, otherwise process startup causes a
            # second search from a different cell as soon as the boat moves.
            # Keep the latest state, map and parameters: plan_navigation will
            # revalidate this route against that map before reusing it.
            if (self.pending is not None and self.pending.version == result.version
                    and result.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED)):
                self.pending = replace(self.pending, route=result.route.copy())
            return result
        finally:
            self._start()

    # 丢弃等待请求并尝试取消未运行 Future；运行中结果由上层版本号过滤。
    def invalidate(self):
        self.pending = None
        if self.future is not None and self.future.cancel():
            self.future = None

    # 清除待处理工作并等待执行器安全结束。
    def close(self):
        self.invalidate()
        if self._warmup_future is not None:
            self._warmup_future.cancel()
        self.executor.shutdown(wait=True, cancel_futures=True)
