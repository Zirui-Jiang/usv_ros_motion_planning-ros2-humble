# 规划运行时：按任务意图分派规划，并以一个执行任务加一个可替换待办限制积压。
"""Bounded task planning dispatch and freshness checks; no simulation dependency."""
from dataclasses import replace
import math
from common.models import AlgorithmStatus
from .planning_types import PlanningRequest, PlanningResponse
from . import navigation
from .missions.planning import plan_mission


# 时间戳必须有限且落在允许时窗；明显来自未来的数据也视为失效。
def fresh(stamp, now, timeout):
    """Future data is invalid too (clock rewind or mismatched clock domain)."""
    return all(math.isfinite(v) for v in (stamp, now, timeout)) and -1e-6 <= now - stamp <= timeout


# 控制前依次检查状态、地图、参考轨迹的新鲜度及坐标系一致性。
def readiness(now, state, grid, trajectory, config):
    if state is None or not fresh(state.stamp, now, config['navigation.state_timeout']):
        return 'WAITING_STATE'
    if grid is None or not fresh(grid.stamp, now, config['navigation.map_timeout']):
        return 'WAITING_MAP'
    if state.frame_id != grid.frame_id:
        return 'FRAME_MISMATCH'
    if trajectory is None or trajectory.empty() or not fresh(
            trajectory.stamp, now, config['navigation.trajectory_timeout']):
        return 'WAITING_REFERENCE'
    if trajectory.frame_id != state.frame_id:
        return 'FRAME_MISMATCH'
    return 'NAVIGATING'


# 普通目标进入基础导航链，携带任务意图时进入任务专用规划。
def plan_navigation(request):
    """Compatibility dispatch for ordinary goals and mission intents."""
    if request.mission_intent is None:
        return navigation.plan_navigation(request)
    return plan_mission(request)


# 单进程执行较重搜索；主 ROS 回调只提交快照和轮询结果。
class LatestPlanWorker:
    """One executing job and one replaceable pending snapshot, no unbounded queue."""
    def __init__(self, executor=None):
        if executor is None:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor
            executor = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn'))
        self.executor = executor
        self.future = None
        self.pending = None
        self.replaced = 0

    # 新请求覆盖尚未执行的待办，优先处理最新状态而非排长队。
    def offer(self, request):
        if self.pending is not None:
            self.replaced += 1
        self.pending = request
        self._start()

    # 只有当前任务空闲时才提交待办，保证同时最多一个规划搜索。
    def _start(self):
        if self.future is None and self.pending is not None:
            request, self.pending = self.pending, None
            self.future = self.executor.submit(plan_navigation, request)

    # 取回已完成结果；同版本普通航路可转交给待办，待办仍保留其最新状态和地图。
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
                    and result.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED)
                    and (self.pending.mission_intent is None
                         or self.pending.mission_intent.kind != 'circle')):
                self.pending = replace(self.pending, route=result.route.copy())
            return result
        finally:
            self._start()

    # 丢弃待办并尝试取消未启动任务；已经执行的结果仍须由节点版本号过滤。
    def invalidate(self):
        self.pending = None
        if self.future is not None and self.future.cancel():
            self.future = None

    # 关闭工作进程前清理待办，并等待正在执行的任务结束。
    def close(self):
        self.invalidate()
        self.executor.shutdown(wait=True, cancel_futures=True)
