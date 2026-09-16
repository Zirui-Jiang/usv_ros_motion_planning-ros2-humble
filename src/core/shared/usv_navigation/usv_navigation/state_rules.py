# 普通导航就绪状态判定：依次检查状态、地图、参考的新鲜度与坐标系一致性。
"""Original navigation input validity rules, independent of planning imports."""
import math


# 时间单位秒；容忍最多 1e-6 秒的微小未来偏差，拒绝明显未来、过期及非有限时间。
def fresh(stamp, now, timeout):
    return all(math.isfinite(v) for v in (stamp, now, timeout)) and -1e-6 <= now - stamp <= timeout


# 返回第一个未通过的门限状态；所有输入可用且同系时才进入 NAVIGATING。
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
