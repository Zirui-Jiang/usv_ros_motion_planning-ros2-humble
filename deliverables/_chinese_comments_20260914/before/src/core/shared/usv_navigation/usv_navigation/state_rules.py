"""Original navigation input validity rules, independent of planning imports."""
import math


def fresh(stamp, now, timeout):
    return all(math.isfinite(v) for v in (stamp, now, timeout)) and -1e-6 <= now - stamp <= timeout


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
