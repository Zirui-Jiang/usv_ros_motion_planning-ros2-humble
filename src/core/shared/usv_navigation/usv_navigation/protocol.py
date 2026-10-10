# 分布式导航轨迹编解码与心跳接口转出，保留原始轨迹时间、坐标系、八列数值、最终目标和艇附近的航线折线。
"""ROS-independent session bookkeeping and lossless trajectory conversion."""
import numpy as np

from trajectory_planner.trajectory import Trajectory
from .heartbeat import PeerSession, HEARTBEAT_PERIOD, PEER_TIMEOUT

POINT_FIELDS = ('time', 's', 'x', 'y', 'theta', 'kappa', 'velocity', 'acceleration')
# 单条参考携带的航线折线点数上限；0.5 米间距时约覆盖 128 米。
ROUTE_WINDOW_LIMIT = 256


# 航控只需艇附近的一段航线来求横向偏差：从离艇最近的路点起向后留 behind 米、向前取 ahead 米，点数不超过上限。
def route_window(route, position, ahead, behind=0., limit=ROUTE_WINDOW_LIMIT):
    """Return the (M, 2) part of ``route`` around the waypoint nearest to ``position``.

    Distances are measured along the route. The point that first reaches each
    distance is included, so the window always covers the requested lengths
    when the route is long enough.
    """
    points = np.asarray(route, dtype=float)
    if points.size == 0:
        return np.empty((0, 2))
    if points.ndim != 2 or points.shape[1] < 2 or not np.isfinite(points[:, :2]).all():
        raise ValueError('route must be finite with shape (N, >=2)')
    centre = np.asarray(position, dtype=float)[:2]
    if centre.shape != (2,) or not np.isfinite(centre).all():
        raise ValueError('position must be a finite (x, y) pair')
    if not all(np.isfinite(value) and value >= 0 for value in (ahead, behind)) or int(limit) < 2:
        raise ValueError('window lengths must be finite and nonnegative; limit must be at least 2')
    points = points[:, :2]
    nearest = int(np.argmin(np.sum((points - centre) ** 2, axis=1)))
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    first, covered = nearest, 0.
    while first > 0 and covered < behind:
        covered += steps[first - 1]
        first -= 1
    last, covered = nearest, 0.
    while last < len(points) - 1 and covered < ahead and last - first + 1 < int(limit):
        covered += steps[last]
        last += 1
    return points[first:last + 1].copy()


# 将轨迹快照转为可序列化值；None 显式编码为空点集。
def trajectory_to_values(trajectory):
    if trajectory is None:
        return dict(points=[], stamp=0., frame_id='', goal=None)
    return dict(points=trajectory.points.tolist(), stamp=trajectory.stamp,
                frame_id=trajectory.frame_id, goal=trajectory.goal,
                pass_through=trajectory.goal is None, route=trajectory.route.tolist())


# 中间航点显式取消终点停车资格；普通单目标仍要求最终目标。航线折线可缺省。
def trajectory_from_values(points, stamp, frame_id, goal, pass_through=False, route=None):
    if not len(points):
        return None
    if not frame_id or (goal is None and not pass_through):
        raise ValueError('A tracking reference requires a frame and final goal')
    return Trajectory(points, stamp, frame_id, None if pass_through else goal, route)


# 按 POINT_FIELDS 固定顺序读取消息，曲率字段 kappa 对应数组第六列；航线的 x/y 两个数组必须等长。
def trajectory_from_message(message):
    route_x, route_y = getattr(message, 'route_x', ()), getattr(message, 'route_y', ())
    if len(route_x) != len(route_y):
        raise ValueError('route_x and route_y must have the same length')
    return trajectory_from_values(
        [[getattr(point, name) for name in POINT_FIELDS] for point in message.points],
        message.trajectory_stamp, message.trajectory_frame, (message.goal.x, message.goal.y),
        pass_through=getattr(message, 'pass_through', False), route=list(zip(route_x, route_y)))


# 填充轨迹源时间、坐标系、点序列和航线折线；header 的重传时间不代替 trajectory_stamp。
def set_message_trajectory(message, trajectory):
    from usv_navigation_interfaces.msg import TrajectoryPoint
    values = trajectory_to_values(trajectory)
    message.trajectory_stamp, message.trajectory_frame = values['stamp'], values['frame_id']
    message.points = [TrajectoryPoint(**dict(zip(POINT_FIELDS, row))) for row in values['points']]
    route = values.get('route', [])
    message.route_x, message.route_y = [float(x) for x, _ in route], [float(y) for _, y in route]
