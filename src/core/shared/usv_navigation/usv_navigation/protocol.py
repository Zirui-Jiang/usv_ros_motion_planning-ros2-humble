# 分布式导航轨迹编解码与心跳接口转出，保留原始轨迹时间、坐标系、八列数值和最终目标。
"""ROS-independent session bookkeeping and lossless trajectory conversion."""
from trajectory_planner.trajectory import Trajectory
from .heartbeat import PeerSession, HEARTBEAT_PERIOD, PEER_TIMEOUT

POINT_FIELDS = ('time', 's', 'x', 'y', 'theta', 'kappa', 'velocity', 'acceleration')


# 将轨迹快照转为可序列化值；None 显式编码为空点集。
def trajectory_to_values(trajectory):
    if trajectory is None:
        return dict(points=[], stamp=0., frame_id='', goal=None)
    return dict(points=trajectory.points.tolist(), stamp=trajectory.stamp,
                frame_id=trajectory.frame_id, goal=trajectory.goal,
                pass_through=trajectory.goal is None)


# 中间航点显式取消终点停车资格；普通单目标仍要求最终目标。
def trajectory_from_values(points, stamp, frame_id, goal, pass_through=False):
    if not len(points):
        return None
    if not frame_id or (goal is None and not pass_through):
        raise ValueError('A tracking reference requires a frame and final goal')
    return Trajectory(points, stamp, frame_id, None if pass_through else goal)


# 按 POINT_FIELDS 固定顺序读取消息，曲率字段 kappa 对应数组第六列。
def trajectory_from_message(message):
    return trajectory_from_values(
        [[getattr(point, name) for name in POINT_FIELDS] for point in message.points],
        message.trajectory_stamp, message.trajectory_frame, (message.goal.x, message.goal.y),
        pass_through=getattr(message, 'pass_through', False))


# 填充轨迹源时间、坐标系与点序列；header 的重传时间不代替 trajectory_stamp。
def set_message_trajectory(message, trajectory):
    from usv_navigation_interfaces.msg import TrajectoryPoint
    values = trajectory_to_values(trajectory)
    message.trajectory_stamp, message.trajectory_frame = values['stamp'], values['frame_id']
    message.points = [TrajectoryPoint(**dict(zip(POINT_FIELDS, row))) for row in values['points']]
