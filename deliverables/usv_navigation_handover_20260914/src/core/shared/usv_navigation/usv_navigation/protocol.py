"""ROS-independent session bookkeeping and lossless trajectory conversion."""
from trajectory_planner.trajectory import Trajectory
from .heartbeat import PeerSession, HEARTBEAT_PERIOD, PEER_TIMEOUT

POINT_FIELDS = ('time', 's', 'x', 'y', 'theta', 'kappa', 'velocity', 'acceleration')


def trajectory_to_values(trajectory):
    if trajectory is None:
        return dict(points=[], stamp=0., frame_id='', goal=None)
    return dict(points=trajectory.points.tolist(), stamp=trajectory.stamp,
                frame_id=trajectory.frame_id, goal=trajectory.goal)


def trajectory_from_values(points, stamp, frame_id, goal):
    if not len(points):
        return None
    if not frame_id or goal is None:
        raise ValueError('A tracking reference requires a frame and final goal')
    return Trajectory(points, stamp, frame_id, goal)


def trajectory_from_message(message):
    return trajectory_from_values(
        [[getattr(point, name) for name in POINT_FIELDS] for point in message.points],
        message.trajectory_stamp, message.trajectory_frame, (message.goal.x, message.goal.y))


def set_message_trajectory(message, trajectory):
    from usv_navigation_interfaces.msg import TrajectoryPoint
    values = trajectory_to_values(trajectory)
    message.trajectory_stamp, message.trajectory_frame = values['stamp'], values['frame_id']
    message.points = [TrajectoryPoint(**dict(zip(POINT_FIELDS, row))) for row in values['points']]
