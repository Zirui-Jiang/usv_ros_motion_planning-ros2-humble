"""ROS visualization conversions, imported only by ROS adapters."""
import math


def make_path(points, frame_id, stamp):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    result = Path()
    result.header.frame_id = frame_id
    result.header.stamp = stamp
    for point in points:
        pose = PoseStamped()
        pose.header = result.header
        pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
        yaw = float(point[2]) if len(point) > 2 else 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        result.poses.append(pose)
    return result
