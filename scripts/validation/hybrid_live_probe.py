"""Read-only capture of Hybrid planning, control and a replayable map/state."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from nav2_msgs.msg import Costmap
from std_msgs.msg import String, Float32
from tf2_ros import Buffer, TransformListener
from usv_navigation_interfaces.msg import NavigationReference, ControlReport


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(s):
    return s.sec + s.nanosec*1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = rclpy.create_node('hybrid_readonly_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest, events, counts = {}, [], Counter()
    started = time.monotonic()

    def receive(key, msg):
        latest[key] = msg
        counts[key] += 1
        row = dict(topic=key, wall=time.monotonic()-started)
        if key == 'status':
            row.update(json.loads(msg.data))
        elif key == 'reference':
            row.update(state=msg.state, reason=msg.reason, points=len(msg.points),
                       sequence=msg.sequence, task=msg.task_id, source_stamp=msg.trajectory_stamp,
                       published_stamp=stamp(msg.header.stamp), active=msg.active)
        elif key == 'control':
            row.update(state=msg.state, reason=msg.reason, sequence=msg.reference_sequence,
                       command=[msg.left, msg.right, msg.lateral], revoke=msg.revoke_reference)
        elif key in ('global_path', 'local_trajectory'):
            xy = np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses], dtype=float)
            row.update(points=len(xy), stamp=stamp(msg.header.stamp),
                       length=float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy)>1 else 0.,
                       hash=hashlib.sha256(xy.tobytes()).hexdigest()[:12])
            if len(xy) and key == 'global_path':
                np.save(args.output/'last_global_path.npy', np.array([
                    [p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses]))
        elif key == 'odom':
            pose, vel = msg.pose.pose, msg.twist.twist
            row.update(x=pose.position.x, y=pose.position.y, yaw=yaw(pose.orientation),
                       speed=math.hypot(vel.linear.x,vel.linear.y), r=vel.angular.z,
                       stamp=stamp(msg.header.stamp), frame=msg.header.frame_id)
        elif key in ('left','right'):
            row['value'] = float(msg.data)
        elif key == 'goal':
            row['goal']=[msg.pose.position.x,msg.pose.position.y,yaw(msg.pose.orientation)]
        else:
            return
        events.append(row)

    reliable = QoSProfile(depth=10)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(kind, topic, lambda m,k=key:receive(k,m), qos)
                     for key,kind,topic,qos in [
        ('status',String,'/otter/navigation_status',reliable),
        ('reference',NavigationReference,'/otter/navigation/reference',reliable),
        ('control',ControlReport,'/otter/navigation/control_report',reliable),
        ('global_path',RosPath,'/otter/global_path',retained),
        ('local_trajectory',RosPath,'/otter/local_trajectory',reliable),
        ('odom',Odometry,'/otter/odom',sensor),
        ('map',Costmap,'/otter/costmap_raw',retained),
        ('goal',PoseStamped,'/otter/goal',reliable),
        ('left',Float32,'/otter/thrusters/left_thrust_cmd',reliable),
        ('right',Float32,'/otter/thrusters/right_thrust_cmd',reliable)]]
    snapshot = {}
    try:
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
        if 'reference' in latest:
            msg=latest['reference']
            snapshot.update(parameters=json.loads(msg.parameters_json),
                            goal=[msg.goal.x,msg.goal.y,msg.goal.theta], version=msg.revision,
                            frame_id=msg.header.frame_id)
        if 'map' in latest:
            msg=latest['map']; meta=msg.metadata
            data=np.array(msg.data,np.uint8).reshape(meta.size_y,meta.size_x)
            np.savez_compressed(args.output/'map.npz',data=data,resolution=meta.resolution,
                                origin_x=meta.origin.position.x,origin_y=meta.origin.position.y,
                                stamp=stamp(msg.header.stamp),frame_id=msg.header.frame_id)
            snapshot['map_info']=dict(shape=list(data.shape),resolution=meta.resolution,
                origin=[meta.origin.position.x,meta.origin.position.y],stamp=stamp(msg.header.stamp))
        if 'odom' in latest:
            msg=latest['odom']; p=msg.pose.pose; v=msg.twist.twist
            x,y,heading=p.position.x,p.position.y,yaw(p.orientation)
            frame=snapshot.get('frame_id','otter/map')
            if msg.header.frame_id != frame:
                transform=buffer.lookup_transform(frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
                angle=yaw(transform.transform.rotation)
                x,y=(math.cos(angle)*x-math.sin(angle)*y+transform.transform.translation.x,
                     math.sin(angle)*x+math.cos(angle)*y+transform.transform.translation.y)
                heading=math.atan2(math.sin(heading+angle),math.cos(heading+angle))
            snapshot['state']=dict(x=x,y=y,psi=heading,u=v.linear.x,v=v.linear.y,r=v.angular.z,
                                   stamp=stamp(msg.header.stamp),frame_id=frame)
        snapshot['publishers']={topic:[dict(name=e.node_name,namespace=e.node_namespace)
            for e in node.get_publishers_info_by_topic(topic)] for topic in
            ('/otter/global_path','/otter/odom','/otter/thrusters/left_thrust_cmd')}
    finally:
        (args.output/'events.json').write_text(json.dumps(events,ensure_ascii=False,indent=2),encoding='utf-8')
        (args.output/'snapshot.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2),encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    summary=dict(counts=dict(counts),
                 reference_states=dict(Counter((r['state']+': '+r['reason']) for r in events if r['topic']=='reference')),
                 control_states=dict(Counter((r['state']+': '+r['reason']) for r in events if r['topic']=='control')),
                 path_sizes=[r['points'] for r in events if r['topic']=='global_path'],
                 nonzero_commands=sum(any(abs(c)>1e-6 for c in r['command']) for r in events if r['topic']=='control'),
                 speed_range=[min((r['speed'] for r in events if r['topic']=='odom'),default=0.),
                              max((r['speed'] for r in events if r['topic']=='odom'),default=0.)],
                 snapshot=snapshot)
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='snapshot'},ensure_ascii=False))


if __name__=='__main__':
    main()
