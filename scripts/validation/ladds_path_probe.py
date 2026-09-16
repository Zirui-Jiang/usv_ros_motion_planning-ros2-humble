"""Read-only snapshot of DDS input, displayed paths and navigation reference."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from usv_navigation_interfaces.msg import NavigationReference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=6)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('ladds_path_probe')
    latest = {}
    counts = {}

    def receive(key, message):
        counts[key] = counts.get(key, 0) + 1
        latest[key] = message_to_ordereddict(message)

    subscriptions = [node.create_subscription(kind, topic,
        lambda m, k=key: receive(k, m), qos) for key, kind, topic, qos in [
            ('rx', String, '/ladds/rx', 10),
            ('task', String, '/ladds/task_status', 10),
            ('navigation', String, '/otter/navigation_status', 10),
            ('global_path', RosPath, '/otter/global_path', 10),
            ('local_trajectory', RosPath, '/otter/local_trajectory', 10),
            ('reference', NavigationReference, '/otter/navigation/reference', 10),
            ('odom', Odometry, '/otter/odom', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        ]]
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    summary = {'counts': counts}
    for key in ('rx', 'task', 'navigation'):
        try:
            summary[key] = json.loads(latest[key]['data'])
        except (KeyError, ValueError):
            pass
    for key in ('global_path', 'local_trajectory'):
        data = latest.get(key, {})
        points = [p['pose']['position'] for p in data.get('poses', [])]
        summary[key] = dict(
            received=key in latest, header=data.get('header'),
            points=len(points) if key in latest else None,
            length_m=(sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                          for a, b in zip(points, points[1:])) if key in latest else None),
            start=points[0] if points else None, end=points[-1] if points else None)
    reference = latest.get('reference', {})
    points = reference.get('points', [])
    summary['reference'] = {k: reference.get(k) for k in (
        'active', 'state', 'reason', 'goal', 'task_id', 'trajectory_stamp')}
    summary['reference'].update(
        received='reference' in latest,
        point_count=len(points) if 'reference' in latest else None,
        horizon_s=points[-1]['time'] if points else None,
        max_speed_mps=max((p['velocity'] for p in points), default=0.))
    summary['odom'] = latest.get('odom', {}).get('pose', {}).get('pose')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(summary=summary, messages=latest),
                                 ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
