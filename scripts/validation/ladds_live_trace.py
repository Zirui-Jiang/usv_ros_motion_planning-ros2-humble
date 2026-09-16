"""Read-only task/goal/odometry trace for live LaDDS troubleshooting."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=180)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('ladds_live_trace')
    started = time.monotonic()
    last_odom = float('-inf')
    with output.open('w', encoding='utf-8') as log:
        def record(kind, data):
            log.write(json.dumps(dict(elapsed=time.monotonic()-started, kind=kind,
                                      data=data), ensure_ascii=False) + '\n')
            log.flush()

        def text(kind, message):
            try:
                data = json.loads(message.data)
            except ValueError:
                data = {'invalid_json': message.data}
            if kind == 'navigation' and isinstance(data, dict):
                data = {k: v for k, v in data.items()
                        if k not in ('metrics', 'counters', 'process')}
            record(kind, data)

        def odom(message):
            nonlocal last_odom
            now = time.monotonic()
            if now-last_odom < .5:
                return
            last_odom = now
            p, v = message.pose.pose.position, message.twist.twist.linear
            record('odom', dict(x=p.x, y=p.y, u=v.x, v=v.y))

        subscriptions = [
            node.create_subscription(String, topic, lambda m, k=kind: text(k, m), 20)
            for kind, topic in [('rx', '/ladds/rx'), ('task', '/ladds/task_status'),
                                ('tx', '/ladds/tx'), ('navigation', '/otter/navigation_status')]
        ]
        subscriptions += [
            node.create_subscription(PoseStamped, '/otter/goal', lambda m: record(
                'ordinary_goal', dict(frame=m.header.frame_id, x=m.pose.position.x,
                                      y=m.pose.position.y)), 10),
            node.create_subscription(Odometry, '/otter/odom', odom,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        ]
        try:
            while time.monotonic()-started < args.duration:
                rclpy.spin_once(node, timeout_sec=.1)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(str(output))


if __name__ == '__main__':
    main()
