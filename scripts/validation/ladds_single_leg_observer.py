"""Observe real navigation and Gazebo odometry; never publish boat commands."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from usv_navigation_interfaces.msg import NavigationReference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=240)
    parser.add_argument('--travel-id', type=int, default=53)
    args = parser.parse_args()
    rclpy.init()
    node = Node('ladds_single_leg_observer')
    begun = time.monotonic()
    result = dict(travelId=args.travel_id, received_count=0, completed=False,
                  navigation_request_ids=[], max_reference_speed=0.,
                  max_observed_speed=0., status_changes=[], trace=[])
    target = (83.38367674866956, 91.682939134985)
    latest = {}

    def received(msg):
        data = json.loads(msg.data)
        if data.get('content', data).get('travelId') == args.travel_id:
            result['received_count'] += 1

    def task_status(msg):
        data = json.loads(msg.data)
        if data.get('travelId') != args.travel_id:
            return
        state = data.get('state')
        if not result['status_changes'] or result['status_changes'][-1]['state'] != state:
            result['status_changes'].append(dict(elapsed=time.monotonic() - begun, **data))
        request = data.get('request_id')
        if request and request not in result['navigation_request_ids']:
            result['navigation_request_ids'].append(request)
        if state == 'COMPLETED':
            result['completed'] = True

    def reference(msg):
        if msg.active and math.hypot(msg.goal.x - target[0], msg.goal.y - target[1]) < .01:
            result['max_reference_speed'] = max(
                result['max_reference_speed'], max((p.velocity for p in msg.points), default=0.))

    def odometry(msg):
        position = msg.pose.pose.position
        twist = msg.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        latest.update(x=position.x, y=position.y, speed=speed,
                      error=math.hypot(position.x - target[0], position.y - target[1]))
        result['max_observed_speed'] = max(result['max_observed_speed'], speed)
        if not result['trace'] or time.monotonic() - begun - result['trace'][-1]['elapsed'] >= .5:
            result['trace'].append(dict(elapsed=time.monotonic() - begun, **latest))

    subscriptions = [
        node.create_subscription(String, '/ladds/rx', received, 10),
        node.create_subscription(String, '/ladds/task_status', task_status, 10),
        node.create_subscription(NavigationReference, '/otter/navigation/reference', reference, 10),
        node.create_subscription(Odometry, '/otter/odom', odometry,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
    ]
    try:
        while time.monotonic() - begun < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if result['completed'] and latest.get('speed', float('inf')) < .25:
                break
    except KeyboardInterrupt:
        result['interrupted'] = True
    finally:
        result['final'] = latest
        result['elapsed'] = time.monotonic() - begun
        result['success'] = (result['completed'] and result['received_count'] == 1
                             and len(result['navigation_request_ids']) == 1
                             and abs(result['max_reference_speed'] - 4 * 1852 / 3600) < 1e-6
                             and latest.get('error', float('inf')) <= 1.5
                             and latest.get('speed', float('inf')) < .25)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({k: v for k, v in result.items() if k != 'trace'}, ensure_ascii=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
