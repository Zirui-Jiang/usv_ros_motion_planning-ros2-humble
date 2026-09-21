"""Bounded isolated Gazebo validation from the ROS side of the LaDDS bridge.

Uses ROS domain 88 and its own Gazebo master. Publishes exactly one 0211 JSON
to the adapter's ROS input; no Windows DDS publisher or relay is contacted.
Starts and cleans up only its own simulation process group.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=210.)
    parser.add_argument('--shape', choices=('turn', 'straight'), default='turn')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Require ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 for this isolated check')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 11359))  # Refuse to reuse an occupied Gazebo master.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from common.geodesy import GeoReference
    from usv_navigation_interfaces.msg import NavigationReference, ControlReport
    from usv_ladds_bridge.route_node import RouteNode

    root = Path(__file__).resolve().parents[2]
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(success=False, sent_count=0, feedback=[], passages=[], trace=[],
                  navigation_changes=[], active_task_ids=[], intermediate_stops=[])
    latest, navigation, targets = {}, {}, []
    origin = GeoReference(30.5453, 114.392931)
    begun, sent_at, finished_at = time.monotonic(), None, None
    observed_index = 0
    rclpy.init()
    node, adapter = Node('multi_route_gazebo_observer'), RouteNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(adapter)
    publisher = node.create_publisher(String, '/ladds/rx', 10)

    def odometry(msg):
        nonlocal observed_index
        position, twist = msg.pose.pose.position, msg.twist.twist
        latest.update(x=position.x, y=position.y,
                      speed=math.hypot(twist.linear.x, twist.linear.y))
        if sent_at is not None:
            while observed_index < len(targets) - 1:
                target = targets[observed_index]
                distance = math.hypot(position.x-target[0], position.y-target[1])
                if distance > 1.5:
                    break
                result['passages'].append(dict(index=observed_index, distance=distance,
                    speed=latest['speed'], elapsed=time.monotonic()-sent_at))
                observed_index += 1
            if not result['trace'] or time.monotonic()-begun-result['trace'][-1]['elapsed'] >= .1:
                result['trace'].append(dict(elapsed=time.monotonic()-begun,
                    waypoint_index=navigation.get('waypoint_index'), **latest))

    def status(msg):
        navigation.clear()
        navigation.update(json.loads(msg.data))
        key = (navigation.get('state'), navigation.get('waypoint_index'))
        if not result['navigation_changes'] or key != tuple(result['navigation_changes'][-1]['key']):
            result['navigation_changes'].append(dict(key=key, elapsed=time.monotonic()-begun,
                reason=navigation.get('reason'), request_id=navigation.get('request_id'),
                task_id=navigation.get('task_id'), passed_waypoints=navigation.get('passed_waypoints')))
            print('NAV', key, navigation.get('reason', ''), flush=True)

    def reference(msg):
        if sent_at is not None and msg.active:
            if msg.task_id not in result['active_task_ids']:
                result['active_task_ids'].append(msg.task_id)
            result['trajectory_horizon'] = max((p.time for p in msg.points), default=0.)

    def report(msg):
        if sent_at is not None and observed_index < len(targets)-1:
            if msg.task_finished or msg.state in ('SETTLING', 'GOAL_REACHED'):
                result['intermediate_stops'].append(dict(state=msg.state, task_id=msg.task_id))

    def feedback(msg):
        value = json.loads(msg.data)
        result['feedback'].append(value)

    subscriptions = [
        node.create_subscription(Odometry, '/otter/odom', odometry,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(String, '/otter/navigation_status', status, 10),
        node.create_subscription(NavigationReference, '/otter/navigation/reference', reference, 10),
        node.create_subscription(ControlReport, '/otter/navigation/control_report', report, 10),
        node.create_subscription(String, '/ladds/tx', feedback, 10)]
    environment = dict(os.environ, GAZEBO_MASTER_URI='http://127.0.0.1:11359',
        LADDS_SINGLE_LEG_INSTALL='/tmp/usv_ladds_multi_waypoint_20260915/install')
    log = output.with_suffix('.log').open('w', encoding='utf-8')
    process = subprocess.Popen(['bash', str(root/'scripts/start_ladds_single_leg_sim.bash'),
        '--headless', 'true'], cwd=root, env=environment, stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    try:
        while time.monotonic()-begun < args.duration:
            executor.spin_once(timeout_sec=.05)
            if process.poll() is not None:
                result['error'] = 'Simulation exited before validation completed'
                break
            if sent_at is None and latest and adapter.client.service_is_ready() and navigation.get('state') == 'IDLE':
                x, y = latest['x'], latest['y']
                targets.extend([(x+12., y), (x+12., y+12.), (x+24., y+12.)]
                               if args.shape == 'turn' else [(x+4., y), (x+8., y), (x+12., y)])
                points = []
                for (px, py), speed in zip(targets, (2, 4, 2)):
                    lat, lon, _ = origin.to_geographic(px, py)
                    points.append(dict(type=1, latitude=lat, longitude=lon, speed=speed))
                task = dict(travelId=88, taskType=16, count=3, points=points,
                            areasCount=0, areas=[])
                result.update(targets=targets, task=task, initial=dict(latest))
                sent_at = time.monotonic()
                publisher.publish(String(data=json.dumps(task)))
                result['sent_count'] += 1
                print('PUBLISHED ONE 0211', json.dumps(task), flush=True)
            if result['feedback'] and finished_at is None:
                finished_at = time.monotonic()
            if finished_at is not None and time.monotonic()-finished_at >= 2.:
                break
        result['final'] = dict(latest)
        if targets and latest:
            result['final_error'] = math.hypot(latest['x']-targets[-1][0], latest['y']-targets[-1][1])
        result['final_navigation'] = navigation
        result['success'] = (result['sent_count'] == 1 and len(result['active_task_ids']) == 1
            and result['feedback'] == [dict(travelId=88, taskFinish=1)]
            and len(result['passages']) == 2 and not result['intermediate_stops']
            and result.get('final_error', math.inf) <= 1.5 and latest.get('speed', math.inf) <= .25
            and navigation.get('passed_waypoints') == 3)
    except Exception as error:
        result['error'] = repr(error)
    finally:
        result['elapsed'] = time.monotonic()-begun
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('trace', 'final_navigation')}, ensure_ascii=True), flush=True)
        adapter.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 8), (signal.SIGKILL, 3)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=wait)
                # The launcher can exit before a descendant; probe the owned group.
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        log.close()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
