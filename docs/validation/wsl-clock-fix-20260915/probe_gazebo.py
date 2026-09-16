"""Bounded real Gazebo probe; only launch/stop the process group owned here."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=150.)
    parser.add_argument('--domain', type=int, default=197)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    repo = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(repo / 'scripts/validation'))
    from ros_acceptance import OwnedProcess, free_port
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
    from geometry_msgs.msg import PoseStamped
    from usv_navigation_interfaces.msg import ControlReport

    rclpy.init()
    node = rclpy.create_node('wsl_clock_validation')
    topics = {'clock': (Clock, '/clock'), 'odom': (Odometry, '/otter/odom'),
              'scan': (LaserScan, '/otter/sensors/lidars/lidar/scan'),
              'left': (Float32, '/otter/thrusters/left_thrust_cmd'),
              'report': (ControlReport, '/otter/navigation/control_report')}
    rows = {name: [] for name in topics}
    def receive(name, msg):
        source = msg.clock if name == 'clock' else msg.header.stamp if name in ('odom', 'scan') else None
        source_s = source.sec + source.nanosec * 1e-9 if source else None
        value = msg.data if name == 'left' else None
        rows[name].append([time.clock_gettime(time.CLOCK_MONOTONIC_RAW), source_s, value])
    subscriptions = [node.create_subscription(kind, topic, lambda msg, n=name: receive(n, msg),
                       qos_profile_sensor_data) for name, (kind, topic) in topics.items()]
    goal_pub = node.create_publisher(PoseStamped, '/otter/goal', 10)
    config = repo / 'src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml'
    command = ['ros2', 'launch', 'sim_env', 'main.launch.py', 'world:=ocean', 'gui:=false',
               'rviz:=false', 'controller_name:=adrc', 'robot_namespace:=otter', 'robot_x:=23.0',
               'robot_y:=-10.0', 'robot_yaw:=0.0', 'controller_config:=' + str(config)]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{free_port()}')
    process = None
    result = dict(command=command, domain=args.domain, gazebo_master=env['GAZEBO_MASTER_URI'],
                  config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(), passed=False)
    try:
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if node.count_publishers('/otter/odom'):
            raise RuntimeError('Chosen domain has existing odom; refusing to start another simulation')
        process = OwnedProcess(args.workspace, command, args.output / 'launch.log', env)
        deadline = time.monotonic() + 90.
        # This probe does not subscribe to its own goal, unlike ros_acceptance.
        while not all(rows.values()) or goal_pub.get_subscription_count() < 1:
            process.check()
            if time.monotonic() >= deadline:
                raise RuntimeError('Topics or goal subscribers did not become ready')
            rclpy.spin_once(node, timeout_sec=.02)
        for values in rows.values():
            values.clear()
        start = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        next_progress = 10.
        goals = [(0., 43., -10.), (40., 43., 10.), (80., 23., 10.), (120., 23., -10.)]
        sent = []
        while True:
            process.check()
            elapsed = time.clock_gettime(time.CLOCK_MONOTONIC_RAW) - start
            if elapsed >= args.duration:
                break
            if goals and elapsed >= goals[0][0]:
                _, x, y = goals.pop(0)
                msg = PoseStamped()
                msg.header.frame_id = 'otter/map'
                msg.header.stamp.sec = int(rows['clock'][-1][1]) if rows['clock'] else 0
                msg.header.stamp.nanosec = int((rows['clock'][-1][1] % 1.) * 1e9) if rows['clock'] else 0
                msg.pose.position.x, msg.pose.position.y = x, y
                msg.pose.orientation.w = 1.
                goal_pub.publish(msg)
                sent.append(dict(at_raw_s=elapsed, x=x, y=y))
            rclpy.spin_once(node, timeout_sec=.01)
            if elapsed >= next_progress:
                print('PROGRESS', round(elapsed, 1), {k: len(v) for k, v in rows.items()}, flush=True)
                next_progress += 10.
        end = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        stats = {}
        for name, values in rows.items():
            gaps = [b[0]-a[0] for a, b in zip(values, values[1:])]
            edge_gaps = [values[0][0]-start, end-values[-1][0]] if values else [end-start]
            stats[name] = dict(count=len(values), max_gap_s=max(gaps + edge_gaps),
                               gaps_over_half_second=sum(g > .5 for g in gaps + edge_gaps))
        clock = rows['clock']
        result.update(duration_raw_s=end-start, goals=sent, topics=stats,
                      realtime_factor=(clock[-1][1]-clock[0][1])/(clock[-1][0]-clock[0][0]),
                      nonzero_left_samples=sum(abs(v[2]) > .01 for v in rows['left']),
                      passed=all(s['count'] > 100 and s['max_gap_s'] < .5 for s in stats.values()))
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        (args.output / 'result.json').write_text(json.dumps(dict(result=result, rows=rows), indent=2))
        if process is not None:
            process.close()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
