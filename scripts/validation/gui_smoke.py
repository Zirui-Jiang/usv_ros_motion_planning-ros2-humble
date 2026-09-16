#!/usr/bin/env python3
# 短时启动图形界面并保存候选证据；不下发目标，截图是否显示正确仍需人工检查。
"""Short, goal-free GUI smoke run, including an interactive (Conda) shell.

Source ROS and the target install before running with the system Python.
Screenshots are evidence candidates; inspect them before claiming visual success.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('mode', choices=['script', 'direct'])
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=30.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.mode == 'script':
        launch = 'bash scripts/main.sh'
    else:
        launch = ('source /opt/ros/humble/setup.bash; source install/setup.bash; '
                  'ros2 launch sim_env main.launch.py gui:=true rviz:=true')
    command = ['bash', '-ic', f'cd {shlex.quote(str(args.workspace))}; '
               f'command -v python3; {launch}']
    report = {'workspace': str(args.workspace), 'mode': args.mode, 'command': command,
              'environment': {name: os.environ.get(name) for name in ('ROS_DOMAIN_ID', 'GAZEBO_MASTER_URI', 'DISPLAY')},
              'models': [], 'odometry_messages': 0, 'scan_messages': 0, 'windows': []}
    rclpy.init()
    node = Node('otter_gui_smoke_observer')

    def models(message):
        report['models'] = list(message.name)
        if 'otter' in message.name:
            pose = message.pose[message.name.index('otter')].position
            report['otter_position'] = [pose.x, pose.y, pose.z]

    def count(name):
        def callback(_):
            report[name] += 1
        return callback

    subscriptions = [
        node.create_subscription(ModelStates, '/gazebo/model_states', models, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/otter/odom', count('odometry_messages'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/otter/sensors/lidars/lidar/scan', count('scan_messages'), qos_profile_sensor_data),
    ]
    qt = None
    with (args.output / 'launch.log').open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            started = time.monotonic()
            captured = False
            while process.poll() is None and time.monotonic() - started < args.duration:
                rclpy.spin_once(node, timeout_sec=.1)
                if not captured and time.monotonic() - started > 18:
                    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
                    (args.output / 'xwininfo.txt').write_text(tree)
                    from PyQt5.QtWidgets import QApplication
                    qt = QApplication([])
                    owned_pids = {int(value) for value in re.findall(
                        r'process started with pid \[(\d+)\]', (args.output / 'launch.log').read_text())}
                    for label, pattern in [('rviz', ' - RViz'), ('gazebo', '"Gazebo"')]:
                        matches = [line for line in tree.splitlines() if pattern in line]
                        for line in matches:
                            window = re.search(r'0x[0-9a-fA-F]+', line).group()
                            pid = subprocess.check_output(['xprop', '-id', window, '_NET_WM_PID'], text=True).strip()
                            match = re.search(r'= (\d+)', pid)
                            if not match or int(match.group(1)) not in owned_pids:
                                continue
                            pixmap = qt.primaryScreen().grabWindow(int(window, 16))
                            saved = pixmap.save(str(args.output / f'{label}-x11.png'))
                            report['windows'].append(dict(label=label, id=window, pid=pid, saved=saved))
                    captured = True
                    print('GUI_READY ' + str(args.output), flush=True)
            report['running_before_shutdown'] = process.poll() is None
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                report['exit_code'] = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    report['exit_code'] = process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    report['exit_code'] = process.wait()
            node.destroy_node()
            rclpy.shutdown()
    log = (args.output / 'launch.log').read_text()
    report['spawn_success'] = 'Successfully spawned entity [otter]' in log
    report['texture_errors'] = len(re.findall('Could not load texture|Can not find codec|Texture layer will be blank', log))
    report['python_import_errors'] = len(re.findall('ModuleNotFoundError|ImportError', log))
    report['premature_process_failures'] = [line for line in log.split('[WARNING] [launch]: user interrupted')[0].splitlines()
                                          if 'process has died' in line]
    report['passed'] = (report['running_before_shutdown'] and report['spawn_success']
                        and 'otter' in report['models'] and report['odometry_messages'] > 10
                        and report['scan_messages'] > 10 and not report['texture_errors']
                        and not report['python_import_errors'] and not report['premature_process_failures']
                        and {window['label'] for window in report['windows']} == {'rviz', 'gazebo'})
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
