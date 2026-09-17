#!/usr/bin/env python3
"""Read real node parameters after loading relocated project YAML files."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=193)
    args = parser.parse_args()
    if not args.output.is_absolute() or args.output.exists():
        parser.error('--output must be an absolute new directory')
    if not 1 <= args.domain_id <= 232:
        parser.error('invalid ROS domain')
    args.output.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from std_srvs.srv import Trigger
    from usv_navigation_interfaces.msg import ModuleHeartbeat
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('usv_navigation'))
    configs = {
        'dynamics_config': share / 'config/robots/otter/dynamics_otter.yaml',
        'controller_config': share / 'config/robots/otter/controller_params_otter.yaml',
        'trajectory_config': share / 'config/trajectory_planner/trajectory_planner_params.yaml',
        'planner_config': share / 'config/path_planner/common_planner_params.yaml',
    }
    paths = {}
    for key, source in configs.items():
        values = yaml.safe_load(source.read_text())
        if key == 'dynamics_config':
            values['dynamics']['mass'] = 31.25
        elif key == 'controller_config':
            values['controller'].update(controller_name='lqr', control_frequncy=8.0)
            values['controller']['pid_controller']['k_p_speed'] = 3.25
        elif key == 'trajectory_config':
            values['trajectory_planner']['los']['yaw_rate_max'] = .23
        else:
            values['planner_name'] = 'dijkstra'
            values['path_planner']['obstacle_factor'] = .35
        path = args.output / (key + '.yaml')
        path.write_text(yaml.safe_dump(values, sort_keys=False))
        paths[key] = path
    C, T = 'usv_simulation.controller.', 'usv_simulation.trajectory_planner.'
    expected = {
        'dynamics.mass': 31.25, 'dynamics.ns': 'yaml_probe', 'planner_name': 'dijkstra',
        'obstacle_factor': .35, C + 'controller_name': 'lqr', C + 'control_frequency': 8.,
        C + 'pid_controller.k_p_speed': 3.25, T + 'los.yaw_rate_max': .23,
    }
    result = dict(passed=False, domain_id=args.domain_id, installed_share=str(share),
                  config_paths={key: str(value) for key, value in paths.items()}, expected=expected,
                  commands=[], checks={})
    processes, logs = [], []
    node = None
    try:
        rclpy.init()
        node = Node('distributed_yaml_probe')
        pub = node.create_publisher(ModuleHeartbeat, '/yaml_probe/navigation/heartbeat', 10)
        sequence = 0
        def heartbeat():
            nonlocal sequence
            sequence += 1
            pub.publish(ModuleHeartbeat(module='perception', session='yaml-probe-perception',
                                        sequence=sequence, ready=True))
        node.create_timer(.1, heartbeat)
        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
        def call(name, srv_type, request, timeout=15.):
            client = node.create_client(srv_type, name)
            try:
                deadline = time.monotonic() + timeout
                while not client.service_is_ready() and time.monotonic() < deadline:
                    spin(.05)
                if not client.service_is_ready():
                    raise RuntimeError('Service unavailable: ' + name)
                future = client.call_async(request)
                while not future.done() and time.monotonic() < deadline:
                    spin(.03)
                if not future.done():
                    raise RuntimeError('Service timed out: ' + name)
                return future.result()
            finally:
                node.destroy_client(client)
        for module in ('decision', 'control'):
            command = ['ros2', 'launch', 'usv_' + module, module + '.launch.py',
                       'robot_namespace:=yaml_probe', 'use_sim_time:=false',
                       *[key + ':=' + str(value) for key, value in paths.items()]]
            stream = (args.output / (module + '.log')).open('w')
            logs.append(stream)
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            processes.append(process)
            result['commands'].append(dict(argv=command, launcher_pid=process.pid))
        for name in ('usv_simulation', 'usv_control'):
            response = call('/yaml_probe/' + name + '/get_parameters', GetParameters,
                            GetParameters.Request(names=list(expected)))
            actual = dict(zip(expected, map(parameter_value_to_python, response.values)))
            result['checks'][name + '_startup_yaml'] = dict(passed=actual == expected, actual=actual)
            assert actual == expected, (name, actual)
        response = call('/yaml_probe/usv_simulation/set_parameters_atomically', SetParametersAtomically,
                        SetParametersAtomically.Request(parameters=[
                            Parameter(C + 'pid_controller.k_p_speed', value=4.75).to_parameter_msg()]))
        assert response.result.successful, response.result.reason
        deadline = time.monotonic() + 8.
        remote_value = None
        while time.monotonic() < deadline:
            response = call('/yaml_probe/usv_control/get_parameters', GetParameters,
                            GetParameters.Request(names=[C + 'pid_controller.k_p_speed']))
            remote_value = parameter_value_to_python(response.values[0])
            if remote_value == 4.75:
                break
            spin(.1)
        assert remote_value == 4.75, 'Hot parameter did not reach actual control process'
        result['checks']['hot_update_actual_control'] = dict(passed=True, actual=remote_value)
        response = call('/yaml_probe/export_parameters', Trigger, Trigger.Request())
        exported = yaml.safe_load(response.message)['/yaml_probe/usv_simulation']['ros__parameters']
        assert exported[C + 'pid_controller.k_p_speed'] == 4.75
        assert exported[C + 'controller_name'] == 'lqr'
        (args.output / 'exported_parameters.yaml').write_text(response.message)
        result['checks']['export_compatibility'] = dict(passed=response.success)
        result['passed'] = all(check['passed'] for check in result['checks'].values())
    except Exception as error:
        result.update(failure=str(error), traceback=traceback.format_exc())
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
        for stream in logs:
            stream.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False))
        print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
