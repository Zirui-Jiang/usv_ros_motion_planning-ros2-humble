#!/usr/bin/env python3
# 通过实际启动入口加载独立 YAML，读取节点参数服务，核对决策与航控采用的有效配置。
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


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
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
    # 读取安装区四份项目配置，只在新输出目录写修改副本，用辨识度较高的参数验证外部路径确实被加载。
    for key, source in configs.items():
        values = yaml.safe_load(source.read_text())
        if key == 'dynamics_config':
            values['dynamics']['mass'] = 31.25
        elif key == 'controller_config':
            # 保留历史 control_frequncy 拼写来验证规范化，同时改变算法选择和频率以区别于默认配置。
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
    # 预期键使用节点实际声明的扁平参数名，含命名空间覆盖与归一化后的 control_frequency。
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
        # 提供独立感知会话心跳，使真实决策/控制进程能够发现三方身份并同步参考中的参数快照。
        node.create_timer(.1, heartbeat)
        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
        # 服务发现与异步应答共用单调墙钟截止时间；等待时继续 spin，保证心跳和消息处理不中断。
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
        # 分别通过各自正式 launch 入口启动节点，四份外部 YAML 路径同时传入两端。
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
        # 调用两个实际进程的 GetParameters 服务读回值，不能仅凭启动日志或本地 YAML 推断配置生效。
        for name in ('usv_simulation', 'usv_control'):
            response = call('/yaml_probe/' + name + '/get_parameters', GetParameters,
                            GetParameters.Request(names=list(expected)))
            actual = dict(zip(expected, map(parameter_value_to_python, response.values)))
            result['checks'][name + '_startup_yaml'] = dict(passed=actual == expected, actual=actual)
            assert actual == expected, (name, actual)
        # 从决策端原子修改允许的 PID 热参数，随后单独轮询控制端，检验跨进程同步链路。
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
        # 导出服务应保留 ROS 参数快照格式，并同时反映热更新值与原先 YAML 选定的 LQR 名称。
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
        # 只处理本脚本以新会话创建的启动进程组，先发 SIGINT，超时后强制收尾并保存最终摘要。
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
