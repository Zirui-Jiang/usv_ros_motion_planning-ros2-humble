#!/usr/bin/env python3
# 给独立定位进程发送受控状态与时钟，检查畸形输入、坐标转换、时间回退及退出边界。
"""Isolated ROS input/clock/shutdown checks for one owned localization process.

Source the selected workspace first. Outputs are exclusive; no Gazebo is started.
The malformed probe documents an input-boundary defect, not a prior crash cause.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid


# 解析运行模式、场景、隔离域和输出选项，在启动进程前检查参数合法性。
def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--mode', choices=('malformed-probe', 'acceptance'), default='acceptance')
    parser.add_argument('--gdb', action='store_true', help='Only for the single malformed probe')
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        parser.error('Use a dedicated nonzero ROS domain in [1, 232]')
    if args.gdb and args.mode != 'malformed-probe':
        parser.error('--gdb is only supported for malformed-probe')
    return args


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main():
    args = arguments()
    binary = args.workspace.resolve() / 'install/sim_env/lib/sim_env/fake_localization_node'
    if not binary.is_file():
        raise SystemExit(f'Missing installed executable: {binary}')
    if args.gdb and not shutil.which('gdb'):
        raise SystemExit('gdb is unavailable')
    args.output.mkdir(parents=True, exist_ok=False)
    # 独立非零 DDS 域与仅本机通信隔离输入；输出目录必须新建，避免覆盖已有验收证据。
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Pose, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    namespace = 'localization_smoke_' + uuid.uuid4().hex[:12]
    command = [str(binary), '--ros-args', '-r', '__ns:=/' + namespace,
               '-p', 'use_sim_time:=true', '-p', 'model_name:=otter']
    if args.gdb:
        command = ['gdb', '--batch', '-ex', 'set pagination off', '-ex', 'set confirm off',
                   '-ex', 'run', '-ex', 'thread apply all bt', '--args'] + command
    log_path = args.output / 'node.log'
    receipt = {'schema_version': 1, 'mode': args.mode, 'domain_id': args.domain_id,
               'namespace': namespace, 'binary': str(binary),
               'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
               'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               'command': command, 'started_monotonic_s': time.monotonic(),
               'signals': [], 'checks': {}, 'gdb': args.gdb,
               'scope': 'Owned standalone node only; no Gazebo; input defect is not attribution of a prior shutdown crash.'}
    rclpy.init(args=[])
    driver = rclpy.create_node('driver', namespace=namespace)
    model_pub = driver.create_publisher(ModelStates, '/gazebo/model_states', 10)
    clock_pub = driver.create_publisher(Clock, '/clock', 10)
    odometry = []
    driver.create_subscription(Odometry, '/' + namespace + '/odom', odometry.append, 10)
    process = None
    identity = None
    # 读取 Linux 进程启动时刻，配合 PID 识别本次创建的子进程，防止 PID 复用后误发信号。
    def starttime(pid):
        try:
            return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
        except (OSError, ValueError, IndexError):
            return None

    def owned_signal(sig):
        if process is not None and process.poll() is None:
            if identity is None or starttime(process.pid) != identity:
                raise RuntimeError('Refusing signal: child process identity changed')
            receipt['signals'].append({'pid': process.pid, 'starttime_ticks': identity,
                                       'signal': signal.Signals(sig).name,
                                       'monotonic_s': time.monotonic()})
            process.send_signal(sig)

    # 观察窗口使用单调墙钟，仿真时钟暂停或倒退不会阻塞超时判定。
    def pump(duration, callback=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if callback:
                callback()
            rclpy.spin_once(driver, timeout_sec=0.01)

    def until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            if process.poll() is not None:
                raise RuntimeError(f'Node/debugger exited during {description}: {process.returncode}')
            rclpy.spin_once(driver, timeout_sec=0.02)
        if not predicate():
            raise RuntimeError('Timeout: ' + description)

    def clock(seconds):
        value = Clock()
        nanoseconds = round(seconds * 1e9)
        value.clock.sec, value.clock.nanosec = divmod(nanoseconds, 10**9)
        clock_pub.publish(value)

    # 构造航向为 +π/2 的船舶：世界速度 (3,4) 应转为船体系 (4,-3)，偏航角速度不变。
    def valid_message():
        value = ModelStates()
        value.name = ['otter']
        pose, twist = Pose(), Twist()
        pose.position.x, pose.position.y = 2.0, 3.0
        pose.orientation.z = math.sin(math.pi / 4)
        pose.orientation.w = math.cos(math.pi / 4)
        twist.linear.x, twist.linear.y, twist.angular.z = 3.0, 4.0, 0.2
        value.pose, value.twist = [pose], [twist]
        return value

    try:
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            identity = starttime(process.pid)
            receipt.update(owned_pid=process.pid, owned_starttime_ticks=identity)
            until(lambda: model_pub.get_subscription_count() > 0, 15, 'model-state subscription discovery')
            if model_pub.get_subscription_count() != 1:
                raise RuntimeError('Refusing shared domain: model-state subscriber count is not one')
            # 探针模式仅发一次 name 有元素但 pose/twist 为空的消息，记录是否存活，不推断其它历史崩溃原因。
            if args.mode == 'malformed-probe':
                message = ModelStates()
                message.name = ['otter']
                receipt['probe_input'] = {'name': ['otter'], 'pose_length': 0, 'twist_length': 0}
                receipt['probe_published_monotonic_s'] = time.monotonic()
                model_pub.publish(message)  # One malformed message; no repeated injections.
                deadline = time.monotonic() + 10
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(driver, timeout_sec=0.05)
                receipt['probe_process_returncode'] = process.poll()
                if process.poll() is None:
                    receipt['checks']['malformed_outcome'] = 'survived_observation_window'
                else:
                    receipt['checks']['malformed_outcome'] = 'process_or_debugger_ended'
            else:
                clock(10.0)
                pump(0.1)
                # Both missing pose and missing twist must be rejected without a crash.
                # 逐项破坏 ModelStates 平行数组长度，要求节点不崩溃且不生成里程计。
                for field in ('pose', 'twist'):
                    malformed = valid_message()
                    setattr(malformed, field, [])
                    before = len(odometry)
                    model_pub.publish(malformed)
                    pump(0.15)
                    assert process.poll() is None, f'malformed {field}: node crashed'
                    assert len(odometry) == before, f'malformed {field}: unexpected odometry'
                receipt['checks']['malformed_parallel_arrays_rejected'] = True
                model_pub.publish(valid_message())
                until(lambda: len(odometry) > 0, 3, 'valid odometry')
                observed = odometry[-1]
                # 合法输入应采用受控仿真时间，并输出命名空间内 odom/base_link 及已转换的船体系速度。
                assert observed.header.stamp.sec == 10
                assert observed.header.frame_id == namespace + '/odom'
                assert observed.child_frame_id == namespace + '/base_link'
                assert math.isclose(observed.twist.twist.linear.x, 4.0, abs_tol=1e-9)
                assert math.isclose(observed.twist.twist.linear.y, -3.0, abs_tol=1e-9)
                assert math.isclose(observed.twist.twist.angular.z, 0.2, abs_tol=1e-9)
                receipt['checks']['world_to_body_velocity'] = [4.0, -3.0, 0.2]
                before = len(odometry)
                clock(10.0)
                pump(0.1)
                model_pub.publish(valid_message())
                pump(0.2)
                # 同一仿真时刻的重复状态不应再次产生里程计，避免同时间样本重复消费。
                assert len(odometry) == before, 'duplicate clock produced duplicate odometry'
                receipt['checks']['duplicate_clock_suppressed'] = True
                # 从十秒回退到五秒，再推进到五点一秒，分别验证回退后的首帧和后续时间进展。
                for value in (5.0, 5.1):
                    before = len(odometry)
                    clock(value)
                    pump(0.1)
                    model_pub.publish(valid_message())
                    until(lambda: len(odometry) > before, 3, 'clock rewind recovery')
                    stamp = odometry[-1].header.stamp
                    assert stamp.sec * 10**9 + stamp.nanosec == round(value * 1e9)
                receipt['checks']['clock_rewind_and_subsequent_progress'] = True
                tick = [6.0]
                def stream():
                    tick[0] += 0.02
                    clock(tick[0])
                    model_pub.publish(valid_message())
                # 在合法输入持续到达时发送 SIGINT，退出观察期间继续驱动输入，覆盖回调活动中的关闭路径。
                pump(0.3, stream)
                owned_signal(signal.SIGINT)
                deadline = time.monotonic() + 5
                while process.poll() is None and time.monotonic() < deadline:
                    pump(0.02, stream)
                assert process.poll() == 0, f'legal-stream SIGINT exit: {process.poll()}'
                receipt['checks']['legal_stream_sigint_exit_zero'] = True
            receipt['completed'] = True
    except Exception as error:
        receipt.update(completed=False, error=f'{type(error).__name__}: {error}')
    finally:
        # 仍存活的自有进程依次尝试 SIGINT、SIGTERM、SIGKILL；每次信号发送都会重新核验身份。
        if process is not None and process.poll() is None:
            owned_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                owned_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    owned_signal(signal.SIGKILL)
                    process.wait(timeout=5)
        receipt['process_returncode'] = process.returncode if process is not None else None
        receipt['finished_monotonic_s'] = time.monotonic()
        driver.destroy_node()
        rclpy.shutdown()
        data = log_path.read_bytes() if log_path.exists() else b''
        # 归档退出码、日志哈希和调试器实际观测，异常路径也落盘 receipt，便于追溯本次运行。
        receipt['log_sha256'] = hashlib.sha256(data).hexdigest()
        receipt['gdb_observed_sigsegv'] = args.gdb and b'received signal SIGSEGV' in data
        with (args.output / 'receipt.json').open('x') as output:
            json.dump(receipt, output, indent=2, allow_nan=False)
            output.write('\n')
        print(json.dumps(receipt, indent=2), flush=True)
    return 0 if receipt.get('completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
