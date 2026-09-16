# 拆分前的单节点导航参照，仅用于回归对照；当前业务入口使用独立决策、航控节点。
"""Pre-split LiDAR navigation oracle, for regression tests only (2026-09-13).

This module is not installed or launched. Production uses independent nodes.
"""
from collections import defaultdict, deque
import json
import math
import time
import os
import resource
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import Costmap
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from common.models import AlgorithmStatus, ControlCommand, GridSnapshot, USVModel, VesselState
from common.math_helper import quaternion_yaw, wrap_angle
from common.collision import footprint_path_is_clear
from common.visualizer import make_path
from controller.controller_factory import create_controller
from sim_env.configuration import DEFAULTS, HOT_PARAMETERS, C, T, resolve_parameters, section, validate_update
from sim_env.runtime import LatestPlanWorker, PlanningRequest, fresh, readiness


# 将 ROS 时间戳的秒与纳秒合并为浮点秒。
def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 从消息四元数提取平面艏向角，供平面状态模型使用。
def yaw(q):
    return quaternion_yaw(q.x, q.y, q.z, q.w)


# 保留旧单节点的感知输入、异步规划和控制调度，用于与拆分后的实现做回归对照。
class Simulator(Node):
    def __init__(self):
        super().__init__('usv_simulation')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=name not in HOT_PARAMETERS))
        self.config = resolve_parameters({k: self.get_parameter(k).value for k in DEFAULTS})
        self.add_on_set_parameters_callback(self._validate_parameters)
        prefix = self.get_namespace().strip('/')
        self.frame = (prefix + '/' if prefix else '') + 'map'
        self.base_frame = (prefix + '/' if prefix else '') + 'base_link'
        self.model = USVModel.from_mapping(section(self.config, 'dynamics.'))
        self.controller_name = self.config[C + 'controller_name']
        self.controller = create_controller(self.controller_name, self.model,
                                           section(self.config, C + self.controller_name + '_controller.'))
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.worker = LatestPlanWorker()
        self.version = 0
        self.state = self.grid = self.trajectory = self.goal = self.pending_odom = None
        self.route = np.empty((0, 3))
        self.mode, self.reason = 'IDLE', ''
        self.counters = defaultdict(int)
        self.timings = defaultdict(lambda: deque(maxlen=10000))
        self.last_clock = self.now_seconds()
        self.last_control = self.last_clock
        self.last_status_wall = 0.0
        self.path_pub = self.create_publisher(Path, 'global_path', 1)
        self.trajectory_pub = self.create_publisher(Path, 'local_trajectory', 1)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.thrust_pubs = [self.create_publisher(Float32, 'thrusters/' + name + '_thrust_cmd', 1)
                           for name in ('left', 'right', 'lateral')]
        self.create_subscription(PoseStamped, 'goal', self._on_goal, 1)
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Costmap, self.config['navigation.costmap_topic'], self._on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, 'cancel_navigation', self._cancel)
        self.create_service(Trigger, 'export_parameters', self._export)
        # A steady timer also emits zero commands while simulation time is paused.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.config[C + 'control_frequency'], self._control, clock=self.steady_clock)
        self.create_timer(1.0 / self.config['navigation.planning_frequency'], self._planning)
        self.get_logger().info('Effective navigation parameters: ' + json.dumps(self.config, sort_keys=True))

    # 读取 ROS 时钟秒数；状态和轨迹的新鲜度都需要使用同一时间基准。
    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 先验证参数更新候选值；失败时拒绝整批更新，防止留下部分生效配置。
    def _validate_parameters(self, parameters):
        try:
            # Validation has no side effects: rclpy has not committed the values yet.
            current = {k: self.get_parameter(k).value for k in DEFAULTS}
            validate_update(current, {p.name: p.value for p in parameters})
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    # 在控制调度中应用已确认参数，必要时使旧规划结果失效。
    def _apply_parameters(self):
        changes = {k: self.get_parameter(k).value for k in HOT_PARAMETERS
                   if self.get_parameter(k).value != self.config[k]}
        if not changes:
            return
        self.config = validate_update(self.config, changes)
        if self.controller_name == 'pid':
            self.controller.update_config(section(self.config, C + 'pid_controller.'))
        if any(k.startswith(T) for k in changes):
            self.version += 1
            self.worker.invalidate()
            self.trajectory = None
        self.counters['parameter_updates'] += 1

    # 用消息对应时刻的 TF 转换位置与艏向，不把不同坐标系的数据直接拼接。
    def _pose(self, pose, frame, stamp):
        x, y, heading = pose.position.x, pose.position.y, yaw(pose.orientation)
        if not frame:
            raise ValueError('pose frame_id is empty')
        if frame != self.frame:
            transform = self.tf.lookup_transform(self.frame, frame, Time.from_msg(stamp))
            rotation = yaw(transform.transform.rotation)
            x, y = (math.cos(rotation) * x - math.sin(rotation) * y + transform.transform.translation.x,
                    math.sin(rotation) * x + math.cos(rotation) * y + transform.transform.translation.y)
            heading = wrap_angle(heading + rotation)
        if not np.isfinite((x, y, heading)).all():
            raise ValueError('pose is nonfinite')
        return (float(x), float(y), float(heading))

    # 收到里程计后交给统一转换逻辑，保持订阅入口和状态解析分开。
    def _on_odom(self, message):
        self.pending_odom = message
        self._consume_odom()

    # 构造带时间和坐标系的 VesselState，并拒绝无效状态输入。
    def _consume_odom(self):
        message = self.pending_odom
        if message is None:
            return
        try:
            if message.child_frame_id != self.base_frame:
                raise ValueError('odometry twist must use ' + self.base_frame)
            pose = self._pose(message.pose.pose, message.header.frame_id, message.header.stamp)
            twist = message.twist.twist
            state = VesselState(*pose, twist.linear.x, twist.linear.y, twist.angular.z,
                                seconds(message.header.stamp), self.frame)
            if not state.is_finite():
                raise ValueError('odometry contains nonfinite state')
            self.state = state
            self.pending_odom = None
        except TransformException:
            self.counters['odom_tf_waits'] += 1
        except ValueError as error:
            self.pending_odom = None
            self.counters['invalid_odom'] += 1
            self.reason = str(error)

    # 将收到的地图转换为内部快照；新障碍可能立即使当前参考失效并触发停推。
    def _on_map(self, message):
        begun = time.perf_counter()
        try:
            meta = message.metadata
            if message.header.frame_id != self.frame:
                raise ValueError('costmap frame mismatch')
            if abs(yaw(meta.origin.orientation)) > 1e-8:
                raise ValueError('rotated costmap origins are unsupported')
            data = np.asarray(message.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)
            map_stamp = seconds(message.header.stamp)
            self.grid = GridSnapshot(data, meta.resolution, meta.origin.position.x,
                                     meta.origin.position.y, map_stamp, message.header.frame_id)
            if self.trajectory is not None and not self._trajectory_clear(self.trajectory):
                self.trajectory = None
                self.route = np.empty((0, 3))
                self.mode, self.reason = 'WAITING_REFERENCE', 'Latest map invalidated reference'
                self.counters['map_invalidations'] += 1
                self._publish_command(ControlCommand())
        except (ValueError, TypeError) as error:
            self.counters['invalid_costmap'] += 1
            self.reason = str(error)
        self.timings['costmap_conversion_ms'].append((time.perf_counter() - begun) * 1000)

    # 同时检查当前船体与参考扫掠区域，不只检测轨迹中心线。
    def _trajectory_clear(self, trajectory):
        return self._current_footprint_clear() and footprint_path_is_clear(
            self.grid, trajectory.points[:, [2, 3, 4]], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    # 按当前姿态检查完整矩形船体占用，覆盖船首、船尾和四角。
    def _current_footprint_clear(self):
        return self.state is not None and self.grid is not None and footprint_path_is_clear(
            self.grid, [self.state.pose], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    # 任务边界清除旧目标、轨迹和规划请求，同时重置控制状态并发布零输出。
    def _reset_task(self, mode, reason='', clear_inputs=False):
        self.version += 1
        self.worker.invalidate()
        self.goal = self.trajectory = None
        self.route = np.empty((0, 3))
        self.controller.reset()
        self.mode, self.reason = mode, reason
        if clear_inputs:
            self.state = self.grid = self.pending_odom = None
            self.tf.clear()
        self._publish_command(ControlCommand())
        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(make_path(self.route, self.frame, stamp))
        self.trajectory_pub.publish(make_path(self.route, self.frame, stamp))

    # 转换外部目标到导航坐标系，建立新任务并触发规划。
    def _on_goal(self, message):
        # A rejected replacement must not silently continue the previous mission.
        self._reset_task('WAITING_REFERENCE')
        try:
            self.goal = self._pose(message.pose, message.header.frame_id, message.header.stamp)
            self._planning()
        except (ValueError, TransformException) as error:
            self.mode, self.reason = 'INVALID_GOAL', str(error)

    # 取消服务进入任务重置流程；零推力不代表船体已物理停止。
    def _cancel(self, request, response):
        self._reset_task('CANCELLED')
        response.success, response.message = True, 'Navigation cancelled; zero thrust active'
        return response

    # 导出节点当前有效参数，便于核对运行状态而不是只看启动文件。
    def _export(self, request, response):
        values = {k: self.get_parameter(k).value for k in DEFAULTS}
        response.success = True
        response.message = yaml.safe_dump({self.get_fully_qualified_name(): {'ros__parameters': values}}, sort_keys=True)
        return response

    # 仅在状态和地图有效时提交带快照的规划请求，避免让异步工作进程读取可变输入。
    def _planning(self):
        now = self.now_seconds()
        if self.goal is None or self.state is None or self.grid is None:
            return
        if not fresh(self.state.stamp, now, self.config['navigation.state_timeout']) or not fresh(
                self.grid.stamp, now, self.config['navigation.map_timeout']):
            return
        try:
            self.worker.offer(PlanningRequest(self.version, self.goal, self.state, self.grid,
                                               self.route.copy(), dict(self.config)))
        except Exception as error:
            self.trajectory = None
            self.mode, self.reason = 'PLANNING_ERROR', str(error)
            self.counters['worker_errors'] += 1
            self._publish_command(ControlCommand())

    # 接收最新有效规划结果并再次检查地图，过期结果不能恢复旧任务。
    def _consume_plan(self):
        try:
            result = self.worker.poll()
        except Exception as error:
            self.trajectory = None
            self.mode, self.reason = 'PLANNING_ERROR', str(error)
            self.counters['worker_errors'] += 1
            return
        if result is None:
            return
        self.timings['planning_ms'].append(result.elapsed * 1000)
        if result.version != self.version or self.goal is None:
            self.counters['stale_results'] += 1
            return
        self.counters['global_searches'] += int(result.searched)
        self.route = result.route
        self.reason = result.message
        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(make_path(self.route, self.frame, stamp))
        if result.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            self.mode = result.status.value.upper()
            self.trajectory = None
            return
        if not self._trajectory_clear(result.trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Worker reference conflicts with latest map'
            self.counters['unsafe_results'] += 1
            return
        self.trajectory = result.trajectory
        self.trajectory_pub.publish(make_path(self.trajectory.points[:, [2, 3, 4]], self.frame, stamp))

    # 将数值控制输出转换为推进器话题消息，三个通道保持一致的发布流程。
    def _publish_command(self, command):
        for publisher, value in zip(self.thrust_pubs, (command.left, command.right, command.lateral)):
            publisher.publish(Float32(data=float(value)))
        self.last_published_command = command

    # 控制周期依次处理参数、状态与规划结果，校验输入后跟踪轨迹，失效时停推。
    def _control(self):
        begun = time.perf_counter()
        now = self.now_seconds()
        if now < self.last_clock - 1e-6:
            self._reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
            self.counters['clock_resets'] += 1
        dt = now - self.last_control
        self.last_clock = self.last_control = now
        command = ControlCommand()
        try:
            self._apply_parameters()
            self._consume_odom()
            self._consume_plan()
            if self.goal is not None:
                ready = readiness(now, self.state, self.grid, self.trajectory, self.config)
                # Tracking errors can put the actual hull outside the safe
                # reference. Recheck it every control tick, even when planning
                # has not returned another trajectory yet.
                if ready in ('NAVIGATING', 'WAITING_REFERENCE') and not self._current_footprint_clear():
                    self.trajectory = None
                    self.route = np.empty((0, 3))
                    ready = 'CURRENT_FOOTPRINT_BLOCKED'
                    self.reason = 'Current vessel footprint intersects lethal or unknown map cells'
                    self.counters['current_footprint_blocks'] += 1
                if ready == 'NAVIGATING' and dt > 0:
                    self.mode = ready
                    command = self.controller.compute(self.state, self.trajectory, dt)
                    if command.status == AlgorithmStatus.GOAL_REACHED:
                        self._reset_task('GOAL_REACHED')
                    elif command.status == AlgorithmStatus.SETTLING:
                        self.mode, self.reason = 'SETTLING', command.message
                    elif command.status != AlgorithmStatus.SUCCESS or not command.is_valid():
                        self.mode, self.reason = 'CONTROL_ERROR', command.message
                        # Count every control-step failure; one-second status
                        # snapshots alone can miss a transient solver timeout.
                        self.counters['control_result_' + command.status.value] += 1
                        command = ControlCommand()
                elif dt <= 0:
                    self.mode = 'SIMULATION_PAUSED'
                else:
                    # Keep the explicit planning failure reason until a new plan exists.
                    if ready != 'WAITING_REFERENCE' or self.trajectory is not None or not self.reason:
                        self.mode = ready
            elapsed = (time.perf_counter() - begun) * 1000
            if elapsed >= 1000 / self.config[C + 'control_frequency']:
                self.counters['control_overruns'] += 1
            self.timings['control_ms'].append(elapsed)
        except Exception as error:
            self.mode, self.reason = 'CONTROL_ERROR', str(error)
            self.counters['control_errors'] += 1
            command = ControlCommand()
        self._publish_command(command)
        if time.perf_counter() - self.last_status_wall >= 1:
            self._publish_status()

    # 发布任务状态、计算耗时和过程统计，供监视与验收工具读取。
    def _publish_status(self):
        self.last_status_wall = time.perf_counter()
        metrics = {name: {'count': len(samples), 'p50': float(np.percentile(samples, 50)),
                          'p95': float(np.percentile(samples, 95)), 'p99': float(np.percentile(samples, 99)),
                          'max': max(samples)} for name, samples in self.timings.items() if samples}
        self.counters['pending_replacements'] = self.worker.replaced
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.mode, reason=self.reason, version=self.version, stamp=self.now_seconds(),
            controller=self.controller_name, goal=self.goal, metrics=metrics, counters=dict(self.counters),
            process={'pid': os.getpid(), 'cpu_seconds': usage.ru_utime + usage.ru_stime,
                     'peak_rss_kib': usage.ru_maxrss, 'monotonic': time.monotonic()}))))

    # 退出时先请求零推力并关闭工作进程，再释放 ROS 节点资源。
    def destroy_node(self):
        if self.context.ok():
            self._publish_command(ControlCommand())
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    # Keep the context alive until the last zero command has been published.
    # rclpy's asynchronous default signal shutdown races that final publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = Simulator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    main()
