# 决策与控制节点共用的 ROS 输入适配层；每个节点独立保存里程计、代价地图和模块心跳快照。
"""Shared input adapters; each node owns its own received state/map snapshots."""
from collections import defaultdict, deque
import json
import math
import os
import resource
import time
import uuid

import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from tf2_ros import Buffer, TransformListener, TransformException
from common.models import GridSnapshot, USVModel, VesselState
from common.math_helper import quaternion_yaw, wrap_angle
from common.collision import footprint_path_is_clear
from usv_navigation_interfaces.msg import ModuleHeartbeat
from .configuration import DEFAULTS, HOT_PARAMETERS, resolve_parameters, section, validate_update
from .heartbeat import PeerSession, HEARTBEAT_PERIOD
from .node_runner import run_node


# 将 ROS 秒与纳秒时间字段合成浮点秒。
def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 通过已验证的四元数工具提取 ROS 平面航向。
def yaw(q):
    return quaternion_yaw(q.x, q.y, q.z, q.w)


# 声明参数、TF 与输入订阅，按命名空间构造 map/base_link，采用稳态时钟发送心跳。
class NavigationInputs(Node):
    def __init__(self, name, role):
        super().__init__(name)
        for key, default in DEFAULTS.items():
            self.declare_parameter(key, default, ParameterDescriptor(read_only=key not in HOT_PARAMETERS))
        self.config = resolve_parameters({key: self.get_parameter(key).value for key in DEFAULTS})
        self.add_on_set_parameters_callback(self._validate_parameters)
        prefix = self.get_namespace().strip('/')
        self.frame = (prefix + '/' if prefix else '') + 'map'
        self.base_frame = (prefix + '/' if prefix else '') + 'base_link'
        self.model = USVModel.from_mapping(section(self.config, 'dynamics.'))
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.state = self.grid = self.pending_odom = None
        self.reason = ''
        self.counters = defaultdict(int)
        self.timings = defaultdict(lambda: deque(maxlen=10000))
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.session, self.module_role = uuid.uuid4().hex, role
        self.peers = {key: PeerSession() for key in ('decision', 'control', 'perception') if key != role}
        self._heartbeat_sequence = 0
        self.heartbeat_pub = self.create_publisher(ModuleHeartbeat, 'navigation/heartbeat', 10)
        self.create_subscription(ModuleHeartbeat, 'navigation/heartbeat', self._on_heartbeat, 10)
        self.create_timer(HEARTBEAT_PERIOD, self.publish_heartbeat, clock=self.steady_clock)
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Costmap, self.config['navigation.costmap_topic'], self._on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info('Effective navigation parameters: ' + json.dumps(self.config, sort_keys=True))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ROS 参数写入回调只验证候选值；业务循环在拍边界应用通过的热更新。
    def _validate_parameters(self, parameters):
        try:
            current = {key: self.get_parameter(key).value for key in DEFAULTS}
            validate_update(current, {p.name: p.value for p in parameters})
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def publish_heartbeat(self):
        self._heartbeat_sequence += 1
        self.heartbeat_pub.publish(ModuleHeartbeat(
            module=self.module_role, session=self.session, sequence=self._heartbeat_sequence,
            ready=True))

    # 按墙钟接收时间更新对端会话，分别调用首次可用或重启钩子。
    def _on_heartbeat(self, message):
        peer = self.peers.get(message.module)
        if peer is None:
            return
        change = peer.observe(message.session, message.sequence, time.monotonic(), message.ready)
        if change == 'restart':
            self.counters['peer_restarts'] += 1
            self._on_peer_restart(message.module)
        elif change == 'new':
            self._on_peer_available(message.module)

    def _on_peer_restart(self, role):
        pass

    def _on_peer_available(self, role):
        pass

    # 指定角色均需心跳新鲜且 ready 标志为真。
    def peers_ready(self, *roles):
        now = time.monotonic()
        return all(self.peers[role].fresh(now) and self.peers[role].ready for role in roles)

    # 按消息时间查询 TF，把来源平面位置与航向变换到本节点 map 坐标系。
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
        return float(x), float(y), float(heading)

    # 暂存最新里程计并尝试立即转换；TF 未到时留待后续循环重试。
    def _on_odom(self, message):
        self.pending_odom = message
        self._consume_odom()

    # 要求 twist 已属于本船 base_link，位置经 TF 转换后直接保留船体系 u/v/r 的符号。
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
            self.state, self.pending_odom = state, None
        # 缺少 TF 时保留待处理里程计；格式错误则丢弃，避免反复消费坏数据。
        except TransformException:
            self.counters['odom_tf_waits'] += 1
        except ValueError as error:
            self.pending_odom = None
            self.counters['invalid_odom'] += 1
            self.reason = str(error)

    # 只接受同 map 坐标系且原点未旋转的 Nav2 原始地图，按 size_y×size_x 重建只读快照。
    def _on_map(self, message):
        begun = time.perf_counter()
        try:
            meta = message.metadata
            if message.header.frame_id != self.frame:
                raise ValueError('costmap frame mismatch')
            if abs(yaw(meta.origin.orientation)) > 1e-8:
                raise ValueError('rotated costmap origins are unsupported')
            data = np.asarray(message.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)
            stamp = seconds(message.header.stamp)
            self.grid = GridSnapshot(data, meta.resolution, meta.origin.position.x,
                                     meta.origin.position.y, stamp, message.header.frame_id)
            self._map_updated()
        except (ValueError, TypeError) as error:
            self.counters['invalid_costmap'] += 1
            self.reason = str(error)
        self.timings['costmap_conversion_ms'].append((time.perf_counter() - begun) * 1000)

    def _map_updated(self):
        pass

    # 用最新状态的单个位姿检查完整矩形船体，而非仅检查船舶中心点。
    def _current_footprint_clear(self):
        return self.state is not None and self.grid is not None and footprint_path_is_clear(
            self.grid, [self.state.pose], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    # 清除状态、地图、待转换里程计与 TF，防止时间重置后继续使用旧输入。
    def _clear_inputs(self):
        self.state = self.grid = self.pending_odom = None
        self.tf.clear()

    # 输出每类计时样本的分位数以及本进程 CPU/RSS 统计，供诊断报告汇总。
    def statistics(self):
        metrics = {name: {'count': len(samples), 'p50': float(np.percentile(samples, 50)),
                         'p95': float(np.percentile(samples, 95)), 'p99': float(np.percentile(samples, 99)),
                         'max': max(samples)} for name, samples in self.timings.items() if samples}
        usage = resource.getrusage(resource.RUSAGE_SELF)
        process = dict(pid=os.getpid(), cpu_seconds=usage.ru_utime + usage.ru_stime,
                       peak_rss_kib=usage.ru_maxrss, monotonic=time.monotonic())
        return metrics, dict(self.counters), process
