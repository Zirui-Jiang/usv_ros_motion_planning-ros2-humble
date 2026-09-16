#!/usr/bin/env python3
# 基础导航与故障验收工具：采集真实 ROS 数据，验证到达、航迹偏差、扫掠间距及取消/里程计失效停车。
"""Run actual ROS 2/Gazebo acceptance trials; never synthesize trial results.

Source ROS 2 Humble and the Python workspace before running this script in
WSL/Linux. Only child process groups started by this harness are terminated.
ROS imports are delayed so --dry-run and geometry tests work without ROS.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

from task_metrics import distribution, point_to_polyline_distance, swept_clearance, trial_verdict, wrap_angle, supervision_stop_reason
from task_graph_guard import GRAPH_REQUIREMENTS, activate_graph_guard
from task_source_layout import find_source_root


# 缺少安装、服务、输入或环境预条件时标记试验阻塞，避免混作算法测量结果。
class TrialBlocked(RuntimeError):
    pass


# 单独标记控制器名称暂未返回，可进行有界重试。
class EmptyParameterResponse(TrialBlocked):
    pass


# 递归将 NaN/无穷值换成 JSON null，保持严格 JSON 文件可解析。
def sanitized(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): sanitized(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitized(v) for v in value]
    return value


# 逐行写事件并即时 flush，保留异常或中断之前已经收到的观测。
class Jsonl:
    def __init__(self, path):
        self.file = path.open("w", encoding="utf-8")

    def write(self, event, **values):
        payload = sanitized({"event": event, "wall_monotonic_s": time.monotonic(), **values})
        self.file.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


# 仅管理本工具创建的 launch 进程组，退出先交给 launch 转发信号。
class OwnedProcess:
    def __init__(self, workspace, command, logfile, environment):
        setup = workspace / "install" / "setup.bash"
        if not setup.is_file():
            raise TrialBlocked(f"workspace is not built: {setup}")
        self.output = logfile.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            ["bash", "-c", 'source "$1"; shift; exec "$@"', "validation", str(setup), *command],
            cwd=workspace, env=environment, start_new_session=True,
            stdout=self.output, stderr=subprocess.STDOUT,
        )

    def check(self):
        if self.process.poll() is not None:
            raise TrialBlocked(f"owned ROS process exited with code {self.process.returncode}; inspect launch.log")

    def close(self):
        # Let ros2 launch forward the first SIGINT itself. Signalling the whole
        # group here as well would deliver SIGINT twice to child nodes while
        # they are destructing ROS entities.
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                pass
        # The launch parent may have exited while its children still run.
        # The session/group id is owned by us even in that failure case.
        for sig, timeout in [(signal.SIGTERM, 4), (signal.SIGKILL, 3)]:
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.process.poll()
                try:
                    os.killpg(self.process.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                continue
            break
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
        self.output.close()


# 临时绑定本机端口取得可用 Gazebo master 端口号。
def free_port():
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        return server.getsockname()[1]


# 把 ROS 秒与纳秒转为秒值，用于保留消息自身的采样时刻。
def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 按 ROS ParameterValue 类型编号取出标量或数组。
def parameter_value(value):
    fields = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
              5: "byte_array_value", 6: "bool_array_value", 7: "integer_array_value", 8: "double_array_value", 9: "string_array_value"}
    item = getattr(value, fields[value.type]) if value.type in fields else None
    return list(item) if value.type >= 5 else item


# 延迟导入 ROS 并创建订阅探针，记录状态、地图、路径、命令及参数事件。
def make_probe(namespace, log, use_sim_time=True):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Trigger, Empty
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    from rcl_interfaces.srv import GetParameters, ListParameters, SetParametersAtomically
    from rcl_interfaces.msg import ParameterEvent

    # 把多个公开话题组成验收观测，不访问规划器内部私有状态。
    class Probe(Node):
        def __init__(self):
            super().__init__("acceptance_" + uuid.uuid4().hex[:8],
                             parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
            self.namespace = "/" + namespace.strip("/")
            self.types = locals_types
            self.log = log
            self.graph_guard = None
            self.pose = None
            self.previous_pose = None
            self.velocity = None
            self.odom_stamp = None
            self.odom_wall = None
            self.scan_wall = None
            self.costmap = None
            self.costmap_wall = None
            self.costmap_stamp = None
            self.costmap_frame = None
            self.costmap_topic = None
            self.costmap_discovery_started = time.monotonic()
            self.path = []
            self.initial_path = []
            self.trajectory = []
            self.received_nonempty_trajectory = False
            self.last_nonempty_trajectory = None
            self.path_stamp = None
            self.path_frame = None
            self.failure_snapshot_active = False
            self.commands = {}
            self.command_wall = {}
            self.command_gaps = []
            self.status = None
            self.mapping_status = None
            self.actual_goal_tolerance = None
            self.goal_wall = None
            self.active_goal = None
            self.first_path_latency = None
            self.first_command_latency = None
            self.scan_latency = []
            self.path_latency = []
            self.command_state_age = []
            self.odom_history = []
            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, self.namespace + "/odom", self.on_odom, sensor_qos)
            self.create_subscription(LaserScan, self.namespace + "/sensors/lidars/lidar/scan", self.on_scan, sensor_qos)
            self.create_subscription(RosPath, self.namespace + "/global_path", self.on_path, 10)
            self.create_subscription(RosPath, self.namespace + "/local_trajectory", self.on_trajectory, 10)
            self.create_subscription(String, self.namespace + "/navigation_status", self.on_status, 10)
            self.create_subscription(String, self.namespace + "/mapping_status", self.on_mapping_status, 10)
            self.create_subscription(ParameterEvent, "/parameter_events", self.on_parameter_event, 100)
            self.create_subscription(PoseStamped, self.namespace + "/goal", self.on_goal, 10)
            for axis in ("left", "right", "lateral"):
                self.create_subscription(Float32, self.namespace + f"/thrusters/{axis}_thrust_cmd",
                                         lambda msg, axis=axis: self.on_command(axis, msg), 10)
            self.goal_pub = self.create_publisher(PoseStamped, self.namespace + "/goal", 10)

        # 从探针 ROS 时钟取得仿真秒数。
        def sim_time(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # 报告消息头相对 ROS 时间的年龄，单位毫秒；无有效时戳返回缺失值。
        def message_age(self, header):
            stamp = stamp_seconds(header.stamp)
            return max(0.0, self.sim_time() - stamp) * 1000 if stamp > 0 else None

        # 提取 map 平面位姿和船体系速度，保留源时戳与回调墙钟时刻。
        def on_odom(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.previous_pose = self.pose
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
            self.odom_stamp = stamp_seconds(msg.header.stamp)
            self.odom_wall = time.monotonic()
            self.odom_history.append((self.odom_wall, self.pose))
            self.log.write("odom", sim_time_s=self.odom_stamp, pose=self.pose, body_velocity=self.velocity,
                           frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id)

        # 记录扫描回调时刻和消息头年龄，用于输入延迟诊断。
        def on_scan(self, msg):
            self.scan_wall = time.monotonic()
            self.scan_latency.append(self.message_age(msg.header))

        # 仅把接近当前目标的路径作为首次参考，记录目标到路径回调的等待时长。
        def on_path(self, msg):
            self.path_stamp = stamp_seconds(msg.header.stamp)
            self.path_frame = msg.header.frame_id
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            for_active_goal = bool(self.path) and self.active_goal is not None and math.hypot(
                self.path[-1][0] - self.active_goal[0], self.path[-1][1] - self.active_goal[1]) <= 1.0
            if for_active_goal and not self.initial_path and self.goal_wall is not None:
                self.initial_path = list(self.path)
            if for_active_goal and self.first_path_latency is None and self.goal_wall is not None:
                self.first_path_latency = time.monotonic() - self.goal_wall
            self.path_latency.append(self.message_age(msg.header))
            self.log.write("reference_path", sim_time_s=stamp_seconds(msg.header.stamp),
                           frame_id=msg.header.frame_id, points=self.path)

        # 保存局部路径与航向，并保留最后一条非空轨迹供故障复现。
        def on_trajectory(self, msg):
            self.trajectory = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.received_nonempty_trajectory |= bool(self.trajectory)
            poses = []
            for p in msg.poses:
                q = p.pose.orientation
                poses.append([p.pose.position.x, p.pose.position.y,
                              math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])
            payload = {"sim_time_s": stamp_seconds(msg.header.stamp), "frame_id": msg.header.frame_id,
                       "points": self.trajectory, "poses": poses}
            if self.trajectory:
                self.last_nonempty_trajectory = payload
            self.log.write("local_trajectory", **payload)

        # 解析导航状态；首次出现参考失败时保存当时已收到的输入快照。
        def on_status(self, msg):
            try:
                self.status = json.loads(msg.data)
            except json.JSONDecodeError:
                self.status = {"unparsed_status": msg.data}
            self.log.write("navigation_status", payload=self.status)
            state = self.status.get("state", "")
            reason = self.status.get("reason", "")
            if state == "NAVIGATING":
                self.failure_snapshot_active = False
            failed_reference = state in ("NO_PATH", "WAITING_REFERENCE", "MAP_INVALID") or (
                "map" in reason.lower() and "invalid" in reason.lower())
            if self.active_goal is not None and failed_reference and not self.failure_snapshot_active:
                if self.capture_inputs("navigation_failure_snapshot"):
                    self.failure_snapshot_active = True

        # 保存建图诊断消息，无法解析的原文也保留。
        def on_mapping_status(self, msg):
            try:
                self.mapping_status = json.loads(msg.data)
            except json.JSONDecodeError:
                self.mapping_status = {"unparsed_status": msg.data}
            self.log.write("mapping_status", payload=self.mapping_status)

        # 仅记录当前船舶任务节点的参数变化，便于追踪试验中实际配置。
        def on_parameter_event(self, msg):
            if msg.node != self.namespace + "/mission_manager":
                return
            self.log.write("navigation_parameter_event", node=msg.node, sim_time_s=stamp_seconds(msg.stamp),
                           new=[{"name": item.name, "value": parameter_value(item.value)} for item in msg.new_parameters],
                           changed=[{"name": item.name, "value": parameter_value(item.value)} for item in msg.changed_parameters],
                           deleted=[item.name for item in msg.deleted_parameters])

        # 记录从公开目标话题观察到的请求及源时刻。
        def on_goal(self, msg):
            self.log.write("goal_observed", x=msg.pose.position.x, y=msg.pose.position.y,
                           sim_time_s=stamp_seconds(msg.header.stamp))

        # 按推进器轴记录归一化推力，并统计左推进器回调间隔和首次非零输出时延。
        def on_command(self, axis, msg):
            now = time.monotonic()
            previous = self.command_wall.get(axis)
            if previous is not None and axis == "left":
                self.command_gaps.append((now - previous) * 1000)
            self.commands[axis] = float(msg.data)
            self.command_wall[axis] = now
            if self.odom_wall is not None and axis == "left":
                self.command_state_age.append((now - self.odom_wall) * 1000)
            if self.goal_wall is not None and abs(msg.data) > 1e-6 and self.first_command_latency is None:
                self.first_command_latency = now - self.goal_wall
            self.log.write("thruster_command", axis=axis, normalized_command=float(msg.data))

        # 优先选择 Nav2 原始代价图，短暂等待其出现后才允许退回有损占用图表示。
        def discover_costmap(self, explicit=None):
            if self.costmap_topic:
                return
            candidates = []
            for topic, types in self.get_topic_names_and_types():
                if explicit and topic != explicit:
                    continue
                if not explicit and not topic.startswith(self.namespace + "/"):
                    continue
                if "nav2_msgs/msg/Costmap" in types and (explicit or topic.endswith("costmap_raw")):
                    candidates.append((0, topic, Costmap))
                elif "nav_msgs/msg/OccupancyGrid" in types and (explicit or "costmap" in topic):
                    candidates.append((1, topic, OccupancyGrid))
            if candidates:
                priority, topic, kind = sorted(candidates, key=lambda item: (item[0], item[1]))[0]
                # Topic discovery is asynchronous: OccupancyGrid can appear a
                # fraction earlier than the raw topic. Do not lock onto lossy
                # display costs just because it was the first graph event.
                if priority > 0 and not explicit and time.monotonic() - self.costmap_discovery_started < 5.0:
                    return
                self.create_subscription(kind, topic, self.on_costmap, QoSProfile(depth=1))
                self.costmap_topic = topic
                self.log.write("costmap_discovered", topic=topic, type=kind.__name__)

        # 兼容原始代价图和占用图消息，记录类型标签以使用相应障碍阈值。
        def on_costmap(self, msg):
            if hasattr(msg, "metadata"):
                meta = msg.metadata
                origin = meta.origin.position
                self.costmap = (int(meta.size_x), int(meta.size_y), float(meta.resolution), origin.x, origin.y, list(msg.data), "raw")
            else:
                meta = msg.info
                origin = meta.origin.position
                self.costmap = (int(meta.width), int(meta.height), float(meta.resolution), origin.x, origin.y, list(msg.data), "occupancy")
            self.costmap_wall = time.monotonic()
            self.costmap_stamp = stamp_seconds(msg.header.stamp)
            self.costmap_frame = msg.header.frame_id

        # 保存地图 NPZ 与状态/参考 JSON；各订阅源时戳分开保留，并非原子规划快照。
        def capture_inputs(self, event):
            """Capture the latest callback data at each failed-reference episode.

            Source stamps are explicit because separate ROS subscriptions are
            not an atomic private planning-worker snapshot.
            """
            if self.costmap is not None:
                import numpy as np
                width, height, resolution, ox, oy, data, kind = self.costmap
                input_id = uuid.uuid4().hex[:8]
                directory = Path(self.log.file.name).parent
                snapshot_path = directory / ("costmap_input_" + input_id + ".npz")
                state_path = directory / ("state_input_" + input_id + ".json")
                np.savez_compressed(snapshot_path, data=np.asarray(data, dtype=np.uint8 if kind == "raw" else np.int16).reshape(height, width),
                                    resolution=resolution, origin_x=ox, origin_y=oy, frame_id=self.costmap_frame,
                                    stamp=self.costmap_stamp, cost_kind=kind)
                state_path.write_text(json.dumps({"goal": self.active_goal, "pose": self.pose, "body_velocity": self.velocity,
                                                 "stamp": self.odom_stamp, "frame_id": self.costmap_frame,
                                                 "costmap_file": snapshot_path.name}, indent=2), encoding="utf-8")
                payload = {"costmap_path": snapshot_path.name, "state_path": state_path.name}
                if event != "goal_input_snapshot":
                    context_path = directory / ("navigation_context_" + input_id + ".json")
                    context = {"status": self.status, "goal": self.active_goal,
                               "path": {"points": self.path, "stamp": self.path_stamp, "frame_id": self.path_frame},
                               "last_nonempty_trajectory": self.last_nonempty_trajectory,
                               "capture_wall_monotonic_s": time.monotonic(),
                               "source_age_wall_s": {"map": time.monotonic() - self.costmap_wall,
                                                     "odom": None if self.odom_wall is None else time.monotonic() - self.odom_wall},
                               "limitation": "Latest subscription callback data; separate map/state/path stamps retained; not an atomic worker snapshot"}
                    context_path.write_text(json.dumps(sanitized(context), indent=2, allow_nan=False), encoding="utf-8")
                    payload["context_path"] = context_path.name
                self.log.write(event, **payload)
                return True
            return False

        # 清理上次目标测量基准，保存输入后以 map 坐标系发布本次目标。
        def publish_goal(self, goal):
            self.goal_wall = time.monotonic()
            self.active_goal = goal
            self.initial_path = []
            self.first_path_latency = None
            self.first_command_latency = None
            self.received_nonempty_trajectory = False
            self.last_nonempty_trajectory = None
            self.failure_snapshot_active = False
            self.capture_inputs("goal_input_snapshot")
            msg = PoseStamped()
            msg.header.frame_id = self.namespace.strip("/") + "/map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = float(goal[0]), float(goal[1])
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
            self.goal_wall = time.monotonic()
            self.goal_pub.publish(msg)
            self.log.write("goal_published", goal=goal, sim_time_s=self.sim_time())

        # 三个推进器轴均须在指定墙钟时刻之后收到足够接近零的命令。
        def zero_received_since(self, since, epsilon):
            return all(self.command_wall.get(axis, -math.inf) >= since and
                       abs(self.commands.get(axis, math.inf)) <= epsilon for axis in ("left", "right", "lateral"))

    locals_types = {"rclpy": rclpy, "Trigger": Trigger, "Empty": Empty, "SetEntityState": SetEntityState,
                   "SpawnEntity": SpawnEntity, "GetParameters": GetParameters, "ListParameters": ListParameters,
                   "SetParametersAtomically": SetParametersAtomically, "Parameter": Parameter,
                   "Odometry": Odometry, "Costmap": Costmap, "QoSProfile": QoSProfile,
                   "DurabilityPolicy": DurabilityPolicy, "ReliabilityPolicy": ReliabilityPolicy}
    return Probe()


# 短周期处理 ROS 回调，同时检查自有进程和控制发布权，直到谓词或墙钟期限。
def wait_until(probe, predicate, timeout, process=None, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process:
            process.check()
        probe.types["rclpy"].spin_once(probe, timeout_sec=0.02)
        if probe.graph_guard is not None:
            probe.graph_guard.check()
        if tick:
            tick()
        if predicate():
            return True
    return False


# 按候选服务名查找并限时等待响应；必需服务不可用时显式阻塞试验。
def call_service(probe, kind, names, request, timeout=10.0, optional=False):
    for name in names:
        client = probe.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=min(2.0, timeout)):
            probe.destroy_client(client)
            continue
        future = client.call_async(request)
        ready = wait_until(probe, future.done, timeout)
        probe.destroy_client(client)
        if not ready:
            raise TrialBlocked(f"service response timed out: {name}")
        response = future.result()
        if response is None:
            raise TrialBlocked(f"service failed: {name}")
        return response
    if optional:
        return None
    raise TrialBlocked(f"required service unavailable: {names}")


# 只重试空参数响应；明确返回了错误控制器类型时立即拒绝试验。
def verify_controller(probe, requested):
    """Retry empty replies from legacy duplicate node-name parameter services.

    An explicit, different controller is never accepted or retried away.
    """
    deadline = time.monotonic() + 5.0
    while True:
        try:
            return _verify_controller_once(probe, requested)
        except EmptyParameterResponse as error:
            if time.monotonic() >= deadline:
                raise TrialBlocked(f"controller parameter remained unavailable after 5 s: {error}") from error
            wait_until(probe, lambda: False, 0.2)


# 目标下发前原子设置试验覆盖参数，再读回校验并记录。
def apply_start_parameters(probe, overrides):
    """Apply an explicitly logged fixed test configuration before any goal."""
    if not overrides:
        return
    kind = probe.types["SetParametersAtomically"]
    request = kind.Request()
    request.parameters = [probe.types["Parameter"](name, value=value).to_parameter_msg()
                          for name, value in overrides.items()]
    response = call_service(probe, kind, [probe.namespace + "/mission_manager/set_parameters_atomically"], request)
    probe.log.write("pre_goal_parameter_configuration", requested=overrides,
                    successful=response.result.successful, reason=response.result.reason)
    if not response.result.successful:
        raise TrialBlocked("pre-goal parameter configuration rejected: " + response.result.reason)
    get_kind = probe.types["GetParameters"]
    verify = get_kind.Request()
    verify.names = list(overrides)
    actual = call_service(probe, get_kind, [probe.namespace + "/mission_manager/get_parameters"], verify)
    values = {name: parameter_value(value) for name, value in zip(verify.names, actual.values)}
    probe.log.write("pre_goal_parameter_readback", values=values)
    if values != overrides:
        raise TrialBlocked(f"pre-goal parameter readback differs: {values!r}")


# 从活动节点读取算法与动力学参数，用实际控制器到达半径替代假设阈值。
def _verify_controller_once(probe, requested):
    kind = probe.types["GetParameters"]
    request = kind.Request()
    prefix = "usv_simulation.controller."
    request.names = [prefix + "controller_name", prefix + "control_frequency", prefix + "controller_frequncy",
                     prefix + "pid_controller." + "goal_distance_tolerance", prefix + "pid_controller.goal_speed_tolerance", prefix + "pid_controller.lookahead_time",
                     *[prefix + "pid_controller." + gain for gain in ("k_p_speed", "k_i_speed", "k_d_speed", "k_p_course", "k_i_course", "k_d_course")],
                     *[prefix + "lqr_controller." + field for field in ("goal_distance_tolerance", "goal_speed_tolerance", "lookahead_time", "discrete_dt", "solve_eps", "max_iterations", "Q_matrix_diag", "R_matrix_diag")],
                     *["usv_simulation.trajectory_planner.los." + field for field in ("lookahead_distance", "switch_dist", "kp_yaw", "yaw_rate_max", "trajectory_time", "trajectory_time_interval", "goal_tolerance")],
                     "obstacle_factor", "path_planner.obstacle_factor",
                     "navigation.state_timeout", "navigation.trajectory_timeout", "dynamics.mass", "dynamics.max_force", "dynamics.D", "dynamics.cruising_speed"]
    list_kind = probe.types["ListParameters"]
    listed = call_service(probe, list_kind, [probe.namespace + "/mission_manager/list_parameters"], list_kind.Request())
    available = set(listed.result.names)
    request.names = [name for name in request.names if name in available]
    # Capture newly declared numerical settings (for example LOS slowdown)
    # without silently losing them when configuration evolves between runs.
    algorithm_prefixes = ("usv_simulation.controller.", "usv_simulation.trajectory_planner.",
                          "dynamics.", "navigation.", "path_planner.")
    request.names += sorted(name for name in available - set(request.names)
                            if name.startswith(algorithm_prefixes))
    if prefix + "controller_name" not in request.names:
        raise EmptyParameterResponse("parameter service reply omitted controller_name")
    response = call_service(probe, kind, [probe.namespace + "/mission_manager/get_parameters"], request)
    actual = response.values[0].string_value if response.values else ""
    if not actual:
        raise EmptyParameterResponse("parameter service returned an empty controller_name")
    parameters = {name: parameter_value(value) for name, value in zip(request.names, response.values)}
    for controller in ("pid", "lqr"):
        parameters.setdefault(prefix + controller + "_controller.goal_speed_tolerance", None)
    probe.log.write("controller_verified", requested=requested, actual=actual, parameters=parameters)
    if actual != requested:
        raise TrialBlocked(f"requested controller {requested!r} but running node reports {actual!r}; the launch selector is not effective")
    tolerance = parameters.get(prefix + requested + "_controller.goal_distance_tolerance")
    if not isinstance(tolerance, (int, float)) or not math.isfinite(tolerance) or tolerance <= 0:
        raise TrialBlocked("running controller has no valid positive goal_distance_tolerance")
    probe.actual_goal_tolerance = float(tolerance)
    probe.log.write("effective_arrival_threshold", distance_m=probe.actual_goal_tolerance,
                    source="running controller goal_distance_tolerance")


# 检查名义起终点之间胶囊形走廊，确保没有既有致命或未知单元干扰试验。
def preflight_corridor(probe, start, goal, half_width):
    width, height, resolution, origin_x, origin_y, data, kind = probe.costmap
    if resolution <= 0 or len(data) != width * height:
        raise TrialBlocked("invalid costmap metadata")
    # Grid centres within a capsule around the nominal route must be observed free.
    ix0 = math.floor((min(start[0], goal[0]) - half_width - origin_x) / resolution)
    ix1 = math.ceil((max(start[0], goal[0]) + half_width - origin_x) / resolution)
    iy0 = math.floor((min(start[1], goal[1]) - half_width - origin_y) / resolution)
    iy1 = math.ceil((max(start[1], goal[1]) + half_width - origin_y) / resolution)
    if min(ix0, iy0) < 0 or ix1 >= width or iy1 >= height:
        raise TrialBlocked("test corridor exceeds the observed costmap")
    failures = []
    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            point = (origin_x + (ix + 0.5) * resolution, origin_y + (iy + 0.5) * resolution)
            if point_to_polyline_distance(point, [start, goal]) > half_width:
                continue
            cost = data[iy * width + ix]
            if (kind == "raw" and cost >= 254) or (kind == "occupancy" and (cost < 0 or cost >= 99)):
                failures.append((point, cost))
                if len(failures) >= 5:
                    raise TrialBlocked(f"existing world geometry or unknown cells interfere with the test corridor: {failures}")
    if failures:
        raise TrialBlocked(f"test corridor contains non-free cells: {failures}")
    probe.log.write("environment_preflight_passed", start=start, goal=goal, half_width_m=half_width,
                    costmap_topic=probe.costmap_topic)


# 在自有仿真内放置初始船位并生成场景障碍，要求新地图确实观测到它们。
def setup_world(probe, config, scenario, trial_id, process):
    empty = probe.types["Empty"]
    call_service(probe, empty, ["/reset_world", "/gazebo/reset_world"], empty.Request(), optional=True)
    kind = probe.types["SetEntityState"]
    request = kind.Request()
    request.state.name = probe.namespace.strip("/")
    request.state.reference_frame = "world"
    request.state.pose.position.x, request.state.pose.position.y = config["start"][:2]
    request.state.pose.position.z = 0.0
    request.state.pose.orientation.z = math.sin(config["start"][2] / 2)
    request.state.pose.orientation.w = math.cos(config["start"][2] / 2)
    response = call_service(probe, kind, ["/gazebo/set_entity_state", "/set_entity_state"], request)
    if not response.success:
        raise TrialBlocked("Gazebo could not reset the vessel state")
    settled_since = time.monotonic()
    if not wait_until(probe, lambda: probe.pose is not None and probe.odom_wall > settled_since and
                      math.hypot(probe.pose[0] - config["start"][0], probe.pose[1] - config["start"][1]) < 0.5,
                      10, process):
        raise TrialBlocked("reset vessel pose was not observed")
    # Wait for a fresh post-reset map, then prove the corridor is clear before spawn.
    if not wait_until(probe, lambda: probe.costmap_wall is not None and probe.costmap_wall > settled_since + 1,
                      15, process):
        raise TrialBlocked("fresh costmap was not received after reset")
    preflight_corridor(probe, config["start"], scenario["goal"], config["thresholds"]["preflight_corridor_half_width_m"])
    for obstacle in scenario["obstacles"]:
        spawn = probe.types["SpawnEntity"]
        request = spawn.Request()
        request.name = "acceptance_" + trial_id + "_" + obstacle["name"]
        request.reference_frame = "world"
        request.initial_pose.position.x, request.initial_pose.position.y = obstacle["x"], obstacle["y"]
        request.initial_pose.position.z = obstacle["height"] / 2 - 1.0
        request.initial_pose.orientation.w = 1.0
        request.xml = f'''<sdf version="1.6"><model name="{request.name}"><static>true</static><link name="body">
<collision name="collision"><geometry><cylinder><radius>{obstacle['radius']}</radius><length>{obstacle['height']}</length></cylinder></geometry></collision>
<visual name="visual"><geometry><cylinder><radius>{obstacle['radius']}</radius><length>{obstacle['height']}</length></cylinder></geometry></visual>
</link></model></sdf>'''
        response = call_service(probe, spawn, ["/spawn_entity", "/gazebo/spawn_entity"], request)
        if not response.success:
            raise TrialBlocked("obstacle spawn failed: " + response.status_message)
        probe.log.write("obstacle_spawned", entity_name=request.name, geometry=obstacle)
    if scenario["obstacles"]:
        spawned_at = time.monotonic()
        if not wait_until(probe, lambda: probe.costmap_wall > spawned_at + 2, 15, process):
            raise TrialBlocked("no fresh costmap after obstacle spawn")
        width, height, resolution, ox, oy, data, kind = probe.costmap
        for obstacle in scenario["obstacles"]:
            ix, iy = int((obstacle["x"] - ox) / resolution), int((obstacle["y"] - oy) / resolution)
            radius_cells = max(1, math.ceil(obstacle["radius"] / resolution) + 1)
            costs = [data[y * width + x] for y in range(max(0, iy - radius_cells), min(height, iy + radius_cells + 1))
                     for x in range(max(0, ix - radius_cells), min(width, ix + radius_cells + 1))]
            if not any(c >= 254 and c != 255 if kind == "raw" else c >= 99 for c in costs):
                raise TrialBlocked("spawned cylinder was not observed as occupied in costmap")


# 采集基础导航或目标替换/取消场景；几何越界立即停止采集并留下快照。
def navigation_trial(args, config, workspace, mode, controller, scenario, repeat, fault=None):
    trial_id = f"{mode}_{controller}_{fault or scenario['name']}_{repeat:02d}"
    directory = args.output / trial_id
    directory.mkdir(parents=True, exist_ok=False)
    log = Jsonl(directory / "events.jsonl")
    log.write("trial_started", mode=mode, controller=controller, scenario=scenario,
              repeat=repeat, thresholds=config["thresholds"], footprint=config["footprint"])
    probe, process = None, None
    acquisition_closed = False
    outcome = {"trial": trial_id, "mode": mode, "controller": controller, "scenario": scenario["name"],
               "fault": fault, "repeat": repeat, "result": "not_run", "passed": False}
    try:
        env = dict(os.environ, GAZEBO_MASTER_URI=f"http://127.0.0.1:{free_port()}")
        launch_file = args.baseline_launch_file if mode == "cpp" else args.launch_file
        command = ["ros2", "launch", "usv_mission", launch_file, "world:=ocean", "gui:=false", "rviz:=false",
                   f"controller_name:={controller}", "robot_namespace:=otter", "robot_x:=23.0", "robot_y:=-10.0", "robot_yaw:=0.0"]
        if args.controller_config is not None:
            digest = hashlib.sha256(args.controller_config.read_bytes()).hexdigest()
            if digest != args.controller_config_sha256:
                raise TrialBlocked("controller configuration changed after the manifest was created")
            command.append("controller_config:=" + str(args.controller_config))
            log.write("controller_config_input", path=str(args.controller_config), sha256=digest)
        probe = make_probe("otter", log)
        wait_until(probe, lambda: False, 1.0)
        if probe.count_publishers("/otter/odom") > 0:
            raise TrialBlocked("pre-existing /otter/odom found in this ROS domain; refusing to mutate another simulation")
        process = OwnedProcess(workspace, command, directory / "launch.log", env)
        if not wait_until(probe, lambda: probe.pose is not None and probe.scan_wall is not None and probe.costmap is not None
                          and probe.goal_pub.get_subscription_count() >= 2, args.startup_timeout, process,
                          lambda: probe.discover_costmap(args.costmap_topic)):
            raise TrialBlocked("odom, scan, discovered costmap, or navigation goal subscriber did not become ready")
        activate_graph_guard(probe, log)
        apply_start_parameters(probe, args.parameter_overrides)
        verify_controller(probe, controller)
        setup_world(probe, config, scenario, trial_id, process)
        probe.odom_history.clear()
        probe.commands.clear()
        probe.command_wall.clear()
        probe.command_gaps.clear()
        probe.command_state_age.clear()
        probe.publish_goal(scenario["goal"])
        goal = scenario["goal"]
        started, sim_started = time.monotonic(), probe.sim_time()
        threshold = {**config["thresholds"], "arrival_distance_m": probe.actual_goal_tolerance}
        cross_track, live_cross_track = [], []
        min_clearance = None
        valid_commands = True
        arrived, dwell_since, changed, cancelled = False, None, False, False
        fault_action_wall = None
        fault_zero_latency = None
        nonzero_after_cancel_deadline = False
        previous_pose = None
        history_index = 0
        next_sample = started
        safety_stop = None
        while time.monotonic() - started < threshold["max_duration_wall_s"] and probe.sim_time() - sim_started < threshold["max_duration_sim_s"]:
            process.check()
            probe.types["rclpy"].spin_once(probe, timeout_sec=0.02)
            probe.graph_guard.check()
            now = time.monotonic()
            if args.stop_on_reference_failure and probe.status and (
                probe.status.get("state") == "NO_PATH" or
                ("map" in probe.status.get("reason", "").lower() and "invalid" in probe.status.get("reason", "").lower())
            ):
                if not probe.failure_snapshot_active:
                    probe.capture_inputs("navigation_failure_snapshot")
                outcome.update(result="diagnostic_stopped", reason=probe.status.get("reason", "reference failed"))
                break
            if fault == "goal_change" and not changed and now - started >= 4 and probe.first_command_latency is not None:
                replacement = next((s for s in config["scenarios"] if s["name"] == "turn"), None)
                if replacement is None:
                    raise TrialBlocked("goal_change requires a turn goal in the scenario file")
                goal = list(replacement["goal"])
                probe.publish_goal(goal)
                changed, fault_action_wall = True, now
                log.write("fault_goal_replacement", goal=goal)
            if fault == "cancel" and not cancelled and now - started >= 4 and probe.first_command_latency is not None:
                kind = probe.types["Trigger"]
                response = call_service(probe, kind, ["/otter/cancel_navigation"], kind.Request(), optional=True)
                if response is None:
                    outcome.update(result="unsupported", reason="cancel service is absent in this implementation")
                    break
                if not response.success:
                    raise TrialBlocked("cancel request rejected: " + response.message)
                cancelled, fault_action_wall = True, now
                log.write("fault_cancel_accepted", response=response.message)
            if cancelled:
                if probe.zero_received_since(fault_action_wall, threshold["zero_command_epsilon"]) and fault_zero_latency is None:
                    fault_zero_latency = max(probe.command_wall.values()) - fault_action_wall
                if now - fault_action_wall >= threshold["cancel_zero_deadline_wall_s"] and any(
                        abs(value) > threshold["zero_command_epsilon"] for value in probe.commands.values()):
                    nonzero_after_cancel_deadline = True
                if now - fault_action_wall >= threshold["cancel_zero_deadline_wall_s"] + 1.0:
                    passed = fault_zero_latency is not None and fault_zero_latency <= threshold["cancel_zero_deadline_wall_s"] and not nonzero_after_cancel_deadline and probe.zero_received_since(fault_action_wall, threshold["zero_command_epsilon"])
                    outcome.update(result="completed", passed=passed, zero_command_latency_wall_s=fault_zero_latency,
                                   nonzero_after_deadline=nonzero_after_cancel_deadline)
                    break
            while history_index < len(probe.odom_history):
                _, pose = probe.odom_history[history_index]
                history_index += 1
                if previous_pose is not None:
                    for obstacle in scenario["obstacles"]:
                        clearance = swept_clearance(previous_pose, pose, obstacle,
                                                    config["footprint"]["length_m"], config["footprint"]["width_m"])
                        min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                previous_pose = pose
            safety_stop = supervision_stop_reason(min_clearance, None, threshold["maximum_cross_track_m"])
            if safety_stop is not None:
                log.write("supervision_boundary_stop", reason=safety_stop, pose=probe.pose,
                          sim_time_s=probe.sim_time(), minimum_footprint_clearance_m=min_clearance)
                outcome.update(result="safety_stopped", reason=safety_stop, safety_stopped=True)
                probe.capture_inputs("supervision_boundary_snapshot")
                break
            if now < next_sample or probe.pose is None:
                continue
            next_sample = now + 0.1
            fixed_error = point_to_polyline_distance(probe.pose, probe.initial_path)
            live_error = point_to_polyline_distance(probe.pose, probe.path)
            if fixed_error is not None:
                cross_track.append(fixed_error)
            if live_error is not None:
                live_cross_track.append(live_error)
            valid_commands &= all(math.isfinite(value) and abs(value) <= 1 for value in probe.commands.values())
            distance = math.hypot(probe.pose[0] - goal[0], probe.pose[1] - goal[1])
            speed = math.hypot(*probe.velocity[:2])
            in_goal = distance <= threshold["arrival_distance_m"] and speed <= threshold["arrival_speed_mps"]
            dwell_since = now if in_goal and dwell_since is None else dwell_since if in_goal else None
            arrived = dwell_since is not None and now - dwell_since >= threshold["arrival_dwell_wall_s"]
            log.write("sample", sim_time_s=probe.sim_time(), pose=probe.pose, body_velocity=probe.velocity,
                      command=probe.commands, goal_distance_m=distance, cross_track_initial_m=fixed_error,
                      cross_track_live_m=live_error, minimum_footprint_clearance_m=min_clearance,
                      arrived=arrived, status=probe.status)
            safety_stop = supervision_stop_reason(min_clearance, fixed_error, threshold["maximum_cross_track_m"])
            if safety_stop is not None:
                log.write("supervision_boundary_stop", reason=safety_stop, pose=probe.pose,
                          sim_time_s=probe.sim_time(), cross_track_initial_m=fixed_error)
                outcome.update(result="safety_stopped", reason=safety_stop, safety_stopped=True)
                probe.capture_inputs("supervision_boundary_snapshot")
                break
            if arrived and fault != "cancel" and (fault != "goal_change" or changed):
                break
        # 正式采集结束点先落盘，后续统计、文件输出及清理耗时不应混入采集区间。
        # Close the acquisition interval before distribution calculations,
        # JSON serialization and process teardown add wall-clock overhead.
        log.write("acquisition_finished", sim_time_s=probe.sim_time(), cause=safety_stop or "loop_finished")
        acquisition_closed = True
        if outcome["result"] == "not_run":
            verdict = trial_verdict(arrived, cross_track, min_clearance, valid_commands, threshold, bool(scenario["obstacles"]))
            if not all(axis in probe.commands for axis in ("left", "right", "lateral")) or not probe.received_nonempty_trajectory:
                verdict["passed"] = False
                verdict["reasons"].append("missing_trajectory_or_thruster_topics")
            if fault == "goal_change" and not changed:
                verdict["passed"] = False
                verdict["reasons"].append("goal_change_was_not_exercised")
            if fault == "cancel":
                verdict["passed"] = False
                verdict["reasons"].append("cancel_was_not_exercised")
            outcome.update(result="completed", **verdict)
        outcome.update(arrived=arrived, duration_wall_s=time.monotonic() - started,
                       effective_arrival_distance_m=probe.actual_goal_tolerance,
                       duration_sim_s=probe.sim_time() - sim_started, costmap_topic=probe.costmap_topic,
                       cross_track_initial_m=distribution(cross_track), cross_track_live_m=distribution(live_cross_track),
                       minimum_footprint_clearance_m=min_clearance,
                       goal_to_first_path_wall_s=probe.first_path_latency, goal_to_first_nonzero_command_wall_s=probe.first_command_latency,
                       command_receipt_gap_ms=distribution(probe.command_gaps),
                       observed_odom_receipt_to_command_receipt_ms=distribution(probe.command_state_age),
                       path_header_age_sim_ms=distribution(probe.path_latency), status=probe.status,
                       mapping_status=probe.mapping_status)
    except KeyboardInterrupt:
        if not acquisition_closed and probe is not None and probe.goal_wall is not None:
            log.write("acquisition_finished", sim_time_s=probe.sim_time(), cause="interrupted")
        outcome.update(result="interrupted", reason="user interrupted this live trial")
        raise
    except (TrialBlocked, RuntimeError, OSError) as error:
        if not acquisition_closed and probe is not None and probe.goal_wall is not None:
            log.write("acquisition_finished", sim_time_s=probe.sim_time(), cause="blocked")
        outcome.update(result="blocked", reason=str(error))
    finally:
        if probe is not None:
            guard = probe.graph_guard
            outcome["control_ownership_graph"] = (guard.evidence() if guard is not None else
                {"requirements": GRAPH_REQUIREMENTS, "observation_count": 0,
                 "observations": [], "passed": False})
            if not outcome["control_ownership_graph"]["passed"]:
                outcome["passed"] = False
        if process:
            process.close()
        if probe:
            probe.destroy_node()
        log.write("trial_finished", **outcome)
        log.close()
        (directory / "summary.json").write_text(json.dumps(sanitized(outcome), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return outcome


# 用受控 ROS 发布器维持新鲜地图并停止里程计，独立测量陈旧状态触发的零推力延迟。
def odom_dropout_trial(args, config):
    """Controlled sensor publishers exercise stale-odom stopping without Gazebo."""
    trial_id = "python_pid_odom_dropout"
    directory = args.output / trial_id
    directory.mkdir(parents=True, exist_ok=False)
    log = Jsonl(directory / "events.jsonl")
    probe, process = None, None
    outcome = {"trial": trial_id, "mode": "python", "fault": "odom_dropout", "passed": False, "result": "not_run"}
    try:
        namespace = "acceptance_smoke_" + uuid.uuid4().hex[:8]
        threshold = config["thresholds"]
        command = ["ros2", "run", "usv_mission", "mission_node", "--ros-args", "-r", f"__ns:=/{namespace}",
                   "-p", "use_sim_time:=false", "-p", f"dynamics.ns:={namespace}",
                   "-p", f"navigation.state_timeout:={threshold['odom_timeout_s']}",
                   "-p", "navigation.map_timeout:=2.0", "-p", "navigation.trajectory_timeout:=1.0",
                   "-p", "usv_simulation.controller.controller_name:=pid"]
        probe = make_probe(namespace, log, use_sim_time=False)
        process = OwnedProcess(args.workspace, command, directory / "launch.log", dict(os.environ))
        odom_pub = probe.create_publisher(probe.types["Odometry"], probe.namespace + "/odom", 10)
        qos = probe.types["QoSProfile"](depth=1, durability=probe.types["DurabilityPolicy"].TRANSIENT_LOCAL,
                                        reliability=probe.types["ReliabilityPolicy"].RELIABLE)
        map_pub = probe.create_publisher(probe.types["Costmap"], probe.namespace + "/costmap_raw", qos)
        publish_odom = True
        next_publish = 0.0

        def publish_inputs():
            nonlocal next_publish
            now = time.monotonic()
            if now < next_publish:
                return
            next_publish = now + 0.05
            stamp = probe.get_clock().now().to_msg()
            costmap = probe.types["Costmap"]()
            costmap.header.frame_id = namespace + "/map"
            costmap.header.stamp = stamp
            costmap.metadata.resolution = 0.5
            costmap.metadata.size_x = costmap.metadata.size_y = 128
            costmap.metadata.origin.orientation.w = 1.0
            costmap.metadata.map_load_time = stamp
            costmap.metadata.update_time = stamp
            costmap.data = [0] * (128 * 128)
            map_pub.publish(costmap)
            if publish_odom:
                odom = probe.types["Odometry"]()
                odom.header.stamp = stamp
                odom.header.frame_id = namespace + "/map"
                odom.child_frame_id = namespace + "/base_link"
                odom.pose.pose.position.x = odom.pose.pose.position.y = 10.0
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)

        if not wait_until(probe, lambda: odom_pub.get_subscription_count() >= 2 and map_pub.get_subscription_count() >= 1
                          and probe.goal_pub.get_subscription_count() >= 2, 30, process, publish_inputs):
            raise TrialBlocked("controlled smoke node did not subscribe to inputs")
        wait_until(probe, lambda: False, 1.0, process, publish_inputs)
        activate_graph_guard(probe, log)
        probe.publish_goal([20.0, 10.0, 0.0])
        if not wait_until(probe, lambda: probe.first_command_latency is not None, 20, process, publish_inputs):
            raise TrialBlocked("smoke node never produced a nonzero command before odom dropout")
        # 故障注入只停里程计；地图继续更新，隔离状态超时这一原因。
        publish_odom = False
        dropped_at = time.monotonic()
        log.write("fault_odom_publisher_stopped", state_timeout_s=threshold["odom_timeout_s"])
        zero_at = None
        nonzero_after_deadline = False
        deadline = threshold["odom_zero_deadline_wall_s"]
        while time.monotonic() - dropped_at < deadline + 1.0:
            process.check()
            probe.types["rclpy"].spin_once(probe, timeout_sec=0.01)
            probe.graph_guard.check()
            publish_inputs()  # map remains fresh while odometry stops.
            now = time.monotonic()
            if probe.zero_received_since(dropped_at, threshold["zero_command_epsilon"]) and zero_at is None:
                zero_at = now
            if now - dropped_at >= deadline and any(abs(v) > threshold["zero_command_epsilon"] for v in probe.commands.values()):
                nonzero_after_deadline = True
        latency = None if zero_at is None else zero_at - dropped_at
        outcome.update(result="completed", passed=latency is not None and latency <= deadline and not nonzero_after_deadline,
                       zero_command_latency_wall_s=latency, nonzero_after_deadline=nonzero_after_deadline,
                       tested_state_timeout_s=threshold["odom_timeout_s"], deadline_wall_s=deadline, status=probe.status)
    except KeyboardInterrupt:
        outcome.update(result="interrupted", reason="user interrupted this smoke trial")
        raise
    except (TrialBlocked, RuntimeError, OSError) as error:
        outcome.update(result="blocked", reason=str(error))
    finally:
        if probe is not None:
            guard = probe.graph_guard
            outcome["control_ownership_graph"] = (guard.evidence() if guard is not None else
                {"requirements": GRAPH_REQUIREMENTS, "observation_count": 0,
                 "observations": [], "passed": False})
            if not outcome["control_ownership_graph"]["passed"]:
                outcome["passed"] = False
        if process:
            process.close()
        if probe:
            probe.destroy_node()
        log.write("trial_finished", **outcome)
        log.close()
        (directory / "summary.json").write_text(json.dumps(sanitized(outcome), indent=2, allow_nan=False), encoding="utf-8")
    return outcome


# 解析工作区、场景、控制器及参数覆盖，拒绝重复覆盖键和非法 ROS 域。
def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=find_source_root(), help="built core task ROS workspace")
    parser.add_argument("--baseline-workspace", type=Path)
    parser.add_argument("--mode", choices=["python"], default="python")
    parser.add_argument("--controllers", nargs="+", choices=["pid", "lqr"], default=["pid", "lqr"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenario", nargs="+", choices=["straight", "turn", "single_obstacle"])
    parser.add_argument("--scenarios-file", type=Path, default=Path(__file__).with_name("scenarios.json"))
    parser.add_argument("--launch-file", default="mission_sim.launch.py")
    parser.add_argument("--baseline-launch-file", default="main.launch.py")
    parser.add_argument("--controller-config", type=Path,
                        help="project controller YAML passed as an absolute controller_config launch argument, hashed before trials")
    parser.add_argument("--costmap-topic", help="optional absolute override; otherwise discover message type and topic")
    parser.add_argument("--parameter", action="append", default=[], metavar="NAME=JSON",
                        help="set and verify a parameter before each trial goal; repeat for multiple values")
    parser.add_argument("--stop-on-reference-failure", action="store_true",
                        help="diagnostic only: capture and stop on first NO_PATH/map-invalid status")
    parser.add_argument("--output", type=Path, required=True, help="new output directory, never an existing result directory")
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--domain-id", type=int, default=182)
    parser.add_argument("--faults", action="store_true", help="also run goal-change, cancel and controlled odom-dropout cases")
    parser.add_argument("--only-faults", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="write planned matrix only; does not import ROS or start Gazebo")
    args = parser.parse_args(argv)
    args.parameter_overrides = {}
    for assignment in args.parameter:
        try:
            name, value = assignment.split("=", 1)
            if not name or name in args.parameter_overrides:
                raise ValueError("empty or duplicate parameter name")
            args.parameter_overrides[name] = json.loads(value)
        except (ValueError, json.JSONDecodeError) as error:
            parser.error(f"invalid --parameter {assignment!r}: {error}")
    if args.repeats <= 0:
        parser.error("--repeats must be positive")
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in [0, 232]")
    if args.mode in ("cpp", "both") and args.baseline_workspace is None:
        parser.error("C++ mode requires --baseline-workspace")
    return args


# 保存试验计划和配置指纹，运行选定导航/故障组合并汇总真实采集结果。
def main(argv=None):
    args = arguments(argv)
    args.workspace = args.workspace.resolve()
    if args.baseline_workspace:
        args.baseline_workspace = args.baseline_workspace.resolve()
    args.controller_config_sha256 = None
    if args.controller_config is not None:
        args.controller_config = args.controller_config.resolve()
        if not args.controller_config.is_file():
            raise SystemExit(f"controller configuration does not exist: {args.controller_config}")
        args.controller_config_sha256 = hashlib.sha256(args.controller_config.read_bytes()).hexdigest()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    config = json.loads(args.scenarios_file.read_text(encoding="utf-8"))
    modes = ["python", "cpp"] if args.mode == "both" else [args.mode]
    scenarios = [s for s in config["scenarios"] if not args.scenario or s["name"] in args.scenario]
    matrix = [] if args.only_faults else [{"mode": mode, "controller": controller, "scenario": scene["name"], "repeat": repeat}
             for mode in modes for controller in args.controllers for scene in scenarios for repeat in range(1, args.repeats + 1)]
    manifest = {"schema_version": 1, "config": config, "matrix": matrix,
                "launch_package": "usv_mission", "control_ownership_graph_requirements": GRAPH_REQUIREMENTS,
                "workspace": str(args.workspace), "baseline_workspace": str(args.baseline_workspace) if args.baseline_workspace else None,
                "faults_requested": args.faults or args.only_faults, "mode": "dry_run" if args.dry_run else "live",
                "created_unix_s": time.time(), "domain_id": args.domain_id,
                "pre_goal_parameter_overrides": args.parameter_overrides,
                "diagnostic_stop_on_reference_failure": args.stop_on_reference_failure,
                "controller_config": str(args.controller_config) if args.controller_config is not None else None,
                "controller_config_sha256": args.controller_config_sha256,
                "latency_definition": "Receive gaps and observed callback age are wall-clock diagnostics, not measured algorithm WCET or end-to-end source latency. Header ages use ROS simulation time."}
    for label, workspace in [("python", args.workspace), ("cpp", args.baseline_workspace)]:
        if workspace is not None:
            try:
                revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=workspace, capture_output=True, text=True, timeout=5)
                manifest[label + "_git_commit"] = revision.stdout.strip() if revision.returncode == 0 else None
            except (OSError, subprocess.TimeoutExpired):
                manifest[label + "_git_commit"] = None
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.dry_run:
        print(json.dumps({"result": "not_run", "planned_navigation_trials": len(matrix), "manifest": str(args.output / "manifest.json")}, ensure_ascii=False))
        return 0
    if os.name != "posix":
        raise SystemExit("Live trials require WSL/Linux with ROS 2 Humble sourced; use --dry-run on Windows.")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    # Python owns SIGINT so a manual pause reaches our finally blocks before
    # ROS destroys the context; default rclpy handlers can shut it down first.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    results = []
    run_finished = False
    interrupted = False
    safety_stopped = False
    try:
        for item in matrix:
            workspace = args.workspace if item["mode"] == "python" else args.baseline_workspace
            scene = next(s for s in scenarios if s["name"] == item["scenario"])
            outcome = navigation_trial(args, config, workspace, item["mode"], item["controller"], scene, item["repeat"])
            results.append(outcome)
            print(json.dumps(sanitized(outcome), ensure_ascii=False), flush=True)
            if outcome.get("safety_stopped"):
                safety_stopped = True
                break
        if (args.faults or args.only_faults) and not safety_stopped:
            scene = next(s for s in config["scenarios"] if s["name"] == "straight")
            for mode in modes:
                workspace = args.workspace if mode == "python" else args.baseline_workspace
                for fault in ("goal_change", "cancel"):
                    results.append(navigation_trial(args, config, workspace, mode, "pid", scene, 1, fault=fault))
            if "python" in modes:
                results.append(odom_dropout_trial(args, config))
        run_finished = not safety_stopped
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        manifest["control_ownership_graph_observations"] = [
            {"trial": item["trial"], **item.get("control_ownership_graph", {"passed": False})}
            for item in results]
        (args.output / "manifest.json").write_text(json.dumps(sanitized(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
        summary = {"results": results, "completed": sum(r["result"] == "completed" for r in results),
                   "passed": sum(r["passed"] for r in results), "run_finished": run_finished,
                   "interrupted": interrupted, "safety_stopped": safety_stopped,
                   "all_passed": run_finished and bool(results) and all(r["passed"] for r in results)}
        (args.output / "summary.json").write_text(json.dumps(sanitized(summary), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return 130 if interrupted else 0 if run_finished and results and all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
