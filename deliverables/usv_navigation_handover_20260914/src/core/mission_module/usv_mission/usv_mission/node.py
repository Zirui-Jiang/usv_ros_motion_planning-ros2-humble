"""Task execution node; simulation is an external ROS state/actuator provider."""
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
from .configuration import DEFAULTS, HOT_PARAMETERS, C, T, resolve_parameters, section, validate_update
from .runtime import LatestPlanWorker, PlanningRequest, fresh, readiness
from .mission_adapter import MissionAdapter
from .control_authority import ControlAuthority


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return quaternion_yaw(q.x, q.y, q.z, q.w)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
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
        self.control_authority = ControlAuthority()
        self.last_published_command = ControlCommand()
        self.create_subscription(PoseStamped, 'goal', self._on_goal, 1)
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Costmap, self.config['navigation.costmap_topic'], self._on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, 'cancel_navigation', self._cancel)
        self.create_service(Trigger, 'export_parameters', self._export)
        self.mission = MissionAdapter(self)
        # A steady timer also emits zero commands while simulation time is paused.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.config[C + 'control_frequency'], self._control, clock=self.steady_clock)
        self.create_timer(1.0 / self.config['navigation.planning_frequency'], self._planning)
        self.get_logger().info('Effective navigation parameters: ' + json.dumps(self.config, sort_keys=True))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def command_authority_ready(self):
        """Gate the resolved outputs, including remappings and duplicate node names."""
        try:
            ready = self.control_authority.observe({
                publisher.topic_name: self.count_publishers(publisher.topic_name)
                for publisher in self.thrust_pubs})
        except Exception as error:
            ready = self.control_authority.unavailable(error)
        if not ready:
            self.mode = ('CONTROL_CONFLICT' if self.control_authority.conflicted
                         else 'WAITING_CONTROL_AUTHORITY')
            self.reason = self.control_authority.reason
            # Prevent both completed and in-flight results from becoming active.
            if self.control_authority.conflicted and not getattr(self, '_conflict_invalidated', False):
                self._conflict_invalidated = True
                self.version += 1
                self.worker.invalidate()
                self.trajectory = None
                self.route = np.empty((0, 3))
                self.counters['control_conflicts'] += 1
        elif self.mode == 'WAITING_CONTROL_AUTHORITY':
            self.mode = 'WAITING_REFERENCE' if self.goal is not None or self.mission.active else 'IDLE'
            self.reason = ''
        return ready

    def _validate_parameters(self, parameters):
        try:
            # Validation has no side effects: rclpy has not committed the values yet.
            current = {k: self.get_parameter(k).value for k in DEFAULTS}
            candidate = validate_update(current, {p.name: p.value for p in parameters})
            if hasattr(self, 'mission') and self.mission.active:
                self.mission.manager.spec.validate(candidate)
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def _apply_parameters(self):
        changes = {k: self.get_parameter(k).value for k in HOT_PARAMETERS
                   if self.get_parameter(k).value != self.config[k]}
        if not changes:
            return
        self.config = validate_update(self.config, changes)
        if self.controller_name == 'pid':
            self.controller.update_config(section(self.config, C + 'pid_controller.'))
            if self.mission.active:
                self.mission.parameters.update(changes)
                self.mission.nav_controller.update_config(
                    section(self.mission.parameters, C + 'pid_controller.'))
        if self.mission.active:
            self.mission.manager.config.update(changes)
        if any(k.startswith(T) for k in changes):
            self.version += 1
            self.worker.invalidate()
            self.trajectory = None
        self.counters['parameter_updates'] += 1

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

    def _on_odom(self, message):
        self.pending_odom = message
        self._consume_odom()

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
            if self.trajectory is not None and not self.trajectory_clear(self.trajectory):
                self.trajectory = None
                self.route = np.empty((0, 3))
                self.mode, self.reason = 'WAITING_REFERENCE', 'Latest map invalidated reference'
                self.counters['map_invalidations'] += 1
                self._publish_command(ControlCommand())
        except (ValueError, TypeError) as error:
            self.counters['invalid_costmap'] += 1
            self.reason = str(error)
        self.timings['costmap_conversion_ms'].append((time.perf_counter() - begun) * 1000)

    def trajectory_clear(self, trajectory):
        return self.current_footprint_clear() and footprint_path_is_clear(
            self.grid, trajectory.points[:, [2, 3, 4]], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    def current_footprint_clear(self):
        return self.state is not None and self.grid is not None and footprint_path_is_clear(
            self.grid, [self.state.pose], self.config['navigation.footprint_length'],
            self.config['navigation.footprint_width'])

    def reset_task(self, mode, reason='', clear_inputs=False):
        if hasattr(self, 'mission'):
            self.mission.clear()
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

    def _on_goal(self, message):
        if not self.command_authority_ready():
            return
        # A rejected replacement must not silently continue the previous mission.
        self.reset_task('WAITING_REFERENCE')
        if not self.command_authority_ready():
            return
        try:
            self.goal = self._pose(message.pose, message.header.frame_id, message.header.stamp)
            self._planning()
        except (ValueError, TransformException) as error:
            self.mode, self.reason = 'INVALID_GOAL', str(error)

    def _cancel(self, request, response):
        self.reset_task('CANCELLED')
        response.success = True
        response.message = ('Navigation cancelled; actuator output inhibited: ' + self.reason
                            if self.mode in ('CONTROL_CONFLICT', 'WAITING_CONTROL_AUTHORITY')
                            else 'Navigation cancelled; zero thrust active')
        return response

    def _export(self, request, response):
        values = {k: self.get_parameter(k).value for k in DEFAULTS}
        response.success = True
        response.message = yaml.safe_dump({self.get_fully_qualified_name(): {'ros__parameters': values}}, sort_keys=True)
        return response

    def _planning(self):
        if not self.command_authority_ready():
            return
        now = self.now_seconds()
        if self.mission.active:
            try:
                self.mission.planning(now)
            except Exception as error:
                self.trajectory = None
                self.mode, self.reason = 'PLANNING_ERROR', str(error)
                self.counters['worker_errors'] += 1
                self._publish_command(ControlCommand())
            return
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

    def _consume_plan(self, now=None):
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
        if self.mission.active:
            self.mission.consume_plan(result, now=now)
            return
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
        if not self.trajectory_clear(result.trajectory):
            self.trajectory = None
            self.route = np.empty((0, 3))
            self.mode, self.reason = 'WAITING_REFERENCE', 'Worker reference conflicts with latest map'
            self.counters['unsafe_results'] += 1
            return
        self.trajectory = result.trajectory
        self.trajectory_pub.publish(make_path(self.trajectory.points[:, [2, 3, 4]], self.frame, stamp))

    def _publish_command(self, command):
        # Never publish even zero against a second controller. This check also
        # covers reset/cancel/shutdown paths, not only the normal control timer.
        if not self.command_authority_ready():
            return
        for publisher, value in zip(self.thrust_pubs, (command.left, command.right, command.lateral)):
            publisher.publish(Float32(data=float(value)))
        self.last_published_command = command

    def _control(self):
        if not self.command_authority_ready():
            if time.perf_counter() - self.last_status_wall >= 1:
                self._publish_status()
            return
        begun = time.perf_counter()
        now = self.now_seconds()
        if now < self.last_clock - 1e-6:
            self.reset_task('CLOCK_RESET', 'Simulation clock moved backwards', clear_inputs=True)
            self.counters['clock_resets'] += 1
        dt = now - self.last_control
        self.last_clock = self.last_control = now
        command = ControlCommand()
        try:
            self._apply_parameters()
            self._consume_odom()
            self._consume_plan(now)
            if self.mission.active:
                command = self.mission.control(now, dt)
            elif self.goal is not None:
                ready = readiness(now, self.state, self.grid, self.trajectory, self.config)
                # Tracking errors can put the actual hull outside the safe
                # reference. Recheck it every control tick, even when planning
                # has not returned another trajectory yet.
                if ready in ('NAVIGATING', 'WAITING_REFERENCE') and not self.current_footprint_clear():
                    self.trajectory = None
                    self.route = np.empty((0, 3))
                    ready = 'CURRENT_FOOTPRINT_BLOCKED'
                    self.reason = 'Current vessel footprint intersects lethal or unknown map cells'
                    self.counters['current_footprint_blocks'] += 1
                if ready == 'NAVIGATING' and dt > 0:
                    self.mode = ready
                    command = self.controller.compute(self.state, self.trajectory, dt)
                    if command.status == AlgorithmStatus.GOAL_REACHED:
                        self.reset_task('GOAL_REACHED')
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

    def _publish_status(self):
        self.last_status_wall = time.perf_counter()
        self.mission.publish_markers()
        metrics = {name: {'count': len(samples), 'p50': float(np.percentile(samples, 50)),
                          'p95': float(np.percentile(samples, 95)), 'p99': float(np.percentile(samples, 99)),
                          'max': max(samples)} for name, samples in self.timings.items() if samples}
        self.counters['pending_replacements'] = self.worker.replaced
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.mode, reason=self.reason, version=self.version, stamp=self.now_seconds(),
            controller=self.controller_name, goal=self.goal, mission=self.mission.status(),
            control_authority=self.control_authority.status(),
            metrics=metrics, counters=dict(self.counters),
            process={'pid': os.getpid(), 'cpu_seconds': usage.ru_utime + usage.ru_stime,
                     'peak_rss_kib': usage.ru_maxrss, 'monotonic': time.monotonic()}))))

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
        node = MissionNode()
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
