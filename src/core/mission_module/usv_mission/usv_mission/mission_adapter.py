# 任务与 ROS/控制器之间的适配层：把地理请求变成本地状态机，并管理导航、保持及安全制动交接。
"""Mission ROS services and controller handover owned by the task node."""
from dataclasses import replace
import math

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from usv_mission_interfaces.srv import SetMission

from common.geodesy import GeoReference, heading_to_yaw
from common.models import AlgorithmStatus, ControlCommand
from common.dynamics import clear_control, braking_command, braking_candidates
from controller.controller_factory import create_controller
from controller.station_keeping_controller import StationKeepingController
from .configuration import C, T, section
from .missions import MissionSpec, MissionManager
from .runtime import PlanningRequest, fresh, readiness


# 任务服务、标记可视化和任务控制均依附同一个 MissionNode。
class MissionAdapter:
    # 创建 SetMission 服务及持久化任务标记发布器，初始没有活动任务。
    def __init__(self, node):
        self.node = node
        self.manager = None
        self.intent = None
        self.signature = None
        self.sequence = 0
        self.nav_controller = self.holding = None
        self.parameters = None
        self.safety_reason = ''
        self.marker_pub = node.create_publisher(
            MarkerArray, 'mission_markers', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_service(SetMission, 'set_mission', self.set_mission)

    # 以状态机实例是否存在判断任务是否活动，停止意图仍可能属于活动任务。
    @property
    def active(self):
        return self.manager is not None

    # 清除活动任务和控制器积累状态，同时删除 RViz 任务标记。
    def clear(self):
        self.manager = self.intent = self.signature = None
        if self.holding is not None:
            self.holding.reset()
        if self.nav_controller is not None:
            self.nav_controller.reset()
        marker = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    # 先完成任务、地理转换和控制器构造；验证全部成功且发布权仍有效后才替换旧任务。
    def set_mission(self, request, response):
        n = self.node
        if not n.command_authority_ready():
            response.accepted, response.mission_id, response.reason = False, self.sequence, n.reason
            return response
        try:
            if not n.config.get('mission.geo.enabled', False):
                raise ValueError('Active world has no supported WGS84 / ENU geographic reference')
            names = ('mode', 'latitude', 'longitude', 'heading_deg', 'speed', 'position_tolerance',
                     'watch_radius', 'orbit_radius', 'clockwise', 'laps', 'duration')
            spec = MissionSpec(**{key: getattr(request, key) for key in names})
            spec.validate(n.config)
            geo = GeoReference(n.config['mission.geo.latitude'], n.config['mission.geo.longitude'],
                               n.config['mission.geo.altitude'])
            x, y, _ = geo.to_local(spec.latitude, spec.longitude)
            # 地理坐标转换为 ENU 米制位置；真北为零、顺时针为正的航向转换为 ROS 平面 yaw。
            target = (x, y, heading_to_yaw(spec.heading_deg))
            candidate = MissionManager(spec, target, n.now_seconds(), self.sequence + 1, dict(n.config))
            parameters = dict(n.config)
            parameters['dynamics.cruising_speed'] = spec.speed
            for kind in ('pid', 'lqr'):
                parameters[C + kind + '_controller.goal_distance_tolerance'] = spec.position_tolerance
                parameters[C + kind + '_controller.goal_speed_tolerance'] = .1
            parameters[T + 'los.goal_tolerance'] = min(spec.position_tolerance, parameters[T + 'los.goal_tolerance'])
            nav = create_controller(n.controller_name, replace(n.model, cruising_speed=spec.speed),
                                    section(parameters, C + n.controller_name + '_controller.'))
            holding = StationKeepingController(n.model, {
                'position_tolerance': spec.position_tolerance,
                'heading_tolerance_deg': n.config.get('mission.heading_tolerance_deg', 5.0)})
        except (ValueError, TypeError, ArithmeticError) as error:
            response.accepted, response.mission_id, response.reason = False, self.sequence, str(error)
            return response
        # Construction and validation above are side-effect free. Only an
        # accepted request can cancel the current goal or increment the ID.
        if not n.command_authority_ready():
            response.accepted, response.mission_id, response.reason = False, self.sequence, n.reason
            return response
        n.reset_task('MISSION_ACCEPTED')
        # The reset publishes a handover zero through the same output gate. If
        # discovery reports a conflict there, do not install a new task anyway.
        if not n.command_authority_ready():
            response.accepted, response.mission_id, response.reason = False, self.sequence, n.reason
            return response
        self.sequence += 1
        self.manager, self.parameters = candidate, parameters
        self.nav_controller, self.holding = nav, holding
        self.safety_reason = ''
        response.accepted, response.mission_id, response.reason = True, self.sequence, 'Mission accepted'
        self.publish_markers()
        return response

    # 任务控制前检查状态和地图新鲜度及 frame_id 一致性。
    def input_state(self, now):
        n = self.node
        if n.state is None or not fresh(n.state.stamp, now, n.config['navigation.state_timeout']):
            return 'WAITING_STATE'
        if n.grid is None or not fresh(n.grid.stamp, now, n.config['navigation.map_timeout']):
            return 'WAITING_MAP'
        if n.state.frame_id != n.grid.frame_id:
            return 'FRAME_MISMATCH'
        return ''

    # 推进状态机；意图签名变化时提升参考版本、取消待办并重置控制器。
    def prepare(self, now):
        if not self.active:
            return
        n = self.node
        intent = self.manager.tick(n.state, n.grid, now)
        signature = intent.signature
        if signature != self.signature:
            n.version += 1
            n.worker.invalidate()
            n.trajectory = None
            n.route = np.empty((0, 3))
            self.nav_controller.reset()
            self.holding.reset()
            # 交接时继承真正已发布命令，避免保持控制器从虚构零值计算推力变化率。
            self.holding.observe_command(getattr(n, 'last_published_command', ControlCommand()))
            self.signature = signature
        self.intent = intent
        n.goal = intent.goal

    # 停止、保持和制动由控制侧处理；其余意图带任务专用快照提交规划进程。
    def planning(self, now):
        self.prepare(now)
        if self.input_state(now) or self.intent is None:
            return
        if self.intent.kind in ('stop', 'hold', 'brake'):
            return
        n = self.node
        config = dict(self.parameters)
        config.update({k: n.config[k] for k in n.config if k.startswith(T)})
        config[T + 'los.goal_tolerance'] = min(self.manager.spec.position_tolerance,
                                              config[T + 'los.goal_tolerance'])
        n.worker.offer(PlanningRequest(n.version, self.intent.goal, n.state, n.grid,
                                      n.route.copy(), config, mission_intent=self.intent))

    # 确认版本和最新地图安全性后交接参考；条件起步方案不能直接当作可执行轨迹。
    def consume_plan(self, result, now=None):
        n = self.node
        now = n.now_seconds() if now is None else now
        if result.version != n.version:
            n.counters['stale_results'] += 1
            return
        original_kind = self.intent.kind if self.intent else None
        expected_goal = result.selected_goal if result.selected_goal is not None else self.intent.goal
        n.reason = result.message
        n.counters['global_searches'] += int(result.searched)
        if result.status not in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED):
            self.manager.on_plan(result)
            self.prepare(now)
            n.trajectory = None
            n.mode = result.status.value.upper()
            return
        if not n.trajectory_clear(result.trajectory):
            n.trajectory = None
            n.counters['unsafe_results'] += 1
            n.reason = 'Mission reference conflicts with latest map'
            self.manager.on_plan(replace(result, status=AlgorithmStatus.NO_PATH, message=n.reason))
            self.prepare(now)
            return
        self.manager.on_plan(result)
        # Choosing a candidate establishes a new reference generation. Adopt
        # that generation BEFORE transferring the validated tangent/detour
        # route; otherwise prepare() would erase the selected route immediately.
        self.prepare(now)
        metadata = result.metadata or {}
        # 预测停止位置只证明后续可能可行；实测制动、倒退与转向完成后必须重新规划。
        if metadata.get('departure_requires_replan') or metadata.get('departure_pose') is not None:
            # This reference starts at an assumed stop/alignment, so it can
            # never be transferred as an executable reference, even when the
            # manager has already seen this goal/yaw and suppresses the prelude.
            n.route, n.trajectory = np.empty((0, 3)), None
            if self.intent.kind == 'escape':
                n.reason = 'Conditional departure requires a new safe candidate'
                self.manager.on_plan(replace(
                    result, status=AlgorithmStatus.NO_PATH, message=n.reason,
                    metadata={**metadata, 'map_stamp': n.grid.stamp}))
                self.prepare(now)
            n.mode = self.manager.phase
            return
        if self.intent.kind != original_kind or self.intent.kind in ('hold', 'stop'):
            return
        if original_kind != 'circle' and (self.intent.goal is None or expected_goal is None
                or not np.allclose(self.intent.goal, expected_goal, atol=1e-8, rtol=0)):
            return
        n.route, n.trajectory = result.route, result.trajectory
        from common.visualizer import make_path
        stamp = n.get_clock().now().to_msg()
        n.path_pub.publish(make_path(n.route, n.frame, stamp))
        n.trajectory_pub.publish(make_path(n.trajectory.points[:, [2, 3, 4]], n.frame, stamp))

    # 按候选次序选择已通过执行与停止扫掠校验的制动命令，全部失败则返回零命令。
    def safe_braking_command(self, dt):
        """Brake while a safety detour is pending, only with a clear swept hull."""
        n = self.node
        for command in braking_candidates(n.state, n.model):
            if clear_control(n.state, command, n.grid, n.model, dt,
                             length=n.config['navigation.footprint_length'],
                             width=n.config['navigation.footprint_width'],
                             margin=n.config.get('mission.safety_margin', .1)):
                return command
        return ControlCommand()

    # 按意图选择制动、停止、定点保持或导航控制，危险预测可再次触发安全抢占。
    def control(self, now, dt):
        n = self.node
        self.safety_reason = ''
        reason = self.input_state(now)
        if dt <= 0:
            reason = 'SIMULATION_PAUSED'
        if reason:
            if reason != 'SIMULATION_PAUSED':
                self.prepare(now)
            self.holding.reset()
            self.nav_controller.reset()
            n.mode = reason
            return ControlCommand()
        self.prepare(now)
        if not n.current_footprint_clear():
            self.holding.reset()
            n.mode = 'CURRENT_FOOTPRINT_BLOCKED'
            n.trajectory = None
            return ControlCommand()
        intent = self.intent
        n.mode = self.manager.phase
        if intent.kind == 'brake':
            return self.safe_braking_command(dt)
        if intent.kind == 'stop':
            if self.manager.phase == 'AVOIDANCE_BLOCKED':
                return self.safe_braking_command(dt)
            self.holding.observe_command(ControlCommand())
            return ControlCommand()
        if intent.kind == 'hold':
            command = self.holding.compute(n.state, intent.goal, dt)
            length, width = n.config['navigation.footprint_length'], n.config['navigation.footprint_width']
            options = dict(length=length, width=width, margin=n.config.get('mission.safety_margin', .1))
            if not command.is_valid() or command.status != AlgorithmStatus.SUCCESS:
                n.mode, n.reason = 'CONTROL_ERROR', command.message
                return ControlCommand()
            # 定点保持也会平移和旋转，因此每条命令必须通过完整船体运动及停止预测。
            if not clear_control(n.state, command, n.grid, n.model, dt, **options):
                command = braking_command(n.state, n.model)
                self.safety_reason = 'Holding command failed swept-hull prediction'
                if not clear_control(n.state, command, n.grid, n.model, dt, **options):
                    command = ControlCommand()
                n.mode = 'HOLD_CONTROL_BLOCKED'
                n.reason = self.safety_reason
                self.manager.request_escape(self.safety_reason)
            self.holding.observe_command(command)
            return command
        ready = readiness(now, n.state, n.grid, n.trajectory, n.config)
        if ready != 'NAVIGATING':
            n.mode = ready if ready != 'WAITING_REFERENCE' else self.manager.phase
            if intent.kind == 'escape':
                return self.safe_braking_command(dt)
            return ControlCommand()
        command = self.nav_controller.compute(n.state, n.trajectory, dt)
        if command.status == AlgorithmStatus.GOAL_REACHED:
            self.manager.on_arrival(now)
            return ControlCommand()
        if command.status == AlgorithmStatus.SETTLING:
            n.mode = 'SETTLING'
            return ControlCommand()
        if command.status != AlgorithmStatus.SUCCESS or not command.is_valid():
            n.mode, n.reason = 'CONTROL_ERROR', command.message
            return ControlCommand()
        if intent.kind == 'escape' and not clear_control(
                n.state, command, n.grid, n.model, dt,
                length=n.config['navigation.footprint_length'],
                width=n.config['navigation.footprint_width'],
                margin=n.config.get('mission.safety_margin', .1)):
            self.nav_controller.reset()
            self.safety_reason = 'Detour command failed execution and stopping swept-hull prediction'
            n.mode, n.reason = 'ESCAPE_CONTROL_BLOCKED', self.safety_reason
            return self.safe_braking_command(dt)
        return command

    # 合并状态机报告、保持控制阶段以及控制侧安全拒绝原因。
    def status(self):
        if not self.active:
            return None
        result = self.manager.status(self.node.state, self.node.now_seconds())
        result['control_phase'] = self.holding.phase if self.intent and self.intent.kind == 'hold' else None
        result['safety_reason'] = self.safety_reason
        return result

    # 在 map 中画原目标、观察/任务半径、目标朝向和当前阶段标签。
    def publish_markers(self):
        if not self.active:
            return
        n, m = self.node, self.manager
        spec = m.spec
        x, y, heading = m.target_pose
        markers = [Marker(action=Marker.DELETEALL)]

        def marker(kind, color):
            item = Marker()
            item.header.frame_id, item.header.stamp = n.frame, n.get_clock().now().to_msg()
            item.ns, item.id, item.type = 'mission', len(markers), kind
            item.pose.orientation.w = 1.
            item.color.r, item.color.g, item.color.b, item.color.a = (*color, 1.)
            markers.append(item)
            return item

        centre = marker(Marker.SPHERE, (.1, .8, .9))
        centre.pose.position.x, centre.pose.position.y = x, y
        centre.scale.x = centre.scale.y = centre.scale.z = .4
        for radius, color in ((spec.watch_radius, (.2, .6, 1.)),
                              (spec.orbit_radius if spec.mode == 'orbit' else spec.position_tolerance, (1., .6, .1))):
            ring = marker(Marker.LINE_STRIP, color)
            ring.scale.x = .08
            ring.points = [Point(x=x + radius * math.cos(a), y=y + radius * math.sin(a), z=.1)
                           for a in np.linspace(0., 2. * math.pi, 129)]
        arrow = marker(Marker.ARROW, (.9, .2, .3))
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .12, .3, .35
        arrow.points = [Point(x=x, y=y, z=.2),
                        Point(x=x + 3 * math.cos(heading), y=y + 3 * math.sin(heading), z=.2)]
        label = marker(Marker.TEXT_VIEW_FACING, (1., 1., 1.))
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 2.
        label.scale.z = .8
        label.text = f'{spec.mode}: {m.phase}'
        self.marker_pub.publish(MarkerArray(markers=markers))
