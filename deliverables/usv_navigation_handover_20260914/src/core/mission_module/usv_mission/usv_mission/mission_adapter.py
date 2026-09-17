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


class MissionAdapter:
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

    @property
    def active(self):
        return self.manager is not None

    def clear(self):
        self.manager = self.intent = self.signature = None
        if self.holding is not None:
            self.holding.reset()
        if self.nav_controller is not None:
            self.nav_controller.reset()
        marker = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[marker]))

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

    def input_state(self, now):
        n = self.node
        if n.state is None or not fresh(n.state.stamp, now, n.config['navigation.state_timeout']):
            return 'WAITING_STATE'
        if n.grid is None or not fresh(n.grid.stamp, now, n.config['navigation.map_timeout']):
            return 'WAITING_MAP'
        if n.state.frame_id != n.grid.frame_id:
            return 'FRAME_MISMATCH'
        return ''

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
            self.holding.observe_command(getattr(n, 'last_published_command', ControlCommand()))
            self.signature = signature
        self.intent = intent
        n.goal = intent.goal

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

    def status(self):
        if not self.active:
            return None
        result = self.manager.status(self.node.state, self.node.now_seconds())
        result['control_phase'] = self.holding.phase if self.intent and self.intent.kind == 'hold' else None
        result['safety_reason'] = self.safety_reason
        return result

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
