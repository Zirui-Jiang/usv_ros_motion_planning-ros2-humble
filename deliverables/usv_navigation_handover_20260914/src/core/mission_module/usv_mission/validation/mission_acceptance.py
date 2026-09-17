#!/usr/bin/env python3
"""Actual ROS/Gazebo mission acceptance. Dry runs never report measured success.

Source the selected workspace before use. Each trial owns its launch process
group, a fresh ROS domain and Gazebo master port. Never resets the whole world.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

# These are sibling scripts, also importable in source-tree pytest runs.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from task_ros_acceptance import (Jsonl, OwnedProcess, TrialBlocked, call_service,
                            free_port, make_probe, parameter_value, sanitized, wait_until)
from task_metrics import (distribution, point_to_polyline_distance, rectangle_circle_clearance,
                     swept_clearance, wrap_angle)
from task_graph_guard import GRAPH_REQUIREMENTS, activate_graph_guard
from task_source_layout import find_source_root

SCENARIOS = ("station", "station_disturbance", "standby", "standby_obstacle",
             "orbit_cw", "orbit_ccw", "orbit_obstacle")
START = (23.0, -10.0, 0.0)
THRESHOLDS = {"position_m": 0.5, "heading_deg": 5.0, "joint_fraction": 0.95,
              "settle_position_m": 0.25, "settle_heading_deg": 3.0,
              "settle_speed_mps": 0.04, "settle_dwell_s": 5.0,
              "settle_timeout_s": 180.0, "window_s": 60.0,
              "radial_p95_m": 0.5, "clearance_m": 0.2, "max_odom_gap_s": 0.5,
              "max_phase_boundary_interval_s": 1.5, "max_lap_angle_tolerance_rad": 0.15,
              "centre_dwell_s": 1.0, "centre_speed_mps": 0.1,
              "impulse_service_delay_max_s": 2.0, "impulse_post_response_window_s": 0.5}


class TrialFailed(RuntimeError):
    pass


def scenario_config(name):
    if name not in SCENARIOS:
        raise ValueError("Unsupported mission scenario")
    if name.startswith("station"):
        return dict(mode="station_keep", target=(25.0, -8.0, math.pi/2), heading_deg=0.0,
                    speed=0.5, position_tolerance=0.5, watch_radius=4.0)
    if name.startswith("orbit"):
        return dict(mode="orbit", target=(23.0, -10.0, 0.0), heading_deg=90.0,
                    speed=0.5, position_tolerance=0.5, watch_radius=4.0,
                    orbit_radius=8.0, clockwise=name != "orbit_ccw", laps=1, duration=0.0)
    return dict(mode="standby", target=(25.0, -10.0, 0.0), heading_deg=90.0,
                speed=0.5, position_tolerance=0.5, watch_radius=3.0)


def sample_errors(pose, velocity, target):
    return {"position_error_m": math.hypot(pose[0]-target[0], pose[1]-target[1]),
            "heading_error_deg": abs(math.degrees(wrap_angle(pose[2]-target[2]))),
            "speed_mps": math.hypot(velocity[0], velocity[1])}


def fixed_window_metrics(samples, start, duration=60.0):
    """Score exactly the first fixed window, including all excursions."""
    selected = [s for s in samples if start <= s["sim_time_s"] <= start+duration+1e-8]
    times = [s["sim_time_s"] for s in selected]
    complete = bool(times) and times[0]-start <= 0.5 and start+duration-times[-1] <= 0.5
    complete &= all(0 < b-a <= THRESHOLDS["max_odom_gap_s"] for a, b in zip(times, times[1:]))
    joint = [s["position_error_m"] <= 0.5 and s["heading_error_deg"] <= 5 for s in selected]
    fraction = sum(joint)/len(joint) if joint else None
    return {"start_sim_s": start, "end_sim_s": start+duration, "sample_count": len(selected),
            "complete": complete, "joint_fraction": fraction,
            "position_error_m": distribution([s["position_error_m"] for s in selected]),
            "heading_error_deg": distribution([s["heading_error_deg"] for s in selected]),
            "passed": bool(complete and fraction is not None and fraction >= 0.95)}


class SettleDetector:
    def __init__(self, start):
        self.start, self.since, self.last, self.reached = start, None, None, None

    def add(self, sample):
        t = sample["sim_time_s"]
        if t < self.start or self.reached is not None:
            return
        if self.last is not None and not 0 < t-self.last <= 0.5:
            self.since = None
        self.last = t
        good = (sample["position_error_m"] <= 0.25 and sample["heading_error_deg"] <= 3
                and sample["speed_mps"] <= 0.04)
        self.since = (t if self.since is None else self.since) if good else None
        if self.since is not None and t-self.since >= 5-1e-8:
            self.reached = t


class DwellDetector:
    """An observed dwell, with missing/nonmonotonic odometry breaking continuity."""
    def __init__(self, predicate, duration=1.0):
        self.predicate, self.duration = predicate, duration
        self.since = self.last = self.reached = None

    def add(self, sample):
        t = sample["sim_time_s"]
        if self.last is not None and not 0 < t-self.last <= THRESHOLDS["max_odom_gap_s"]:
            self.since = self.reached = None
        self.last = t
        if not self.predicate(sample):
            self.since = self.reached = None
            return
        self.since = t if self.since is None else self.since
        if t-self.since >= self.duration-1e-8:
            self.reached = t


def at_centre(sample, tolerance):
    return (sample["phase"] == "STANDBY" and sample["position_error_m"] <= tolerance
            and sample["speed_mps"] <= 0.1)


def at_safe_waiting_point(sample, tolerance):
    goal = sample.get("temporary_goal")
    return (sample["phase"] == "WAITING_CLEARANCE" and goal is not None and len(goal) >= 2
            and math.hypot(sample["pose"][0]-goal[0], sample["pose"][1]-goal[1]) <= tolerance
            and sample["speed_mps"] <= 0.1
            and sample.get("controlled_obstacle_clearance_m", -math.inf) >= 0.2)


def independent_orbit_metrics(samples, config, statuses, max_speed):
    """Integrate signed raw-odom increments; phase labels are observed at 1 Hz.

    No increments bridge a phase change, out-of-band sample, missing odometry or
    nonphysical position jump. Boundary uncertainty is explicit and globally
    capped, independent of the number of missing samples or avoidance episodes.
    """
    radius, tolerance = config["orbit_radius"], config["position_tolerance"]
    direction = -1 if config["clockwise"] else 1
    net = forward = reverse = 0.0
    accepted = 0
    failures = []
    segments = []
    for previous, current in zip(samples, samples[1:]):
        dt = current["sim_time_s"]-previous["sim_time_s"]
        relevant = previous["phase"] == "ORBITING" or current["phase"] == "ORBITING"
        if not relevant:
            continue
        distance = math.hypot(current["pose"][0]-previous["pose"][0], current["pose"][1]-previous["pose"][1])
        if not 0 < dt <= THRESHOLDS["max_odom_gap_s"]:
            failures.append({"reason": "odom_gap_or_nonmonotonic_stamp", "sim_time_s": current["sim_time_s"], "dt": dt})
            continue
        if distance > max_speed*dt+0.02:
            failures.append({"reason": "unphysical_odom_jump", "sim_time_s": current["sim_time_s"], "distance_m": distance})
            continue
        if not (previous["phase"] == current["phase"] == "ORBITING"
                and previous["radial_error_m"] <= tolerance and current["radial_error_m"] <= tolerance):
            continue
        theta0 = math.atan2(previous["pose"][1]-config["target"][1], previous["pose"][0]-config["target"][0])
        theta1 = math.atan2(current["pose"][1]-config["target"][1], current["pose"][0]-config["target"][0])
        delta = direction*wrap_angle(theta1-theta0)
        net += delta
        forward += max(0.0, delta)
        reverse += min(0.0, delta)
        accepted += 1
        segments.append({"sim_time_s": current["sim_time_s"], "signed_delta_rad": delta,
                         "independent_net_angle_rad": net})
    boundaries = []
    status_gaps = []
    # The status publisher uses a wall timer. A paused simulation may therefore
    # publish the same phase at the same source stamp repeatedly; these are one
    # observation, not evidence of a clock fault or extra phase-boundary time.
    # Only adjacent identical (stamp, phase) snapshots may be collapsed. Keep
    # same-stamp phase changes and backwards time for the strict checks below.
    status_observations = []
    duplicate_status_count = 0
    for current in statuses:
        if (status_observations
                and current["sim_time_s"] == status_observations[-1]["sim_time_s"]
                and current["phase"] == status_observations[-1]["phase"]):
            duplicate_status_count += 1
        else:
            status_observations.append(current)
    for previous, current in zip(status_observations, status_observations[1:]):
        if "ORBITING" not in (previous["phase"], current["phase"]):
            continue
        dt = current["sim_time_s"]-previous["sim_time_s"]
        if not 0 < dt <= THRESHOLDS["max_phase_boundary_interval_s"]:
            status_gaps.append({"stamp": current["sim_time_s"], "interval_s": dt})
        if previous["phase"] != current["phase"]:
            boundaries.append({"from": previous["phase"], "to": current["phase"],
                               "interval_s": dt, "source_stamp_s": current["sim_time_s"]})
    largest_boundary = max((max(0.0, b["interval_s"]) for b in boundaries), default=0.0)
    allowance = min(0.15, 2.0*max_speed/radius*largest_boundary)
    required = 2*math.pi*config["laps"]
    passed = (accepted > 0 and bool(boundaries) and not failures and not status_gaps
              and net+allowance >= required-1e-8)
    return {"net_angle_rad": net, "net_laps": net/(2*math.pi), "forward_angle_rad": forward,
            "reverse_angle_rad": reverse, "required_angle_rad": required, "accepted_segments": accepted,
            "boundary_angle_tolerance_rad": allowance, "boundary_intervals": boundaries,
            "phase_label_semantics": "latest received status; transition timing uncertain within logged source-stamp interval",
            "maximum_boundary_interval_s": largest_boundary, "max_speed_mps": max_speed,
            "invalid_odom_segments": failures, "invalid_status_intervals": status_gaps,
            "duplicate_status_count": duplicate_status_count,
            "status_observation_count": len(status_observations),
            "net_progress_samples": segments, "passed": passed}


def orbit_rejoined_after_obstacle(samples, config, obstacle, spawn_pose):
    """Prove an observed downstream passage and current on-circle recovery.

    Interior detours can cross near the circle centre, where polar winding is
    not a measure of passing the obstacle. Retain it only as a diagnostic;
    this function never contributes to the independent orbit lap integral.
    A proved passage remains true after continuing around the next semicircle.
    """
    if not samples:
        return {"passed": False, "reason": "no measured recovery"}
    centre, direction = config["target"], -1 if config["clockwise"] else 1
    initial_angle = math.atan2(spawn_pose[1]-centre[1], spawn_pose[0]-centre[0])
    obstacle_angle = math.atan2(obstacle["y"]-centre[1], obstacle["x"]-centre[0])
    obstacle_progress = (direction*(obstacle_angle-initial_angle)) % (2*math.pi)
    obstacle_tangent = obstacle_angle+direction*math.pi/2
    tx, ty = math.cos(obstacle_tangent), math.sin(obstacle_tangent)

    def downstream(pose):
        x, y, yaw = pose
        projection = (x-obstacle["x"])*tx+(y-obstacle["y"])*ty
        # Same 2.2 x 1.2 m rectangle as rectangle_circle_clearance. Subtract
        # its support radius so the entire hull must pass the transverse plane.
        support = 1.1*abs(math.cos(yaw-obstacle_tangent))+.6*abs(math.sin(yaw-obstacle_tangent))
        return projection, projection-support

    previous, progress = initial_angle, 0.0
    last_stamp, first_passage, minimum_gap = None, None, math.inf
    continuous, clear_history = True, True
    started_upstream = downstream(spawn_pose)[1] <= 0
    for sample in samples:
        theta = math.atan2(sample["pose"][1]-centre[1], sample["pose"][0]-centre[0])
        progress += direction*wrap_angle(theta-previous)
        previous = theta
        stamp = sample.get("sim_time_s")
        if stamp is None or not math.isfinite(stamp):
            continuous = False
        else:
            if last_stamp is not None and not 0 < stamp-last_stamp <= THRESHOLDS["max_odom_gap_s"]:
                continuous = False
            last_stamp = stamp
        swept_gap = sample.get("controlled_obstacle_clearance_m")
        if swept_gap is None or not math.isfinite(swept_gap):
            clear_history = False
        else:
            minimum_gap = min(minimum_gap, swept_gap)
            clear_history &= swept_gap >= .2
        projection, hull_projection = downstream(sample["pose"])
        if (first_passage is None and started_upstream and continuous and clear_history
                and hull_projection > 0):
            first_passage = {"sim_time_s": stamp, "pose": sample["pose"],
                             "downstream_projection_m": projection,
                             "minimum_footprint_downstream_projection_m": hull_projection}
    current = samples[-1]
    tangent = previous+direction*math.pi/2
    gap = rectangle_circle_clearance(current["pose"], obstacle)
    passed = (current["phase"] == "ORBITING" and current["radial_error_m"] <= config["position_tolerance"]
              and abs(wrap_angle(current["pose"][2]-tangent)) <= math.radians(15)
              and current["speed_mps"] >= 0.1 and first_passage is not None and gap >= 0.2
              and continuous and clear_history)
    return {"passed": passed, "directed_progress_rad": progress, "obstacle_progress_rad": obstacle_progress,
            "downstream_projection_m": projection,
            "minimum_footprint_downstream_projection_m": hull_projection,
            "first_downstream_passage": first_passage,
            "continuous_observations": continuous, "clear_swept_history": clear_history,
            "minimum_observed_swept_gap_m": minimum_gap if math.isfinite(minimum_gap) else None,
            "current_pose": current["pose"], "phase": current["phase"], "footprint_gap_m": gap,
            "radial_error_m": current["radial_error_m"], "speed_mps": current["speed_mps"]}


def observed_lateral_impulse(samples, start, requested_speed=0.4, *, response_time=None):
    """Observe the controlled velocity initial condition, including service latency."""
    response_time = start if response_time is None else response_time
    end = response_time+THRESHOLDS["impulse_post_response_window_s"]
    selected = [s for s in samples if start <= s["sim_time_s"] <= end]
    best = max(selected, key=lambda s: s["body_velocity"][1], default=None)
    return {"observed": best is not None and 0.5*requested_speed <= best["body_velocity"][1] <= 1.5*requested_speed,
            "requested_body_lateral_mps": requested_speed, "observation_interval_sim_s": [start, end],
            "peak_observed_body_lateral_mps": None if best is None else best["body_velocity"][1],
            "observed_sample": best}


def preflight_failures(costmap, config):
    """All cells in the test envelope must be observed and free of lethal cost."""
    width, height, resolution, ox, oy, data, kind = costmap
    if resolution <= 0 or len(data) != width*height:
        return ["invalid_costmap"]
    target = config["target"]
    if config["mode"] == "orbit":
        radius = config["orbit_radius"]+4.5
        inside = lambda p: math.hypot(p[0]-target[0], p[1]-target[1]) <= radius
        bounds = (target[0]-radius, target[0]+radius, target[1]-radius, target[1]+radius)
    else:
        # Covers initial route, station lateral pulse and standby radius breach.
        margin = 5.0
        inside = lambda p: point_to_polyline_distance(p, [START, target]) <= margin
        bounds = (min(START[0], target[0])-margin, max(START[0], target[0])+margin,
                  min(START[1], target[1])-margin, max(START[1], target[1])+margin)
    ix0, ix1 = math.floor((bounds[0]-ox)/resolution), math.ceil((bounds[1]-ox)/resolution)
    iy0, iy1 = math.floor((bounds[2]-oy)/resolution), math.ceil((bounds[3]-oy)/resolution)
    if min(ix0, iy0) < 0 or ix1 >= width or iy1 >= height:
        return ["test_envelope_outside_observed_map"]
    failures = []
    for iy in range(iy0, iy1+1):
        for ix in range(ix0, ix1+1):
            p = (ox+(ix+0.5)*resolution, oy+(iy+0.5)*resolution)
            cost = data[iy*width+ix]
            blocked = cost >= 254 if kind == "raw" else cost < 0 or cost >= 99
            if inside(p) and blocked:
                failures.append({"position": p, "cost": cost})
                if len(failures) >= 10:
                    return failures
    return failures


class MissionLog(Jsonl):
    def __init__(self, path, config):
        super().__init__(path)
        self.config, self.samples, self.command_samples, self.transitions = config, [], [], []
        self.phase, self.last_time, self.last_pose = "", None, None
        self.status_samples, self.latest_mission = [], {}
        self.obstacle, self.clearances, self.invalid_command = None, [], False
        self.active = False

    def write(self, event, **values):
        super().write(event, **values)
        if event == "navigation_status":
            mission = values.get("payload", {}).get("mission") or {}
            self.latest_mission = mission
            phase = mission.get("phase", "")
            status_stamp = values.get("payload", {}).get("stamp", self.last_time)
            if status_stamp is not None and math.isfinite(status_stamp):
                self.status_samples.append({"phase": phase, "sim_time_s": status_stamp,
                                            "latest_odom_stamp_s": self.last_time})
            if phase != self.phase:
                self.phase = phase
                self.transitions.append({"phase": phase, "sim_time_s": self.last_time,
                                         "wall_monotonic_s": time.monotonic()})
        elif event == "thruster_command":
            command = values["normalized_command"]
            valid = math.isfinite(command) and -1 <= command <= 1
            self.command_samples.append({"sim_time_s": self.last_time, "phase": self.phase, **values})
            if not valid:
                self.invalid_command = True
                raise TrialFailed("nonfinite_or_out_of_range_command: acquisition stopped")
        elif event == "odom":
            pose, velocity = values["pose"], values["body_velocity"]
            stamp = values["sim_time_s"]
            if not all(math.isfinite(v) for v in (*pose, *velocity, stamp)):
                raise TrialFailed("nonfinite_odometry: acquisition stopped")
            if self.last_time is not None and stamp < self.last_time-1e-6:
                raise TrialFailed("simulation_clock_rewound")
            row = {"sim_time_s": stamp, "pose": list(pose), "body_velocity": list(velocity),
                   "phase": self.phase, "temporary_goal": self.latest_mission.get("temporary_goal"),
                   **sample_errors(pose, velocity, self.config["target"])}
            if self.config["mode"] == "orbit":
                row["radial_error_m"] = abs(row["position_error_m"]-self.config["orbit_radius"])
            if self.obstacle is not None:
                clearance = (rectangle_circle_clearance(pose, self.obstacle) if self.last_pose is None
                             else swept_clearance(self.last_pose, pose, self.obstacle))
                row["controlled_obstacle_clearance_m"] = clearance
                self.clearances.append(clearance)
            self.last_time, self.last_pose = stamp, pose
            if self.active:
                self.samples.append(row)
                super().write("mission_sample", **row)
            if self.clearances and self.clearances[-1] < 0:
                raise TrialFailed("negative_swept_footprint_clearance: acquisition stopped")


def advance(probe, process, seconds, predicate=None, wall_factor=5.0):
    start = probe.sim_time()
    predicate = predicate or (lambda: False)
    done = wait_until(probe, lambda: predicate() or probe.sim_time()-start >= seconds,
                      max(60.0, seconds*wall_factor), process)
    if not done:
        raise TrialBlocked("simulation clock stalled or live acquisition wall timeout")
    return predicate()


def require_phase(probe, process, log, phase, timeout=180):
    if not advance(probe, process, timeout, lambda: log.phase == phase):
        raise TrialFailed(f"phase {phase} not observed within {timeout} simulation seconds")


def measured_dwell(probe, process, log, predicate, label, timeout=180, duration=1.0):
    detector, cursor = DwellDetector(predicate, duration), [len(log.samples)]
    def observed():
        for sample in log.samples[cursor[0]:]:
            detector.add(sample)
        cursor[0] = len(log.samples)
        return detector.reached is not None
    if not advance(probe, process, timeout, observed):
        raise TrialFailed(f"{label}: independent odometry dwell not observed within {timeout} simulation seconds")
    result = {"passed": True, "dwell_start_sim_s": detector.since, "dwell_end_sim_s": detector.reached,
              "required_dwell_s": duration, "last_observed_sample": log.samples[-1]}
    log.write("measured_dwell_confirmed", label=label, evidence=result)
    return result


def measured_centre_return(probe, process, log, label):
    return measured_dwell(probe, process, log,
                          lambda sample: at_centre(sample, log.config["position_tolerance"]), label)


def station_window(probe, process, log, label):
    start = probe.sim_time()
    detector, cursor = SettleDetector(start), [len(log.samples)]
    def settled():
        for sample in log.samples[cursor[0]:]:
            detector.add(sample)
        cursor[0] = len(log.samples)
        return detector.reached is not None
    if not advance(probe, process, 180, settled):
        raise TrialFailed(label+": station convergence not achieved within 180 simulation seconds")
    window_start = detector.reached
    log.write("fixed_window_started", label=label, sim_time_s=window_start, duration_s=60.0)
    advance(probe, process, max(0.0, window_start+60-probe.sim_time()))
    result = fixed_window_metrics(log.samples, window_start)
    result["settle_elapsed_sim_s"] = window_start-start
    log.write("fixed_window_completed", label=label, metrics=result)
    return result


def set_vessel_state(probe, pose, event, velocity=None):
    kind = probe.types["SetEntityState"]
    request = kind.Request()
    request.state.name, request.state.reference_frame = "otter", "world"
    request.state.pose.position.x, request.state.pose.position.y = float(pose[0]), float(pose[1])
    request.state.pose.position.z = 0.0
    request.state.pose.orientation.z = math.sin(pose[2]/2)
    request.state.pose.orientation.w = math.cos(pose[2]/2)
    if velocity:
        request.state.twist.linear.x, request.state.twist.linear.y = velocity
    request_time = probe.sim_time()
    probe.log.write(event+"_requested", sim_time_s=request_time, service_request_sim_s=request_time, requested_pose=pose,
                    requested_world_velocity=velocity or [0.0, 0.0])
    response = call_service(probe, kind, ["/gazebo/set_entity_state", "/set_entity_state"], request)
    response_time = probe.sim_time()
    timing = {"service_request_sim_s": request_time, "service_response_sim_s": response_time,
              "service_delay_sim_s": response_time-request_time}
    probe.log.write(event, sim_time_s=response_time, **timing, requested_pose=pose,
                    requested_world_velocity=velocity or [0.0, 0.0], success=response.success)
    if not response.success:
        raise TrialBlocked("SetEntityState rejected the controlled disturbance")
    return timing


def inject_lateral_impulse(probe, process, log):
    pose, cursor = probe.pose, len(log.samples)
    velocity = (-0.4*math.sin(pose[2]), 0.4*math.cos(pose[2]))
    timing = set_vessel_state(probe, pose, "lateral_velocity_impulse_0_4mps", velocity=velocity)
    within_delay = 0 <= timing["service_delay_sim_s"] <= THRESHOLDS["impulse_service_delay_max_s"]

    def observation():
        evidence = observed_lateral_impulse(log.samples[cursor:], timing["service_request_sim_s"],
                                            response_time=timing["service_response_sim_s"])
        return dict(evidence, **timing, service_delay_within_limit=within_delay,
                    service_delay_limit_sim_s=THRESHOLDS["impulse_service_delay_max_s"],
                    requested_world_velocity_mps=velocity)

    if not within_delay:
        log.write("lateral_velocity_impulse_observation", **observation())
        raise TrialBlocked("lateral velocity injection service delay outside the permitted [0, 2] simulation seconds")
    if not observation()["observed"] and not advance(
            probe, process, THRESHOLDS["impulse_post_response_window_s"], lambda: observation()["observed"]):
        evidence = observation()
        log.write("lateral_velocity_impulse_observation", **evidence)
        raise TrialBlocked("the requested lateral velocity impulse was not observed in odometry")
    evidence = observation()
    log.write("lateral_velocity_impulse_observation", **evidence)
    return evidence


def spawn_obstacle(probe, process, log, scenario):
    config, pose = log.config, probe.pose
    if scenario == "orbit_obstacle":
        theta = math.atan2(pose[1]-config["target"][1], pose[0]-config["target"][0])
        theta += (-1 if config["clockwise"] else 1)*math.radians(22)
        x = config["target"][0]+config["orbit_radius"]*math.cos(theta)
        y = config["target"][1]+config["orbit_radius"]*math.sin(theta)
    else:
        x, y = pose[0], pose[1]+2.8
    obstacle = {"x": x, "y": y, "radius": 0.5, "height": 4.0}
    gap = rectangle_circle_clearance(pose, obstacle)
    if gap <= 0.2:
        raise TrialBlocked("controlled obstacle spawn would violate the initial footprint clearance")
    kind = probe.types["SpawnEntity"]
    request = kind.Request()
    request.name = "mission_acceptance_"+uuid.uuid4().hex[:10]
    request.reference_frame = "world"
    request.initial_pose.position.x, request.initial_pose.position.y = float(x), float(y)
    request.initial_pose.position.z, request.initial_pose.orientation.w = 1.0, 1.0
    request.xml = f'''<sdf version="1.6"><model name="{request.name}"><static>true</static><link name="body">
<collision name="collision"><geometry><cylinder><radius>0.5</radius><length>4</length></cylinder></geometry></collision>
<visual name="visual"><geometry><cylinder><radius>0.5</radius><length>4</length></cylinder></geometry></visual>
</link></model></sdf>'''
    log.obstacle = obstacle
    log.last_pose = None  # The new obstacle did not exist on earlier odometry segments.
    response = call_service(probe, kind, ["/spawn_entity", "/gazebo/spawn_entity"], request)
    log.write("controlled_obstacle_spawned", sim_time_s=probe.sim_time(), entity_name=request.name,
              obstacle=obstacle, initial_footprint_gap_m=gap, success=response.success)
    if not response.success:
        log.obstacle = None
        raise TrialBlocked("controlled obstacle spawn failed: "+response.status_message)
    return request.name


def obstacle_observed(costmap, obstacle):
    if costmap is None:
        return False
    width, height, resolution, ox, oy, data, kind = costmap
    if resolution <= 0 or len(data) != width*height:
        return False
    cx, cy = math.floor((obstacle["x"]-ox)/resolution), math.floor((obstacle["y"]-oy)/resolution)
    reach = max(1, math.ceil(obstacle["radius"]/resolution)+1)
    for iy in range(max(0, cy-reach), min(height, cy+reach+1)):
        for ix in range(max(0, cx-reach), min(width, cx+reach+1)):
            cost = data[iy*width+ix]
            if (kind == "raw" and cost == 254) or (kind == "occupancy" and cost >= 99):
                return True
    return False


def remove_obstacle(probe, log, name, reason="measured_recovery_complete"):
    from gazebo_msgs.srv import DeleteEntity
    request = DeleteEntity.Request()
    request.name = name
    response = call_service(probe, DeleteEntity, ["/delete_entity", "/gazebo/delete_entity"], request)
    log.write("controlled_obstacle_deleted", sim_time_s=probe.sim_time(), entity_name=name,
              success=response.success, reason=reason)
    if not response.success:
        raise TrialBlocked("controlled obstacle deletion failed")
    log.obstacle = None


def controlled_avoidance(probe, process, log, scenario):
    start_index = len(log.transitions)
    sample_start, spawn_pose = len(log.samples), probe.pose
    entity = spawn_obstacle(probe, process, log, scenario)
    recovered = False
    try:
        if not advance(probe, process, 20, lambda: obstacle_observed(probe.costmap, log.obstacle)):
            raise TrialBlocked("spawned controlled obstacle was not observed as lethal in the live costmap")
        log.write("controlled_obstacle_observed_in_costmap", sim_time_s=probe.sim_time(),
                  costmap_stamp=probe.costmap_stamp)
        probe.capture_inputs("controlled_obstacle_snapshot")
        if not advance(probe, process, 60, lambda: any(t["phase"] == "AVOIDING" for t in log.transitions[start_index:])):
            raise TrialFailed("controlled obstacle did not trigger AVOIDING")
        if scenario == "orbit_obstacle":
            def on_original_circle(sample):
                history = [row for row in log.samples[sample_start:] if row["sim_time_s"] <= sample["sim_time_s"]]
                return orbit_rejoined_after_obstacle(history, log.config,
                                                     log.obstacle, spawn_pose)["passed"]
            evidence = measured_dwell(probe, process, log, on_original_circle,
                                      "obstacle_kept_until_measured_orbit_rejoin", timeout=180)
            evidence["orbit_rejoin"] = orbit_rejoined_after_obstacle(
                log.samples[sample_start:], log.config, log.obstacle, spawn_pose)
        else:
            evidence = measured_dwell(probe, process, log,
                lambda sample: at_safe_waiting_point(sample, log.config["position_tolerance"]),
                "obstacle_kept_until_measured_safe_point_wait", timeout=180)
        recovered = True
        log.write("controlled_obstacle_recovery_confirmed", sim_time_s=probe.sim_time(), evidence=evidence)
        return evidence
    finally:
        remove_obstacle(probe, log, entity,
                        reason="measured_recovery_complete" if recovered else "cleanup_after_failed_recovery")


def fingerprint(workspace):
    workspace = Path(workspace).resolve()
    suffixes = {".py", ".yaml", ".yml", ".srv", ".msg", ".action", ".world", ".xml",
                ".cpp", ".hpp", ".h", ".bash", ".json", ".rviz", ".xacro", ".sdf",
                ".cmake", ".ini", ".cfg", ".txt"}
    files = {}
    # Task validation now lives under src, included exactly once. Scripts are
    # also hashed because the optional legacy navigation regression uses them.
    for folder in (workspace/"src", workspace/"scripts"):
        if folder.is_dir():
            for path in sorted(folder.rglob("*")):
                if (not {"__pycache__", ".pytest_cache"}.intersection(path.parts)
                        and path.is_file() and (path.suffix in suffixes or path.name == "CMakeLists.txt")):
                    files[str(path.relative_to(workspace))] = hashlib.sha256(path.read_bytes()).hexdigest()
    digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    try:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=workspace,
                                  capture_output=True, text=True, timeout=10)
        commit = revision.stdout.strip() if revision.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        commit = None
    installed = {}
    install = workspace/"install"
    for folder in sorted(install.iterdir()) if install.is_dir() else []:
        if folder.is_dir():
            for path in sorted(folder.rglob("*")):
                if (not {"__pycache__", ".pytest_cache"}.intersection(path.parts)
                        and path.is_file() and (path.suffix in suffixes | {".so"}
                                       or path.name in {"sim_env_node", "mission_node", "mission_cli"})):
                    installed[str(path.relative_to(workspace))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"git_commit": commit, "source_sha256": digest, "files": files,
            "install_sha256": hashlib.sha256(json.dumps(installed, sort_keys=True).encode()).hexdigest(),
            "installed_files": installed}


def run_trial(args, scenario, repeat, domain):
    from common.geodesy import GeoReference
    from usv_mission_interfaces.srv import SetMission
    from ament_index_python.packages import get_package_share_directory
    config = scenario_config(scenario)
    directory = args.output/f"{scenario}_{repeat:02d}"
    directory.mkdir()
    log = MissionLog(directory/"events.jsonl", config)
    result = {"trial": directory.name, "scenario": scenario, "repeat": repeat,
              "domain_id": domain, "result": "not_run", "passed": False, "config": config,
              "thresholds": THRESHOLDS, "windows": {}}
    process = probe = None
    max_speed = None
    try:
        probe = make_probe("otter", log)
        wait_until(probe, lambda: False, 1.0)
        existing_services = {name for name, _ in probe.get_service_names_and_types()}
        simulator_services = {"/spawn_entity", "/gazebo/spawn_entity", "/set_entity_state",
                              "/gazebo/set_entity_state", "/delete_entity", "/gazebo/delete_entity"}
        if (probe.count_publishers("/otter/odom") or probe.count_publishers("/clock")
                or existing_services & simulator_services):
            raise TrialBlocked("pre-existing simulation in selected domain; no processes started")
        port = free_port()
        result["gazebo_master_uri"] = f"http://127.0.0.1:{port}"
        environment = dict(os.environ, GAZEBO_MASTER_URI=result["gazebo_master_uri"],
                           ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
        command = ["ros2", "launch", "usv_mission", "mission_sim.launch.py", "world:="+args.world,
                   "headless:=true", "gui:=false", "rviz:=false", "robot_namespace:=otter",
                   "robot_x:=23.0", "robot_y:=-10.0", "robot_yaw:=0.0"]
        log.write("trial_started", **result, command=command)
        process = OwnedProcess(args.workspace, command, directory/"launch.log", environment)
        if not wait_until(probe, lambda: probe.pose is not None and probe.scan_wall is not None
                          and probe.costmap is not None and probe.status is not None,
                          args.startup_timeout, process, lambda: probe.discover_costmap()):
            raise TrialBlocked("odom, scan, costmap and navigation status did not become ready")
        activate_graph_guard(probe, log)
        if config["mode"] == "orbit":
            kind = probe.types["GetParameters"]
            request = kind.Request()
            request.names = ["dynamics.velocity_max"]
            response = call_service(probe, kind, ["/otter/mission_manager/get_parameters"], request)
            max_speed = parameter_value(response.values[0]) if response.values else None
            if type(max_speed) not in (float, int) or not math.isfinite(max_speed) or max_speed <= 0:
                raise TrialBlocked("live model velocity_max unavailable for independent odometry validation")
            result["observed_model_max_speed_mps"] = max_speed
            log.write("independent_orbit_model_limit", maximum_speed_mps=max_speed)
        set_vessel_state(probe, START, "initial_vessel_placement")
        placed = time.monotonic()
        if not wait_until(probe, lambda: probe.odom_wall > placed and probe.costmap_wall > placed+1.0,
                          20, process):
            raise TrialBlocked("fresh state and map not observed after initial placement")
        probe.capture_inputs("mission_preflight_snapshot")
        failures = preflight_failures(probe.costmap, config)
        log.write("environment_preflight", passed=not failures, failures=failures)
        if failures:
            raise TrialBlocked("existing geometry or unknown cells in test envelope: "+str(failures))
        world = Path(args.world)
        if not world.is_absolute():
            world = Path(get_package_share_directory("sim_env"))/"worlds"/(args.world.removesuffix(".world")+".world")
        geo = GeoReference.from_world(world)
        lat, lon, _ = geo.to_geographic(*config["target"][:2])
        request = SetMission.Request()
        for name, value in config.items():
            if name != "target":
                setattr(request, name, value)
        request.latitude, request.longitude = lat, lon
        accepted = call_service(probe, SetMission, ["/otter/set_mission"], request)
        log.write("mission_requested", sim_time_s=probe.sim_time(), latitude=lat, longitude=lon,
                  config=config, accepted=accepted.accepted, reason=accepted.reason,
                  mission_id=accepted.mission_id)
        if not accepted.accepted:
            raise TrialFailed("mission request rejected: "+accepted.reason)
        result["mission_id"] = accepted.mission_id
        log.active = True
        began = probe.sim_time()
        if scenario.startswith("station"):
            result["windows"]["initial"] = station_window(probe, process, log, "initial")
            if scenario == "station_disturbance":
                result["lateral_velocity_impulse"] = inject_lateral_impulse(probe, process, log)
                result["windows"]["after_disturbance"] = station_window(probe, process, log, "after_disturbance")
            checks = [window["passed"] for window in result["windows"].values()]
        elif scenario.startswith("standby"):
            result["initial_centre_dwell"] = measured_centre_return(probe, process, log, "initial_centre")
            idle_start = probe.sim_time()
            advance(probe, process, 10)
            idle = [c for c in log.command_samples if c["sim_time_s"] is not None
                    and idle_start+0.5 <= c["sim_time_s"] <= idle_start+10]
            result["idle_zero_commands"] = bool(idle) and all(abs(c["normalized_command"]) <= 1e-6 for c in idle)
            if scenario == "standby_obstacle":
                result["controlled_avoidance"] = controlled_avoidance(probe, process, log, scenario)
            else:
                pose = probe.pose
                displaced = (pose[0], pose[1]+config["watch_radius"]+1, pose[2])
                set_vessel_state(probe, displaced, "standby_boundary_displacement")
                require_phase(probe, process, log, "RETURN_CENTER", 30)
            result["returned_centre_dwell"] = measured_centre_return(probe, process, log, "returned_centre")
            checks = [result["idle_zero_commands"], result["initial_centre_dwell"]["passed"],
                      result["returned_centre_dwell"]["passed"]]
        else:
            require_phase(probe, process, log, "ORBITING", 180)
            if scenario == "orbit_obstacle":
                advance(probe, process, 10)
                result["controlled_avoidance"] = controlled_avoidance(probe, process, log, scenario)
                require_phase(probe, process, log, "ORBITING", 180)
            require_phase(probe, process, log, "RETURN_CENTER", 600)
            result["orbit_completed_status"] = (probe.status.get("mission") or {}).get("orbit_completed", False)
            result["returned_centre_dwell"] = measured_centre_return(probe, process, log, "orbit_completed_centre")
            radial = [s["radial_error_m"] for s in log.samples if s["phase"] == "ORBITING"]
            result["normal_orbit_radial_error_m"] = distribution(radial)
            result["independent_orbit_progress"] = independent_orbit_metrics(log.samples, config, log.status_samples, max_speed)
            log.write("independent_orbit_progress", metrics=result["independent_orbit_progress"])
            checks = [bool(radial) and distribution(radial)["p95"] <= 0.5, result["orbit_completed_status"],
                      result["independent_orbit_progress"]["passed"], result["returned_centre_dwell"]["passed"]]
        result["duration_sim_s"] = probe.sim_time()-began
        final_mission = probe.status.get("mission") or {}
        original = final_mission.get("original_target") or {}
        result["original_mission_preserved"] = (
            str(final_mission.get("mission_id")) == str(accepted.mission_id)
            and final_mission.get("mode") == config["mode"]
            and math.isclose(original.get("latitude", math.inf), lat, abs_tol=1e-10)
            and math.isclose(original.get("longitude", math.inf), lon, abs_tol=1e-10)
            and math.isclose(original.get("heading_deg", math.inf), config["heading_deg"], abs_tol=1e-10))
        checks.append(result["original_mission_preserved"])
        if "obstacle" in scenario:
            result["minimum_controlled_obstacle_footprint_gap_m"] = min(log.clearances) if log.clearances else None
            checks.append(bool(log.clearances) and min(log.clearances) >= 0.2)
        probe.graph_guard.check(force=True)
        checks.append(probe.graph_guard.evidence()["passed"])
        result["passed"] = all(checks) and not log.invalid_command
        result["result"] = "passed" if result["passed"] else "failed"
        result["reason"] = "all measured criteria passed" if result["passed"] else "one or more measured acceptance criteria failed"
    except TrialBlocked as error:
        result.update(result="blocked", reason=str(error))
    except (TrialFailed, RuntimeError, ValueError) as error:
        result.update(result="failed", reason=str(error))
        if probe is not None:
            try:
                probe.capture_inputs("mission_failure_snapshot")
            except Exception as capture_error:
                log.write("mission_failure_snapshot_error", error=str(capture_error))
    finally:
        log.active = False
        if probe is not None:
            result["final_status"] = probe.status
            guard = getattr(probe, "graph_guard", None)
            result["control_ownership_graph"] = (guard.evidence() if guard is not None else
                {"requirements": GRAPH_REQUIREMENTS, "observation_count": 0,
                 "observations": [], "passed": False})
            # Acquisition is over. Cleanup must not append observations after
            # the measured observation_count/manifest snapshot was frozen.
            probe.graph_guard = None
            if not result["control_ownership_graph"]["passed"]:
                result["passed"] = False
            if process is not None and process.process.poll() is None:
                try:
                    kind = probe.types["Trigger"]
                    call_service(probe, kind, ["/otter/cancel_navigation"], kind.Request(), timeout=3, optional=True)
                except Exception as error:
                    log.write("cleanup_cancel_error", error=str(error))
            probe.destroy_node()
        if process is not None:
            process.close()
        result["phase_transitions"] = log.transitions
        result["invalid_command_observed"] = log.invalid_command
        result["sample_count"] = len(log.samples)
        if config["mode"] == "orbit" and max_speed is not None and "independent_orbit_progress" not in result:
            result["independent_orbit_progress"] = independent_orbit_metrics(log.samples, config, log.status_samples, max_speed)
        (directory/"metrics.json").write_text(json.dumps(sanitized(result), ensure_ascii=False, indent=2), encoding="utf-8")
        log.write("trial_finished", **result)
        log.close()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=find_source_root())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--domain-id", type=int, default=191)
    parser.add_argument("--world", default="ocean")
    parser.add_argument("--startup-timeout", type=float, default=120)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    args.workspace, args.output = args.workspace.resolve(), args.output.resolve()
    matrix = [(scenario, repeat) for scenario in args.scenarios for repeat in range(1, args.repeats+1)]
    if args.repeats < 1 or not matrix:
        parser.error("repeats must be positive")
    if len(set(args.scenarios)) != len(args.scenarios):
        parser.error("scenarios must not be duplicated")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("startup timeout must be finite and positive")
    if not 0 <= args.domain_id <= args.domain_id+len(matrix)-1 <= 232:
        parser.error("selected trial ROS domain range must fit within [0,232]")
    if args.output.exists():
        parser.error("output already exists; use a new evidence directory")
    args.output.mkdir(parents=True)
    before = fingerprint(args.workspace)
    manifest = {"classification": "dry_run" if args.dry_run else "actual ROS/Gazebo mission acceptance",
                "launch": {"package": "usv_mission", "file": "mission_sim.launch.py"},
                "control_ownership_graph_requirements": GRAPH_REQUIREMENTS,
                "workspace": str(args.workspace), "world": args.world, "created_unix_s": time.time(),
                "thresholds": THRESHOLDS, "matrix": [{"scenario": s, "repeat": r,
                "domain_id": args.domain_id+i} for i, (s, r) in enumerate(matrix)], "source_before": before}
    (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    if args.dry_run:
        summary = {"result": "not_run", "all_passed": False, "planned_trials": len(matrix), "results": []}
        (args.output/"summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary), flush=True)
        return 0
    if os.name != "posix":
        parser.error("live trials require WSL/Linux with the selected ROS install sourced")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    results, interrupted = [], False
    try:
        for i, (scenario, repeat) in enumerate(matrix):
            domain = args.domain_id+i
            os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            try:
                result = run_trial(args, scenario, repeat, domain)
                results.append(result)
                print(json.dumps(sanitized(result), ensure_ascii=False), flush=True)
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        after = fingerprint(args.workspace)
        manifest["control_ownership_graph_observations"] = [
            {"trial": item["trial"], **item.get("control_ownership_graph", {"passed": False})}
            for item in results]
        (args.output/"manifest.json").write_text(json.dumps(sanitized(manifest), indent=2), encoding="utf-8")
        summary = {"results": results, "interrupted": interrupted, "source_after": after,
                   "source_unchanged": before["source_sha256"] == after["source_sha256"],
                   "installation_unchanged": before["install_sha256"] == after["install_sha256"],
                   "planned_trials": len(matrix), "completed_trials": len(results),
                   "all_passed": len(results) == len(matrix) and all(r["passed"] for r in results) and not interrupted}
        summary["all_passed"] &= summary["source_unchanged"]
        summary["all_passed"] &= summary["installation_unchanged"]
        (args.output/"summary.json").write_text(json.dumps(sanitized(summary), ensure_ascii=False, indent=2), encoding="utf-8")
    return 130 if interrupted else 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
