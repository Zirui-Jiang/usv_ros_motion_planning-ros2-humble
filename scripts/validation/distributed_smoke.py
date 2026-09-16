#!/usr/bin/env python3
# 用合成 odom 和空地图验证独立进程间的目标、停推和重启协议，不用于评价 Gazebo 导航精度。
"""Exercise installed decision/control/perception as independent ROS processes.

Synthetic odometry and costmaps isolate process/protocol behavior. This is not a
Gazebo navigation-accuracy test or a multi-computer network validation. Only this
runner's children are signalled; results are always written to a new directory.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from ros_acceptance import Jsonl, OwnedProcess, TrialBlocked, sanitized, zero_output_window


# 持有测试节点、受控输入和自建业务子进程，观测会话、参考与真实推进器输出。
class Smoke:
    def __init__(self, args, log):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from nav2_msgs.msg import Costmap
        from std_msgs.msg import Float32, String
        from std_srvs.srv import Trigger
        from usv_navigation_interfaces.msg import NavigationReference, ControlReport, ModuleHeartbeat
        self.args, self.log, self.rclpy = args, log, rclpy
        self.namespace = "process_smoke_" + uuid.uuid4().hex[:10]
        self.node = Node("distributed_validation", namespace=self.namespace)
        self.types = dict(PoseStamped=PoseStamped, Odometry=Odometry, Costmap=Costmap, Trigger=Trigger)
        self.odom_pub = self.node.create_publisher(Odometry, "odom", 10)
        self.map_pub = self.node.create_publisher(Costmap, "costmap_raw",
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.goal_pub = self.node.create_publisher(PoseStamped, "goal", 10)
        self.replay_pub = self.node.create_publisher(NavigationReference, "navigation/reference", 10)
        self.client = self.node.create_client(Trigger, "cancel_navigation")
        self.commands = {axis: [] for axis in ("left", "right", "lateral")}
        self.peers, self.status, self.report, self.reference = {}, {}, None, None
        self.status_wall = float("-inf")
        self.reference_wall = self.report_wall = float("-inf")
        self.children, self.generations = {}, {}
        self.publish_odom = self.publish_map = True
        self.next_publish = 0.
        self.subscriptions = [
            self.node.create_subscription(Float32, f"thrusters/{axis}_thrust_cmd",
                lambda msg, axis=axis: self.on_command(axis, msg), 10)
            for axis in self.commands]
        self.subscriptions += [
            self.node.create_subscription(String, "navigation_status", self.on_status, 10),
            self.node.create_subscription(ControlReport, "navigation/control_report", self.on_report, 10),
            self.node.create_subscription(ModuleHeartbeat, "navigation/heartbeat", self.on_heartbeat, 10),
            self.node.create_subscription(NavigationReference, "navigation/reference", self.on_reference, 10),
        ]

    # 保存每个推进器通道的实际输出和接收时间，用于判定运动及停推。
    def on_command(self, axis, msg):
        received = time.monotonic()
        self.commands[axis].append((received, float(msg.data)))
        self.log.write("command", axis=axis, value=msg.data, received=received)

    # 记录决策公开状态，供目标取消和任务失效的后续断言读取。
    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_wall = time.monotonic()
        self.log.write("navigation_status", **self.status)

    # 记录航控回报，区分决策请求与控制端实际处理结果。
    def on_report(self, msg):
        self.report, self.report_wall = msg, time.monotonic()
        self.log.write("control_report", control_session=msg.control_session,
            planner_session=msg.planner_session, task_id=msg.task_id,
            reference_sequence=msg.reference_sequence, state=msg.state,
            task_finished=msg.task_finished, reason=msg.reason,
            left=msg.left, right=msg.right, lateral=msg.lateral,
            counters=json.loads(msg.counters_json))

    # 记录模块会话存活信息，用于识别进程重启后的身份变化。
    def on_heartbeat(self, msg):
        self.peers[msg.module] = (msg.session, msg.ready, time.monotonic())

    # 深拷贝已收到的完整轨迹参考，便于测试迟到消息和旧版本重放。
    def on_reference(self, msg):
        self.reference, self.reference_wall = copy.deepcopy(msg), time.monotonic()
        self.log.write("reference", planner_session=msg.planner_session,
            control_session=msg.control_session, perception_session=msg.perception_session,
            task_id=msg.task_id, sequence=msg.sequence, active=msg.active,
            trajectory_stamp=msg.trajectory_stamp, points=len(msg.points), state=msg.state)

    # 以独立子进程启动指定模块，保存日志与进程所有权。
    def start(self, role):
        executable = {"decision": "decision_node", "control": "control_node",
                      "perception": "perception_monitor"}[role]
        package = {"decision": "usv_decision", "control": "usv_control",
                   "perception": "usv_perception"}[role]
        installed = self.args.workspace / "install" / package / "lib" / package / executable
        if not installed.is_file():
            raise TrialBlocked(f"installed executable absent: {installed}")
        generation = self.generations.get(role, 0) + 1
        self.generations[role] = generation
        command = ["/usr/bin/python3", str(installed), "--ros-args", "-r", f"__ns:=/{self.namespace}",
                   "-p", "use_sim_time:=false"]
        if role == "perception":
            command += ["-p", "watched_topics:=[odom,costmap_raw]"]
        else:
            command += ["-p", f"dynamics.ns:={self.namespace}",
                        "-p", "navigation.state_timeout:=0.4", "-p", "navigation.map_timeout:=1.5"]
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        child = OwnedProcess(self.args.workspace, command,
                    self.args.output / f"{role}-{generation}.log", environment)
        self.children[role] = child
        self.log.write("process_started", role=role, generation=generation,
                       pid=child.process.pid, command=command)

    # 只停止测试自己创建的指定模块，模拟单模块退出与重启。
    def stop(self, role):
        child = self.children.pop(role)
        child.close()
        self.log.write("process_stopped", role=role, pid=child.process.pid,
                       exit_code=child.process.returncode)

    # 按测试开关发送静止自船状态和空地图，可单独停止某一路输入。
    def inputs(self):
        now = time.monotonic()
        if now < self.next_publish:
            return
        self.next_publish = now + .05
        stamp = self.node.get_clock().now().to_msg()
        if self.publish_odom:
            msg = self.types["Odometry"]()
            msg.header.stamp, msg.header.frame_id = stamp, self.namespace + "/map"
            msg.child_frame_id = self.namespace + "/base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y = 10., 10.
            msg.pose.pose.orientation.w = 1.
            self.odom_pub.publish(msg)
        if self.publish_map:
            msg = self.types["Costmap"]()
            msg.header.stamp, msg.header.frame_id = stamp, self.namespace + "/map"
            msg.metadata.size_x = msg.metadata.size_y = 128
            msg.metadata.resolution = .5
            msg.metadata.origin.orientation.w = 1.
            msg.metadata.map_load_time = msg.metadata.update_time = stamp
            msg.data = [0] * (128 * 128)
            self.map_pub.publish(msg)

    # 执行一次受控输入、进程健康检查和 ROS 回调处理。
    def step(self):
        for child in self.children.values():
            child.check()
        self.inputs()
        self.rclpy.spin_once(self.node, timeout_sec=.01)

    # 等待可观测条件成立，超时则报告阻塞，不修改原有等待阈值。
    def until(self, predicate, timeout=12.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.step()
        if not predicate():
            raise TrialBlocked(f"condition timed out: status={self.status}, "
                               f"report={None if self.report is None else self.report.state}")

    # 在给定时长内持续推进输入与回调，为稳态观察提供时间窗口。
    def pump(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.step()

    # 等待订阅连接和模块就绪后再下发任务，减少启动阶段对协议断言的干扰。
    def ready(self):
        self.until(lambda: all(role in self.peers and self.peers[role][1]
                   and time.monotonic() - self.peers[role][2] < 1.
                   for role in ("decision", "control", "perception"))
                   and self.goal_pub.get_subscription_count() >= 1)
        self.pump(.3)

    # 发布带当前 ROS 时间的目标点，建立下一次目标处理观察。
    def goal(self, x=20.):
        msg = self.types["PoseStamped"]()
        msg.header.frame_id = self.namespace + "/map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, 10.
        msg.pose.orientation.w = 1.
        self.goal_pub.publish(msg)
        self.log.write("goal_sent", x=x, y=10.)

    # 等待实际收到非零命令，证明控制端进入输出阶段后再开展故障测试。
    def moving(self):
        since = time.monotonic()
        self.until(lambda: self.report is not None and self.report_wall >= since
                   and self.report.state == "NAVIGATING"
                   and any(stamp >= since and abs(value) > 1e-6
                           for stamp, value in self.commands["left"]), 20.)

    # 收集观察窗口并调用持续零输出判据，拒绝仅有一条历史零命令的假停推。
    def zero_window(self, duration=.6):
        since = time.monotonic()
        self.pump(duration)
        result = zero_output_window(self.commands, since, time.monotonic())
        self.log.write("zero_output_window", **result)
        if not result["passed"]:
            raise AssertionError(f"zero-output window failed: {result}")
        return {axis: evidence["samples"] for axis, evidence in result["axes"].items()}

    # 依次回收本测试的子进程并销毁观测节点。
    def close(self):
        for role in list(self.children):
            self.stop(role)
        self.node.destroy_node()


# 顺序执行进程所有权、取消、断流和模块重启用例，每项都依据真实订阅结果判定。
def run(args):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    log = Jsonl(args.output / "events.jsonl")
    smoke = None
    checks = []
    summary = dict(passed=False, result="running", checks=checks,
                   scope="independent local ROS processes with synthetic sensors; no Gazebo/multi-host claim")

    def passed(name, **evidence):
        checks.append(dict(name=name, passed=True, **evidence))
        log.write("check_passed", name=name, **evidence)
        print(json.dumps(checks[-1]), flush=True)

    try:
        smoke = Smoke(args, log)
        smoke.pump(.4)
        if smoke.node.count_publishers("thrusters/left_thrust_cmd"):
            raise TrialBlocked("test namespace is already controlled")
        for role in ("perception", "control", "decision"):
            smoke.start(role)
        smoke.ready()
        counts = {axis: smoke.node.count_publishers(f"thrusters/{axis}_thrust_cmd") for axis in smoke.commands}
        if any(value != 1 for value in counts.values()):
            raise AssertionError(f"duplicate/missing control publisher: {counts}")
        pids = {role: child.process.pid for role, child in smoke.children.items()}
        if len(set(pids.values())) != 3:
            raise AssertionError("modules are not separate processes")
        passed("three_processes_and_unique_output", pids=pids, publishers=counts)
        smoke.goal()
        smoke.moving()
        smoke.until(lambda: smoke.reference is not None and smoke.reference.active and len(smoke.reference.points) > 0)
        old_reference = copy.deepcopy(smoke.reference)
        passed("installed_end_to_end_reference", points=len(old_reference.points),
               trajectory_stamp=old_reference.trajectory_stamp, task_id=old_reference.task_id)

        smoke.until(smoke.client.service_is_ready)
        future = smoke.client.call_async(smoke.types["Trigger"].Request())
        smoke.until(future.done)
        if not future.result().success:
            raise AssertionError(future.result().message)
        smoke.until(lambda: smoke.report is not None and smoke.report.state == "CANCELLED")
        # Pause only our decision process so its recurring inactive snapshot
        # cannot overwrite an erroneously accepted old TRACK. Keep this test
        # shorter than peer expiry, which must not mask a protocol failure.
        decision_process = smoke.children["decision"].process
        paused_at = time.monotonic()
        decision_process.send_signal(signal.SIGSTOP)
        try:
            smoke.pump(.15)
            cancelled_identity = (smoke.report.task_id, smoke.report.reference_sequence)
            rejected_before = json.loads(smoke.report.counters_json).get("stale_references", 0)
            sent_at = time.monotonic()
            smoke.replay_pub.publish(old_reference)
            smoke.until(lambda: smoke.report_wall >= sent_at
                        and json.loads(smoke.report.counters_json).get("stale_references", 0) > rejected_before,
                        timeout=.35)
            if (smoke.report.task_id, smoke.report.reference_sequence) != cancelled_identity:
                raise AssertionError("late reference changed the acknowledged cancelled task identity")
            if smoke.report.state != "CANCELLED":
                raise AssertionError("late reference was masked by a different control gate")
            counts = smoke.zero_window(.45)
            paused_duration = time.monotonic() - paused_at
            if paused_duration >= 1.2 or smoke.report.state != "CANCELLED":
                raise AssertionError("decision pause reached the heartbeat safety gate")
            passed("cancel_ack_and_late_reference_do_not_revive", observed_zero_samples=counts,
                   cancelled_task_id=cancelled_identity[0], reference_sequence=cancelled_identity[1],
                   stale_reference_counter_before=rejected_before,
                   stale_reference_counter_after=json.loads(smoke.report.counters_json).get("stale_references", 0),
                   paused_decision_s=paused_duration)
        finally:
            decision_process.send_signal(signal.SIGCONT)

        for source, deadline in (("odom", .7), ("map", 1.9)):
            smoke.goal()
            smoke.moving()
            began = time.monotonic()
            setattr(smoke, "publish_" + source, False)
            log.write("source_stopped", source=source, deadline_s=deadline)
            smoke.until(lambda: all(any(stamp >= began and abs(value) <= 1e-6 for stamp, value in records)
                                    for records in smoke.commands.values()), deadline)
            elapsed = time.monotonic() - began
            smoke.zero_window(1.)
            passed(source + "_dropout_stops_control", zero_latency_s=elapsed, deadline_s=deadline)
            setattr(smoke, "publish_" + source, True)
            smoke.pump(.4)

        for role in ("decision", "control", "perception"):
            smoke.goal()
            smoke.moving()
            previous = smoke.peers[role][0]
            smoke.stop(role)
            # Keep fresh input streaming while the remaining modules observe
            # the absence. A control-process absence has no publisher at all.
            smoke.pump(1.7)
            if role != "control":
                smoke.zero_window(.5)
            restarted_at = time.monotonic()
            smoke.start(role)
            smoke.until(lambda: smoke.peers.get(role, (previous,))[0] != previous)
            smoke.ready()
            smoke.until(lambda: smoke.status_wall >= restarted_at
                        and smoke.status.get("goal", "not_received") is None)
            samples = smoke.zero_window(.7)
            passed(role + "_restart_requires_new_goal", old_session=previous,
                   new_session=smoke.peers[role][0], observed_zero_samples=samples)
            smoke.goal()
            smoke.moving()
            passed(role + "_restart_recovers_on_new_goal")

        # Recreate an actual perception output publisher while the monitor
        # stays alive. It must rotate the module identity on the new GID.
        previous = smoke.peers["perception"][0]
        restarted_at = time.monotonic()
        smoke.node.destroy_publisher(smoke.odom_pub)
        smoke.odom_pub = smoke.node.create_publisher(smoke.types["Odometry"], "odom", 10)
        smoke.until(lambda: smoke.peers["perception"][0] != previous)
        smoke.until(lambda: smoke.status_wall >= restarted_at
                    and smoke.status.get("goal", "not_received") is None)
        smoke.zero_window(.7)
        passed("perception_output_publisher_restart_invalidates_task")
        summary.update(passed=True, result="completed")
    except KeyboardInterrupt:
        summary.update(result="interrupted", reason="user interrupted")
    except Exception as error:
        summary.update(result="failed", reason=repr(error))
        log.write("failure", reason=repr(error))
    finally:
        if smoke is not None:
            try:
                smoke.close()
            except Exception as error:
                summary.update(passed=False, result="cleanup_failed", cleanup_error=repr(error))
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        (args.output / "summary.json").write_text(json.dumps(sanitized(summary), indent=2), encoding="utf-8")
    return 0 if summary["passed"] else 1


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=217)
    args = parser.parse_args(argv)
    args.workspace, args.output = args.workspace.resolve(), args.output.resolve()
    if os.name != "posix":
        parser.error("run on Linux/WSL with the selected ROS installation sourced")
    if not 1 <= args.domain_id <= 232:
        parser.error("domain-id must be 1..232")
    if args.output.exists():
        parser.error("output must be a new directory")
    if not (args.workspace / "install/setup.bash").is_file():
        parser.error("workspace must contain install/setup.bash")
    args.output.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1")
    manifest = dict(workspace=str(args.workspace), domain_id=args.domain_id,
                    command=sys.argv, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    odom_timeout_s=.4, map_timeout_s=1.5, peer_timeout_s=1.5,
                    zero_output_max_gap_s=.35, zero_output_minimum_samples_per_axis=3,
                    source="synthetic_state_and_empty_costmap", use_sim_time=False)
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
