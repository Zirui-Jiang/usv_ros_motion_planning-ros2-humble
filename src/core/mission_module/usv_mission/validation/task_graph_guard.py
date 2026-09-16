# 验收图所有权检查：确保任务节点唯一且独占三个推进器命令话题，并记录每次检查证据。
"""Measured ROS graph ownership checks for task-node validation.

The pure evaluator accepts a graph snapshot for ROS-free tests. The live guard
records every poll and rejects any ownership violation once acquisition starts.
"""
import time


GRAPH_REQUIREMENTS = {
    "node": "mission_manager",
    "forbidden_node": "usv_simulation",
    "required_thruster_axes": ["left", "right", "lateral"],
    "maximum_poll_interval_wall_s": 0.5,
    "command_publishers_per_topic": 1,
}


# 发现图所有权违约时中止当前验收采集。
class GraphViolation(RuntimeError):
    pass


# 将节点名和命名空间拼成规范绝对路径，保留检查所需身份。
def node_path(name, namespace):
    return "/" + "/".join(part for part in (namespace.strip("/"), name.strip("/")) if part)


# 对公开图快照做纯函数判定，重复节点/端点、旧控制节点及额外竞争轴均可暴露。
def evaluate_graph(namespace, nodes, publishers):
    """Require exactly one task node and one owned endpoint per command topic.

    ``nodes`` is an iterable of (name, namespace) pairs. ``publishers`` maps
    absolute topic names to endpoint node paths, retaining duplicate endpoints.
    All advertised thruster command topics are checked, including extra axes.
    """
    namespace = "/" + namespace.strip("/")
    expected = namespace + "/" + GRAPH_REQUIREMENTS["node"]
    forbidden = namespace + "/" + GRAPH_REQUIREMENTS["forbidden_node"]
    paths = sorted(node_path(name, ns) for name, ns in nodes)
    required = {namespace + "/thrusters/" + axis + "_thrust_cmd"
                for axis in GRAPH_REQUIREMENTS["required_thruster_axes"]}
    relevant = required | {topic for topic in publishers
                           if topic.startswith(namespace + "/thrusters/")
                           and topic.endswith("_thrust_cmd")}
    observed = {topic: sorted(publishers.get(topic, [])) for topic in sorted(relevant)}
    failures = []
    if paths.count(expected) != 1:
        failures.append("expected exactly one " + expected + " node")
    if forbidden in paths:
        failures.append("legacy navigation node is running: " + forbidden)
    for topic, owners in observed.items():
        if owners != [expected]:
            failures.append(f"{topic}: expected sole publisher {expected}, observed {owners!r}")
    return {"namespace": namespace, "expected_publisher": expected,
            "nodes": paths, "publishers": observed,
            "passed": not failures, "failures": failures}


# 周期测量 ROS 图并累积历史；之后恢复正常也不能抹掉此前失败记录。
class GraphGuard:
    def __init__(self, probe, log):
        self.probe, self.log = probe, log
        self.observations = []
        self.last_wall = None

    # 按最多每半秒一次的策略获取节点和推进器发布端点，异常判定立即抛出。
    def check(self, force=False):
        now = time.monotonic()
        if (not force and self.last_wall is not None
                and now - self.last_wall < GRAPH_REQUIREMENTS["maximum_poll_interval_wall_s"]):
            return self.observations[-1]
        self.last_wall = now
        namespace = self.probe.namespace
        topics = {namespace + "/thrusters/" + axis + "_thrust_cmd"
                  for axis in GRAPH_REQUIREMENTS["required_thruster_axes"]}
        topics.update(topic for topic, _ in self.probe.get_topic_names_and_types()
                      if topic.startswith(namespace + "/thrusters/") and topic.endswith("_thrust_cmd"))
        publishers = {topic: [node_path(info.node_name, info.node_namespace)
                              for info in self.probe.get_publishers_info_by_topic(topic)]
                      for topic in sorted(topics)}
        observation = {**evaluate_graph(namespace, self.probe.get_node_names_and_namespaces(), publishers),
                       "wall_monotonic_s": now, "sim_time_s": self.probe.sim_time()}
        self.observations.append(observation)
        self.log.write("control_ownership_graph", **observation)
        if not observation["passed"]:
            raise GraphViolation("ROS graph ownership failed: " + "; ".join(observation["failures"]))
        return observation

    # 只有至少一次观测且所有历史检查通过，才返回整体 passed。
    def evidence(self):
        return {"requirements": GRAPH_REQUIREMENTS, "observation_count": len(self.observations),
                "passed": bool(self.observations) and all(row["passed"] for row in self.observations),
                "observations": self.observations}


# 正常启动发现完成后挂接保护并立即强制检查，失败不能靠重试隐藏。
def activate_graph_guard(probe, log):
    """Call after normal startup discovery; invalid ownership is never retried away."""
    guard = GraphGuard(probe, log)
    probe.graph_guard = guard
    guard.check(force=True)
    return guard
