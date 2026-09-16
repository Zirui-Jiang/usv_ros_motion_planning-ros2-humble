# 验收工具迁移回归：检查任务节点图证据、目录定位、源码指纹及本地验证模块来源。
"""ROS-free regressions for migration provenance and graph ownership gates."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

DIRECTORY = Path(__file__).resolve().parent


def load(name):
    sys.path.insert(0, str(DIRECTORY))
    spec = importlib.util.spec_from_file_location("test_core_" + name, DIRECTORY / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clean_graph():
    return [("mission_manager", "/otter"), ("perception_mapping", "/otter")], {
        f"/otter/thrusters/{axis}_thrust_cmd": ["/otter/mission_manager"]
        for axis in ("left", "right", "lateral")}


# 验证：合法图中唯一任务节点独占三个命令话题。
def test_valid_task_node_owns_all_command_topics():
    guard = load("task_graph_guard")
    nodes, publishers = clean_graph()
    assert guard.evaluate_graph("/otter/", nodes, publishers)["passed"]
    # Another vessel's legacy navigation does not affect this namespace.
    nodes.append(("usv_simulation", "/other_boat"))
    assert guard.evaluate_graph("otter", nodes, publishers)["passed"]


# 验证：缺失、多余或错误控制发布者都成为显式失败。
@pytest.mark.parametrize("kind", ["missing", "duplicate", "legacy", "missing_command",
                                 "extra_publisher", "wrong_owner", "extra_axis"])
def test_ownership_violations_are_explicit_failures(kind):
    guard = load("task_graph_guard")
    nodes, publishers = clean_graph()
    if kind == "missing":
        nodes.pop(0)
    elif kind == "duplicate":
        nodes.append(("mission_manager", "/otter"))
    elif kind == "legacy":
        nodes.append(("usv_simulation", "/otter"))
    elif kind == "missing_command":
        publishers.pop("/otter/thrusters/left_thrust_cmd")
    elif kind == "extra_publisher":
        publishers["/otter/thrusters/left_thrust_cmd"].append("/otter/mission_manager")
    elif kind == "wrong_owner":
        publishers["/otter/thrusters/right_thrust_cmd"] = ["/other_boat/mission_manager"]
    else:
        publishers["/otter/thrusters/bow_thrust_cmd"] = ["/otter/external_control"]
    result = guard.evaluate_graph("otter", nodes, publishers)
    assert not result["passed"] and result["failures"]


# 验证：实时图守卫保存每次观测，后来恢复不能擦除历史失败。
def test_runtime_graph_guard_latches_failure_and_records_observations(monkeypatch):
    guard = load("task_graph_guard")
    nodes, publishers = clean_graph()
    rows = []
    probe = SimpleNamespace(namespace="/otter", graph_guard=None,
        get_node_names_and_namespaces=lambda: nodes,
        get_topic_names_and_types=lambda: [(topic, ["std_msgs/msg/Float32"]) for topic in publishers],
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(
            node_name=name.rsplit("/", 1)[1], node_namespace=name.rsplit("/", 1)[0])
            for name in publishers.get(topic, [])], sim_time=lambda: 12.)
    log = SimpleNamespace(write=lambda event, **values: rows.append({"event": event, **values}))
    counter = guard.activate_graph_guard(probe, log)
    assert probe.graph_guard is counter and counter.evidence()["passed"]
    nodes.append(("usv_simulation", "/otter"))
    with pytest.raises(guard.GraphViolation, match="legacy navigation node"):
        counter.check(force=True)
    nodes.pop()
    counter.check(force=True)
    assert len(rows) == 3 and rows[-1]["passed"]
    assert not counter.evidence()["passed"]  # Never retry a failure into a pass.


# 验证：不同深度的验证目录都能向上找到源码根。
def test_source_root_is_discovered_at_variable_depth(tmp_path):
    finder = load("task_source_layout")
    location = tmp_path / "src/core/mission_module/usv_mission/validation/deeper/script.py"
    location.parent.mkdir(parents=True)
    location.touch()
    assert finder.find_source_root(location) == tmp_path
    with pytest.raises(ValueError, match="No source checkout"):
        finder.find_source_root(tmp_path.parent / "no_source_checkout" / "missing.py")


# 验证：任务验证源码只计入指纹一次，安装文件变化独立改变安装指纹。
def test_fingerprints_include_task_validation_once_and_installed_task(tmp_path):
    acceptance = load("mission_acceptance")
    files = {"src/core/mission_module/usv_mission/validation/mission_acceptance.py": "harness v1",
             "src/core/mission_module/usv_mission/validation/scenarios.json": '{"threshold": 0.5}',
             "src/core/mission_module/usv_mission/usv_mission/node.py": "node v1",
             "install/usv_mission/lib/usv_mission/mission_node": "entry v1",
             "install/usv_mission/share/usv_mission/launch/mission_sim.launch.py": "launch v1",
             "install/usv_mission_interfaces/lib/service_type.so": "generated rosidl",
             "scripts/validation/ros_acceptance.py": "legacy harness"}
    for relative, content in files.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    before = acceptance.fingerprint(tmp_path)
    assert len(before["files"]) == 4 and len(before["installed_files"]) == 3
    assert before["source_sha256"] == hashlib.sha256(json.dumps(before["files"], sort_keys=True).encode()).hexdigest()
    (tmp_path / "src/core/mission_module/usv_mission/validation/scenarios.json").write_text('{"threshold": 1.0}')
    after = acceptance.fingerprint(tmp_path)
    assert before["source_sha256"] != after["source_sha256"]
    assert before["install_sha256"] == after["install_sha256"]
    (tmp_path / "install/usv_mission/lib/usv_mission/mission_node").write_text("entry v2")
    assert acceptance.fingerprint(tmp_path)["install_sha256"] != before["install_sha256"]


# 验证：根级同名模块缓存不会重定向本任务包的验收工具。
def test_root_validation_module_cache_cannot_redirect_task_harness(monkeypatch):
    for name in ("ros_acceptance", "metrics", "mission_acceptance"):
        monkeypatch.setitem(sys.modules, name, SimpleNamespace(unrelated_cached_module=True))
    acceptance = load("mission_acceptance")
    matrix = load("mission_matrix")
    assert acceptance.make_probe.__module__ == "task_ros_acceptance"
    assert acceptance.distribution.__module__ == "task_metrics"
    assert Path(matrix._acceptance.__file__).resolve() == DIRECTORY / "mission_acceptance.py"
    assert matrix.SCENARIOS == acceptance.SCENARIOS


# 验证：旧成功记录缺少实际 ROS 图所有权证据不能提升为成功。
def test_matrix_cannot_promote_old_success_without_graph_evidence(tmp_path):
    matrix = load("mission_matrix")
    baseline = {"source_sha256": "source", "install_sha256": "install"}
    task = {"output": str(tmp_path), "scenario": "station", "matrix_repeat": 1, "domain_id": 171}
    (tmp_path / "manifest.json").write_text(json.dumps({"source_before": baseline}))
    child = {"results": [{"trial": "station_01", "scenario": "station", "repeat": 1,
                          "domain_id": 171, "result": "passed", "passed": True}],
             "all_passed": True, "source_unchanged": True, "installation_unchanged": True,
             "source_after": baseline}
    (tmp_path / "summary.json").write_text(json.dumps(child))
    result = matrix.read_child_result(task, 0, baseline)
    assert result["result"] == "failed" and not result["passed"]
    assert "ROS graph ownership" in result["reason"]


# 验证：导航入口指向核心任务节点并保留公开参数命名契约。
def test_navigation_entry_targets_core_and_keeps_parameter_contract():
    harness = load("task_ros_acceptance")
    args = harness.arguments(["--output", "unused", "--dry-run"])
    assert args.launch_file == "mission_sim.launch.py"
    # Configuration names are deliberately compatible with existing algorithms.
    text = (DIRECTORY / "task_ros_acceptance.py").read_text(encoding="utf-8")
    assert 'prefix = "usv_simulation.controller."' in text
    assert '"ros2", "run", "usv_mission", "mission_node"' in text
