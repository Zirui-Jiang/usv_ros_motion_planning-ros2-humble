# 矩阵调度器回归：使用可控子进程测试并发上限、停止派发、结果身份和证据完整性。
"""ROS-free orchestration tests; fake trial processes produce synthetic evidence."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace


def module():
    path = Path(__file__).with_name("mission_matrix.py")
    assert path.is_file(), "matrix scheduler must exist"
    spec = importlib.util.spec_from_file_location("mission_matrix", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


# 验证：默认 dry-run 规划 21 个独立 ROS 域且不创建子试验目录。
def test_dry_run_plans_21_unique_domains_without_creating_child_directories(tmp_path):
    scheduler = module()
    output = tmp_path / "matrix"
    assert scheduler.main(["--workspace", str(tmp_path), "--output", str(output), "--dry-run"]) == 0
    manifest = json.loads((output / "manifest.json").read_text())
    progress = json.loads((output / "progress.json").read_text())
    assert len(manifest["matrix"]) == 21
    assert {t["domain_id"] for t in manifest["matrix"]} == set(range(170, 191))
    assert progress["counts"]["not_run"] == 21 and progress["counts"]["running"] == 0
    assert all(t["result"] == "not_run" and not Path(t["output"]).exists() for t in manifest["matrix"])
    assert not progress["all_passed"]


# 验证：ROS 域编号溢出在创建输出之前被拒绝。
def test_domain_overflow_rejected_before_making_output(tmp_path):
    import pytest
    scheduler = module()
    output = tmp_path / "bad"
    with pytest.raises(SystemExit):
        scheduler.main(["--workspace", str(tmp_path), "--output", str(output), "--domain-id", "220", "--dry-run"])
    assert not output.exists()


def write_child(path):
    guard_spec = importlib.util.spec_from_file_location("test_task_guard", Path(__file__).with_name("task_graph_guard.py"))
    guard = importlib.util.module_from_spec(guard_spec)
    guard_spec.loader.exec_module(guard)
    observation = guard.evaluate_graph("otter", [("mission_manager", "/otter")], {
        f"/otter/thrusters/{axis}_thrust_cmd": ["/otter/mission_manager"] for axis in ("left", "right", "lateral")})
    graph = {"requirements": guard.GRAPH_REQUIREMENTS, "observation_count": 1,
             "observations": [observation], "passed": True}
    path.write_text('''import argparse,json,time,sys
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--workspace'); p.add_argument('--output'); p.add_argument('--scenarios')
p.add_argument('--repeats');p.add_argument('--domain-id');p.add_argument('--world');p.add_argument('--startup-timeout')
a=p.parse_args();out=Path(a.output);out.mkdir()
fingerprint=json.loads((Path(a.workspace)/'synthetic_fingerprint.json').read_text())
failed=a.scenarios=='station'
time.sleep(.03 if failed else .2)
graph=SYNTHETIC_GRAPH
trial=a.scenarios+'_01'
(out/'manifest.json').write_text(json.dumps({'source_before':fingerprint,'control_ownership_graph_observations':[{'trial':trial,**graph}]}))
result={'trial':trial,'scenario':a.scenarios,'repeat':1,'domain_id':int(a.domain_id),'result':'failed' if failed else 'passed','passed':not failed,'control_ownership_graph':graph}
summary={'results':[result],'all_passed':not failed,'source_unchanged':True,'installation_unchanged':True,'source_after':fingerprint}
(out/'summary.json').write_text(json.dumps(summary))
print('synthetic child completed',flush=True)
sys.exit(1 if failed else 0)
'''.replace("SYNTHETIC_GRAPH", repr(graph)), encoding="utf-8")


# 验证：已有子任务失败后停止派发，正在运行的独立任务仍可完成。
def test_failure_stops_pending_trials_and_running_child_finishes(tmp_path, monkeypatch):
    scheduler = module()
    fake = tmp_path / "fake_trial.py"
    write_child(fake)
    monkeypatch.setattr(scheduler, "HARNESS", fake)
    before = {"source_sha256": "synthetic-source", "install_sha256": "synthetic-install"}
    (tmp_path / "synthetic_fingerprint.json").write_text(json.dumps(before))
    monkeypatch.setattr(scheduler, "fingerprint", lambda workspace: before)
    output = tmp_path / "output"
    output.mkdir()
    args = SimpleNamespace(workspace=tmp_path, output=output, scenarios=["station", "standby"],
                           repeats=2, domain_id=170, jobs=2, world="ocean", startup_timeout=120)
    tasks = scheduler.plan_tasks(args)
    # Deliberately put the failing and slower passing child in the first batch.
    tasks.sort(key=lambda t: (t["matrix_repeat"], t["scenario"] == "standby"))
    summary = scheduler.run_matrix(args, tasks, before)
    assert [t["result"] for t in tasks] == ["failed", "passed", "not_run", "not_run"]
    assert summary["completed_trials"] == 2 and not summary["all_passed"]
    assert tasks[1]["measurement"]["matrix_repeat"] == 1
    assert "synthetic child completed" in Path(tasks[1]["stdout_log"]).read_text()
    progress = json.loads((output / "progress.json").read_text())
    assert progress["counts"] == {"passed": 1, "failed": 1, "blocked": 0, "not_run": 2, "running": 0}


# 验证：安装指纹发生变化或无法验证时子任务不能被标成通过。
def test_child_cannot_pass_with_changed_or_unverifiable_installation(tmp_path):
    scheduler = module()
    baseline = {"source_sha256": "source", "install_sha256": "install"}
    task = {"output": str(tmp_path), "scenario": "station", "matrix_repeat": 3, "domain_id": 171}
    (tmp_path / "manifest.json").write_text(json.dumps({"source_before": baseline}))
    child = {"results": [{"scenario": "station", "repeat": 1, "domain_id": 171, "result": "passed", "passed": True}],
             "all_passed": True, "source_unchanged": True, "installation_unchanged": True,
             "source_after": {"source_sha256": "source", "install_sha256": "changed"}}
    (tmp_path / "summary.json").write_text(json.dumps(child))
    result = scheduler.read_child_result(task, 0, baseline)
    assert result["result"] == "failed" and not result["passed"]
    assert "fingerprint" in result["reason"]


# 验证：成功矩阵遵守 jobs 上限并保留外层真实重复编号。
def test_success_matrix_respects_job_bound_and_retains_matrix_repeat(tmp_path, monkeypatch):
    scheduler = module()
    fake = tmp_path / "fake_trial.py"
    write_child(fake)
    monkeypatch.setattr(scheduler, "HARNESS", fake)
    before = {"source_sha256": "source", "install_sha256": "install"}
    (tmp_path / "synthetic_fingerprint.json").write_text(json.dumps(before))
    monkeypatch.setattr(scheduler, "fingerprint", lambda workspace: before)
    output = tmp_path / "output"
    output.mkdir()
    args = SimpleNamespace(workspace=tmp_path, output=output, scenarios=["standby"],
                           repeats=4, domain_id=170, jobs=2, world="ocean", startup_timeout=120)
    tasks = scheduler.plan_tasks(args)
    summary = scheduler.run_matrix(args, tasks, before)
    assert summary["all_passed"] and summary["completed_trials"] == 4
    assert tasks[2]["measurement"]["repeat"] == 1
    assert tasks[2]["measurement"]["matrix_repeat"] == 3
    events = [(t["started_unix_s"], 1) for t in tasks] + [(t["finished_unix_s"], -1) for t in tasks]
    running, maximum = 0, 0
    for _, delta in sorted(events):
        running += delta
        maximum = max(maximum, running)
    assert maximum == 2 and running == 0


# 验证：缺失结果标记阻塞，中断只向本调度器持有的子进程发信号。
def test_missing_summary_is_blocked_and_interrupt_signals_only_owned_children(tmp_path):
    import signal
    scheduler = module()
    task = {"output": str(tmp_path), "scenario": "station", "matrix_repeat": 1, "domain_id": 170}
    assert scheduler.read_child_result(task, 1, {})["result"] == "blocked"

    class Child:
        def __init__(self):
            self.signals = []

        def poll(self):
            return 130 if self.signals else None

        def send_signal(self, signum):
            self.signals.append(signum)

        def wait(self, timeout):
            assert 0 < timeout <= 60
            return 130

    children = [Child(), Child()]
    scheduler.interrupt_owned({str(i): (child, None, {}) for i, child in enumerate(children)})
    assert all(child.signals == [signal.SIGINT] for child in children)


# 验证：格式错误的子任务 manifest 标记阻塞，不使调度器崩溃。
def test_malformed_child_manifest_is_blocked_without_crashing_scheduler(tmp_path):
    scheduler = module()
    task = {"output": str(tmp_path), "scenario": "station", "matrix_repeat": 1, "domain_id": 170}
    (tmp_path / "manifest.json").write_text("[]")
    (tmp_path / "summary.json").write_text(json.dumps({"results": [
        {"scenario": "station", "repeat": 1, "domain_id": 170, "result": "passed", "passed": True}]}))
    assert scheduler.read_child_result(task, 0, {})["result"] == "blocked"
