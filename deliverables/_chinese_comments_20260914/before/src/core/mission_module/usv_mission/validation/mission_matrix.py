#!/usr/bin/env python3
"""Bounded parallel Gazebo mission trials; the parent never initialises ROS.

Run from the sourced ROS/selected workspace environment. A failure stops new
dispatches while already running independent trials finish. Child evidence is
never rewritten; matrix_repeat identifies repeats in the aggregate results.
"""
import argparse
from collections import Counter
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
_spec = importlib.util.spec_from_file_location(
    "_usv_mission_acceptance", Path(__file__).with_name("mission_acceptance.py"))
_acceptance = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_acceptance)
SCENARIOS, fingerprint = _acceptance.SCENARIOS, _acceptance.fingerprint
from task_graph_guard import GRAPH_REQUIREMENTS
from task_source_layout import find_source_root

HARNESS = Path(__file__).with_name("mission_acceptance.py")
STATES = ("passed", "failed", "blocked", "not_run", "running")
# Start longer observations first; each child still runs exactly one scenario.
PRIORITY = {name: index for index, name in enumerate((
    "orbit_obstacle", "station_disturbance", "orbit_cw", "orbit_ccw",
    "standby_obstacle", "station", "standby"))}


def write_json(path, payload):
    """Atomic replacement lets external progress readers parse every snapshot."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def plan_tasks(args):
    pairs = [(scenario, repeat) for scenario in args.scenarios for repeat in range(1, args.repeats + 1)]
    pairs.sort(key=lambda item: (PRIORITY[item[0]], item[1]))
    tasks = []
    for index, (scenario, repeat) in enumerate(pairs):
        trial_id = f"{scenario}_{repeat:02d}"
        output = args.output / trial_id
        domain = args.domain_id + index
        command = [sys.executable, str(HARNESS), "--workspace", str(args.workspace),
                   "--output", str(output), "--scenarios", scenario, "--repeats", "1",
                   "--domain-id", str(domain), "--world", args.world,
                   "--startup-timeout", str(args.startup_timeout)]
        tasks.append({"trial_id": trial_id, "scenario": scenario, "matrix_repeat": repeat,
                      "domain_id": domain, "output": str(output),
                      "stdout_log": str(args.output / f"{trial_id}.log"), "command": command,
                      "result": "not_run", "passed": False})
    return tasks


def read_child_result(task, returncode, baseline):
    """Promote success only with matching measured evidence and provenance."""
    output = Path(task["output"])
    try:
        summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        if not isinstance(summary, dict) or not isinstance(manifest, dict):
            raise ValueError("child summary and manifest must be JSON objects")
        before, after = manifest.get("source_before", {}), summary.get("source_after", {})
        if not isinstance(before, dict) or not isinstance(after, dict):
            raise ValueError("child fingerprints must be JSON objects")
        measurements = summary.get("results", [])
        if len(measurements) != 1 or not isinstance(measurements[0], dict):
            raise ValueError("expected exactly one completed child trial")
        measured = dict(measurements[0])
        json.dumps(measured, allow_nan=False)
        if (measured.get("scenario") != task["scenario"] or measured.get("repeat") != 1
                or measured.get("domain_id") != task["domain_id"]):
            raise ValueError("child scenario, repeat or domain does not match the dispatched trial")
    except (OSError, ValueError, TypeError, AttributeError) as error:
        return {"result": "blocked", "passed": False, "returncode": returncode,
                "reason": f"child evidence unavailable or invalid: {error}"}
    measured["matrix_repeat"] = task["matrix_repeat"]
    result = {"result": "failed", "passed": False, "returncode": returncode,
              "measurement": measured, "child_summary": str(output / "summary.json")}
    provenance = all(isinstance(baseline.get(key), str) and bool(baseline[key])
                     and baseline[key] == before.get(key) == after.get(key)
                     for key in ("source_sha256", "install_sha256"))
    if not provenance or summary.get("source_unchanged") is not True or summary.get("installation_unchanged") is not True:
        result["reason"] = "source/installation fingerprint changed or could not be verified"
    elif measured.get("result") == "blocked":
        result.update(result="blocked", reason=measured.get("reason", "child trial blocked"))
    elif not graph_evidence_verified(measured, manifest):
        result["reason"] = "task-node ROS graph ownership was not measured and verified"
    elif (returncode == 0 and summary.get("all_passed") is True
          and measured.get("result") == "passed" and measured.get("passed") is True):
        result.update(result="passed", passed=True, reason="child measurements and provenance verified")
    else:
        result["reason"] = measured.get("reason", f"child did not pass (exit {returncode})")
    return result


def graph_evidence_verified(measured, manifest):
    graph = measured.get("control_ownership_graph", {})
    if not isinstance(graph, dict):
        return False
    observations = graph.get("observations", [])
    if (graph.get("passed") is not True or not isinstance(observations, list) or not observations
            or graph.get("observation_count") != len(observations)
            or graph.get("requirements") != GRAPH_REQUIREMENTS
            or any(not isinstance(row, dict) or row.get("passed") is not True for row in observations)):
        return False
    records = manifest.get("control_ownership_graph_observations", [])
    return isinstance(records, list) and any(
        isinstance(row, dict) and row.get("trial") == measured.get("trial")
        and {key: value for key, value in row.items() if key != "trial"} == graph
        for row in records)


def progress(tasks, stopped_dispatch=False, interrupted=False):
    counts = Counter(task["result"] for task in tasks)
    return {"updated_unix_s": time.time(), "planned_trials": len(tasks),
            "completed_trials": sum(counts[name] for name in ("passed", "failed", "blocked")),
            "counts": {name: counts[name] for name in STATES},
            "running_trials": [task["trial_id"] for task in tasks if task["result"] == "running"],
            "dispatch_stopped": stopped_dispatch, "interrupted": interrupted,
            "all_passed": False, "results": tasks}


def interrupt_owned(running):
    """Signal only owned harness PIDs; allow their finally blocks to clean ROS.

    One shared deadline bounds cleanup for the entire batch, not 60 s per child.
    An unresponsive harness is terminated after 55 s; incomplete cleanup is
    explicitly recorded instead of claiming all descendant processes exited.
    """
    deadline = time.monotonic() + 60.
    for process, _, task in running.values():
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except (OSError, ValueError) as error:
                task["cleanup_warning"] = str(error)
    while any(process.poll() is None for process, _, _ in running.values()) and time.monotonic() < deadline - 5:
        time.sleep(.1)
    for process, _, task in running.values():
        if process.poll() is None:
            task["cleanup_warning"] = "harness did not finish SIGINT cleanup; owned PID terminated, inspect launch logs"
            process.terminate()
    while any(process.poll() is None for process, _, _ in running.values()) and time.monotonic() < deadline - 2:
        time.sleep(.1)
    for process, _, task in running.values():
        if process.poll() is None:
            process.kill()
        try:
            process.wait(timeout=max(.001, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            task["cleanup_warning"] = "owned harness PID still running after cleanup deadline"


def run_matrix(args, tasks, before):
    running, next_index = {}, 0
    stopped_dispatch, interrupted = False, False
    orchestration_error = None
    last_progress = 0.

    def publish():
        write_json(args.output / "progress.json", progress(tasks, stopped_dispatch, interrupted))

    try:
        while running or (next_index < len(tasks) and not stopped_dispatch):
            # Inspect all completed children before deciding whether to dispatch.
            for trial_id, (process, logfile, task) in list(running.items()):
                returncode = process.poll()
                if returncode is None:
                    continue
                logfile.close()
                task.update(read_child_result(task, returncode, before), finished_unix_s=time.time())
                del running[trial_id]
                stopped_dispatch |= task["result"] != "passed"
                print(json.dumps({"trial_id": trial_id, "result": task["result"], "reason": task["reason"]}), flush=True)
                publish()
            while not stopped_dispatch and len(running) < args.jobs and next_index < len(tasks):
                task = tasks[next_index]
                next_index += 1
                logfile = Path(task["stdout_log"]).open("w", encoding="utf-8")
                try:
                    environment = dict(os.environ, ROS_DOMAIN_ID=str(task["domain_id"]), ROS_LOCALHOST_ONLY="1")
                    process = subprocess.Popen(task["command"], cwd=args.workspace, env=environment,
                                               stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
                except OSError as error:
                    logfile.close()
                    task.update(result="blocked", reason=f"could not launch child: {error}", finished_unix_s=time.time())
                    stopped_dispatch = True
                    publish()
                    break
                task.update(result="running", pid=process.pid, started_unix_s=time.time())
                running[task["trial_id"]] = (process, logfile, task)
                print(json.dumps({"trial_id": task["trial_id"], "result": "running", "pid": process.pid}), flush=True)
                publish()
            if time.monotonic() - last_progress >= 2:
                publish()
                last_progress = time.monotonic()
            if running:
                time.sleep(.1)
    except KeyboardInterrupt:
        interrupted = stopped_dispatch = True
        publish()
        interrupt_owned(running)
    except Exception as error:
        stopped_dispatch = True
        orchestration_error = f"{type(error).__name__}: {error}"
        interrupt_owned(running)
    finally:
        for process, logfile, task in running.values():
            logfile.close()
            task.update(read_child_result(task, process.poll(), before), finished_unix_s=time.time())
            if interrupted and task["result"] == "passed":
                task.update(result="blocked", passed=False, reason="matrix interrupted during this trial")
        for task in tasks:
            if task["result"] == "not_run":
                task["reason"] = "matrix interrupted" if interrupted else "dispatch stopped after an earlier unsuccessful trial"
        after = fingerprint(args.workspace)
        summary = progress(tasks, stopped_dispatch, interrupted)
        summary.update(source_before=before, source_after=after,
                       orchestration_error=orchestration_error,
                       source_unchanged=before["source_sha256"] == after["source_sha256"],
                       installation_unchanged=before["install_sha256"] == after["install_sha256"])
        summary["all_passed"] = (not interrupted and orchestration_error is None
                                 and all(t["result"] == "passed" for t in tasks)
                                 and summary["source_unchanged"] and summary["installation_unchanged"])
        summary["result"] = ("passed" if summary["all_passed"] else "failed"
                             if summary["counts"]["failed"] or not summary["source_unchanged"]
                             or not summary["installation_unchanged"] else "blocked")
        write_json(args.output / "summary.json", summary)
        write_json(args.output / "progress.json", summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=find_source_root())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--domain-id", type=int, default=170)
    parser.add_argument("--world", default="ocean")
    parser.add_argument("--startup-timeout", type=float, default=120)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    args.workspace, args.output = args.workspace.resolve(), args.output.resolve()
    count = len(args.scenarios) * args.repeats
    if args.jobs < 1 or args.repeats < 1:
        parser.error("jobs and repeats must be positive")
    if len(set(args.scenarios)) != len(args.scenarios):
        parser.error("scenarios must not be duplicated")
    if not 0 <= args.domain_id <= args.domain_id + count - 1 <= 232:
        parser.error("selected trial ROS domain range must fit within [0,232]")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("startup timeout must be finite and positive")
    if not args.workspace.is_dir() or not HARNESS.is_file():
        parser.error("workspace and mission_acceptance.py must exist")
    if args.output.exists():
        parser.error("output already exists; choose a new evidence directory")
    if not args.dry_run and os.name != "posix":
        parser.error("live matrix requires WSL/Linux with ROS and the selected install sourced")
    args.output.mkdir(parents=True)
    before = fingerprint(args.workspace)
    tasks = plan_tasks(args)
    manifest = {"classification": "dry_run" if args.dry_run else "actual parallel ROS/Gazebo mission matrix",
                "launch": {"package": "usv_mission", "file": "mission_sim.launch.py"},
                "control_ownership_graph_requirements": GRAPH_REQUIREMENTS,
                "workspace": str(args.workspace), "world": args.world, "jobs": args.jobs,
                "created_unix_s": time.time(), "matrix": tasks, "source_before": before}
    write_json(args.output / "manifest.json", manifest)
    write_json(args.output / "progress.json", progress(tasks))
    if args.dry_run:
        summary = dict(progress(tasks), result="not_run")
        write_json(args.output / "summary.json", summary)
        print(json.dumps({"result": "not_run", "planned_trials": len(tasks)}), flush=True)
        return 0
    summary = run_matrix(args, tasks, before)
    print(json.dumps({key: summary[key] for key in ("result", "counts", "all_passed")}), flush=True)
    return 130 if summary["interrupted"] else 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
