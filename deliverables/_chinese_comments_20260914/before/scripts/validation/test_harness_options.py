"""ROS-free command-line provenance checks; no live results are generated."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import importlib.util
from types import SimpleNamespace


SCRIPT = Path(__file__).with_name("ros_acceptance.py")


def test_dry_run_records_resolved_configuration_hash_and_typed_overrides(tmp_path):
    config = tmp_path / "candidate.yaml"
    config.write_text("controller:\n  controller_name: pid\n", encoding="utf-8")
    output = tmp_path / "plan"
    process = subprocess.run([
        sys.executable, str(SCRIPT), "--workspace", str(tmp_path),
        "--output", str(output), "--dry-run", "--controller-config", str(config),
        "--parameter", "usv_simulation.trajectory_planner.los.switch_dist=1.0",
    ], capture_output=True, text=True)
    assert process.returncode == 0, process.stderr
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["mode"] == "dry_run"
    assert manifest["controller_config"] == str(config.resolve())
    assert manifest["controller_config_sha256"] == hashlib.sha256(config.read_bytes()).hexdigest()
    assert manifest["pre_goal_parameter_overrides"] == {"usv_simulation.trajectory_planner.los.switch_dist": 1.0}
    assert not (output / "summary.json").exists()


def test_missing_controller_config_fails_before_creating_result_directory(tmp_path):
    output = tmp_path / "missing_plan"
    process = subprocess.run([
        sys.executable, str(SCRIPT), "--workspace", str(tmp_path), "--output", str(output),
        "--dry-run", "--controller-config", str(tmp_path / "absent.yaml"),
    ], capture_output=True, text=True)
    assert process.returncode != 0
    assert "configuration does not exist" in process.stderr
    assert not output.exists()


def test_duplicate_parameter_assignments_are_rejected(tmp_path):
    process = subprocess.run([
        sys.executable, str(SCRIPT), "--workspace", str(tmp_path), "--output", str(tmp_path / "plan"),
        "--dry-run", "--parameter", "example=1.0", "--parameter", "example=2.0",
    ], capture_output=True, text=True)
    assert process.returncode != 0
    assert "duplicate parameter name" in process.stderr


def test_safety_boundary_stops_remaining_matrix_and_faults(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location("supervision_harness", SCRIPT)
    harness = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(harness)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(init=lambda **kwargs: None, ok=lambda: False))
    monkeypatch.setitem(sys.modules, "rclpy.signals", SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)))
    monkeypatch.setenv("ROS_DOMAIN_ID", "199")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    calls = []

    def failed_trial(*args, **kwargs):
        calls.append(args)
        return {"result": "safety_stopped", "passed": False, "safety_stopped": True,
                "reason": "negative_swept_clearance_bound"}

    monkeypatch.setattr(harness, "navigation_trial", failed_trial)
    output = tmp_path / "boundary"
    assert harness.main(["--workspace", str(tmp_path), "--output", str(output),
                         "--controllers", "pid", "lqr", "--repeats", "3", "--faults"]) == 1
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert len(calls) == 1 and len(summary["results"]) == 1
    assert summary["safety_stopped"] and not summary["run_finished"]
    assert not summary["interrupted"] and not summary["all_passed"]
    assert summary["completed"] == 0  # Unrun trials are not labelled completed.
