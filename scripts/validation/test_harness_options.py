"""ROS-free command-line provenance checks; no live results are generated."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


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
