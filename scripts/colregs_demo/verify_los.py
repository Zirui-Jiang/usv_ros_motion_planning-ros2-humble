"""Compare the demo's ordinary LOS port with the repository's Python planner.

Run with Python 3.12 + numpy and Node.js. The source planner is imported read-only;
no ROS installation is required. Report tolerances are absolute, per trajectory
field, and every sample is compared (not just the final position).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/core/decision_module/trajectory_planner/trajectory_planner/los_trajectory_planner.py"
sys.path[:0] = [
    str(ROOT / "src/core/shared/common"),
    str(ROOT / "src/core/decision_module/trajectory_planner"),
]

import numpy as np

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner.los_trajectory_planner import LosTrajectoryPlanner


FIELDS = ["time_s", "arc_length_m", "x_m", "y_m", "yaw_rad", "curvature_per_m", "speed_mps", "acceleration_mps2"]
ABS_TOLERANCE = 1e-9
CASES = [
    {"id": "straight", "path": [[0, 0, 0], [100, 0, 0]], "state": {}, "speed": 1.0, "options": {}},
    {"id": "cross_track", "path": [[0, 0, 0], [100, 0, 0]], "state": {"y": 4}, "speed": 1.0, "options": {"trajectory_time": 4.0}},
    {"id": "wrap_pi_positive", "path": [[0, 0, 0], [-80, -0.5, 0]], "state": {"psi": math.pi - 0.025}, "speed": 1.2, "options": {"trajectory_time": 5.0}},
    {"id": "wrap_pi_negative", "path": [[0, 0, 0], [-80, 0.5, 0]], "state": {"psi": -math.pi + 0.025}, "speed": 1.2, "options": {"trajectory_time": 5.0}},
    {"id": "turn_waypoints", "path": [[0, 0, 0], [20, 0, 0], [20, 20, 0], [0, 3, 0]], "state": {}, "speed": 1.7, "options": {}},
    {"id": "dense_waypoint_switch", "path": [[0, 0, 0], [0.1, 0, 0], [0.2, 0, 0], [0.2, 1, 0], [3, 1, 0]], "state": {}, "speed": 1.0, "options": {"trajectory_time": 1.0, "trajectory_time_interval": 0.5, "lookahead_distance": 1.0, "switch_dist": 5.0}},
    {"id": "near_goal_approach", "path": [[9.3, 10.25, 0], [10.49, 10.49, 0]], "state": {"x": 9.3, "y": 10.25}, "speed": 1.0, "options": {}},
    {"id": "already_at_goal", "path": [[0, 0, 0], [2, 3, 0]], "state": {"x": 2.1, "y": 3.1, "psi": 3.2}, "speed": 1.0, "options": {}},
    {"id": "duplicate_waypoints", "path": [[0, 0, 0], [0, 0, 0], [15, 0, 0], [15, 0, 1], [15, 15, 0]], "state": {"y": 1.5}, "speed": 1.3, "options": {"trajectory_time": 8.0}},
    {"id": "single_waypoint", "path": [[15, 8, 0]], "state": {"x": 1, "y": -2, "psi": 0.8}, "speed": 1.0, "options": {"trajectory_time": 5.0}},
    {"id": "substep_output_interval", "path": [[0, 0, 0], [100, 0, 0]], "state": {"psi": 0.2}, "speed": 1.0, "options": {"trajectory_time": 1.0, "trajectory_time_interval": 0.07}},
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def python_reference(case: dict) -> dict:
    planner = LosTrajectoryPlanner(USVModel(cruising_speed=case["speed"]), case["options"])
    result = planner.plan(case["path"], VesselState(**case["state"]))
    assert result.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED), case["id"]
    points = result.trajectory.points
    assert points.shape[1] == 8 and np.isfinite(points).all(), case["id"]
    assert np.all(np.diff(points[:, 0]) > 0), case["id"]
    assert np.all(np.diff(points[:, 1]) >= -1e-12), case["id"]
    assert np.all((points[:, 6] >= 0) & (points[:, 6] <= case["speed"] + 1e-12)), case["id"]
    if case["id"] == "straight":
        np.testing.assert_allclose(points[:, 3:6], 0, atol=1e-12)
    elif case["id"] == "cross_track":
        assert points[1, 4] < 0, "Cross-track correction should turn toward the route"
    elif case["id"] == "dense_waypoint_switch":
        np.testing.assert_allclose(points[1, [3, 4]], [0, 0], atol=1e-12)
        np.testing.assert_allclose(points[2, 4], 0.12, atol=1e-12)
    elif case["id"] == "near_goal_approach":
        assert 0 < points[0, 6] < case["speed"] and points[-1, 6] == 0
    elif case["id"] == "already_at_goal":
        assert len(points) == 2 and np.all(points[:, 6:] == 0)
    return {"id": case["id"], "python_status": result.status.value, "rows": points.tolist()}


NODE_DRIVER = r"""
const fs = require('fs');
const engine = require(process.argv[1]);
if (typeof engine.losPlan !== 'function') throw new Error('engine.losPlan is missing');
const input = JSON.parse(fs.readFileSync(0, 'utf8'));
// Python constructs a VesselState dataclass, whose omitted fields default to 0.
const result = input.map(c => ({id: c.id, rows: engine.losPlan(c.path, {x:0,y:0,psi:0,u:0,v:0,r:0,stamp:0,...c.state}, c.speed, c.options)
  .map(p => [p.t, p.s, p.x, p.y, p.yaw, p.curvature, p.speed, p.acceleration])}));
process.stdout.write(JSON.stringify(result));
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, default=Path(__file__).with_name("engine.js"))
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("validation") / "los_report.json")
    parser.add_argument("--reference-only", action="store_true", help="Only validate and save Python reference fixtures; no JS pass is claimed.")
    args = parser.parse_args()
    source_hash = sha256(SOURCE)
    references = [python_reference(case) for case in CASES]
    assert sha256(SOURCE) == source_hash, "The Python LOS source changed during verification"
    report = {
        "source": str(SOURCE), "source_sha256": source_hash,
        "python_version": sys.version.split()[0], "numpy_version": np.__version__,
        "absolute_tolerance": ABS_TOLERANCE, "fields": FIELDS,
        "case_count": len(CASES), "passed": False,
    }
    if args.reference_only:
        report.update({"mode": "python_reference_only", "comparison_pending": True, "cases": references})
    else:
        assert args.engine.is_file(), f"Engine not generated yet: {args.engine}"
        node = shutil.which("node")
        assert node, "Node.js is required for the JavaScript comparison"
        run = subprocess.run([node, "-e", NODE_DRIVER, str(args.engine.resolve())], input=json.dumps(CASES), text=True, capture_output=True)
        assert run.returncode == 0, f"JavaScript runner failed:\n{run.stderr}"
        actual = json.loads(run.stdout)
        assert len(actual) == len(references), "Case count mismatch"
        cases = []
        for expected, observed in zip(references, actual, strict=True):
            assert expected["id"] == observed["id"]
            py_rows, js_rows = np.asarray(expected["rows"]), np.asarray(observed["rows"], dtype=float)
            assert py_rows.shape == js_rows.shape, f"{expected['id']}: row shape {py_rows.shape} != {js_rows.shape}"
            np.testing.assert_allclose(js_rows, py_rows, rtol=0, atol=ABS_TOLERANCE, err_msg=expected["id"])
            errors = np.max(np.abs(py_rows - js_rows), axis=0)
            cases.append({"id": expected["id"], "python_status": expected["python_status"], "rows_compared": len(py_rows), "max_abs_error_by_field": dict(zip(FIELDS, errors.tolist())), "passed": True})
        report.update({"mode": "python_js_numeric_comparison", "engine_sha256": sha256(args.engine), "node_version": subprocess.check_output([node, "--version"], text=True).strip(), "passed": True, "cases": cases, "max_abs_error": max(max(c["max_abs_error_by_field"].values()) for c in cases)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(args.output.resolve()), "mode": report["mode"], "cases": len(CASES), "passed": report["passed"], "max_abs_error": report.get("max_abs_error")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
