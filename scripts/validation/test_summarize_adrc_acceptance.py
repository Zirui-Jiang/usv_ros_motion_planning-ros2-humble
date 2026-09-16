import importlib.util
import json
from pathlib import Path


_MODULE_PATH = Path(__file__).with_name("summarize_adrc_acceptance.py")
_SPEC = importlib.util.spec_from_file_location("summarize_adrc_acceptance", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
build_comparison = _MODULE.build_comparison
write_outputs = _MODULE.write_outputs


def _write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _trial(name, controller="adrc", scenario="straight", **overrides):
    row = {
        "trial": name,
        "controller": controller,
        "scenario": scenario,
        "fault": None,
        "result": "completed",
        "passed": True,
        "arrived": True,
        "duration_sim_s": 12.5,
        "cross_track_initial_m": {"rms": 0.4, "max": 0.9},
        "minimum_footprint_clearance_m": 1.2,
        "status": {
            "metrics": {"control_ms": {"p99": 8.0, "max": 13.0}},
            "counters": {"control_overruns": 2},
        },
    }
    row.update(overrides)
    return row


def _write_events(path: Path, requested="adrc", actual="adrc"):
    events = [
        {"event": "controller_verified", "requested": requested, "actual": actual},
        {"event": "thruster_command", "wall_monotonic_s": 1.0, "axis": "left", "normalized_command": 0.99},
        {"event": "goal_published", "wall_monotonic_s": 2.0},
        {"event": "thruster_command", "wall_monotonic_s": 3.0, "axis": "left", "normalized_command": -0.5},
        {"event": "thruster_command", "wall_monotonic_s": 4.0, "axis": "right", "normalized_command": 0.25},
        {"event": "acquisition_finished", "wall_monotonic_s": 5.0},
        {"event": "thruster_command", "wall_monotonic_s": 6.0, "axis": "right", "normalized_command": 1.0},
    ]
    path.write_text("".join(json.dumps(event) + "\n" for event in events), encoding="utf-8")


def test_missing_metrics_stay_null_and_failed_rows_are_counted(tmp_path):
    root = tmp_path / "run"
    failed = _trial(
        "adrc_turn_01",
        scenario="turn",
        result="blocked",
        passed=False,
        arrived=False,
        duration_sim_s=None,
        cross_track_initial_m=None,
        minimum_footprint_clearance_m=None,
        status=None,
    )
    _write_json(root / "summary.json", {"results": [failed], "run_finished": True})
    _write_json(root / "manifest.json", {"controller_config_sha256": "abc123"})
    _write_json(root / failed["trial"] / "summary.json", failed)
    (root / failed["trial"] / "events.jsonl").write_text("", encoding="utf-8")

    report = build_comparison([root])

    row = report["trials"][0]
    assert row["initial_path_rmse_m"] is None
    assert row["control_p99_ms"] is None
    assert row["output_thruster_peak_abs"] is None
    assert row["config_sha256"] == "abc123"
    assert report["groups"][0]["total_trials"] == 1
    assert report["groups"][0]["passed_trials"] == 0
    assert report["groups"][0]["arrived_trials"] == 0


def test_mismatched_actual_controller_prevents_verified_claim(tmp_path):
    root = tmp_path / "run"
    row = _trial("adrc_straight_01")
    _write_json(root / "summary.json", row)
    _write_events(root / "events.jsonl", requested="adrc", actual="pid")

    result = build_comparison([root])["trials"][0]

    assert result["controller_requested"] == "adrc"
    assert result["controller_actual"] == "pid"
    assert result["controller_verification_matches_claim"] is False


def test_deduplicates_matrix_and_individual_summary_and_uses_acquisition_samples(tmp_path):
    root = tmp_path / "run"
    row = _trial("adrc_straight_01")
    _write_json(root / "summary.json", {"results": [row], "run_finished": True})
    _write_json(root / row["trial"] / "summary.json", row)
    _write_events(root / row["trial"] / "events.jsonl")

    report = build_comparison([root, root / row["trial"]])

    assert len(report["trials"]) == 1
    assert report["trials"][0]["output_thruster_peak_abs"] == 0.5
    assert report["trials"][0]["output_thruster_mean_abs"] == 0.375
    assert report["trials"][0]["thruster_sample_count"] == 2


def test_group_timing_uses_max_reported_trial_p99_and_documents_raw_limit(tmp_path):
    roots = []
    for index, p99 in enumerate((8.0, 12.0), start=1):
        root = tmp_path / f"run{index}"
        row = _trial(f"adrc_straight_0{index}")
        row["status"]["metrics"]["control_ms"]["p99"] = p99
        _write_json(root / "summary.json", row)
        _write_events(root / "events.jsonl")
        roots.append(root)

    report = build_comparison(roots)

    group = report["groups"][0]
    assert group["max_trial_reported_p99_ms"] == 12.0
    assert "mean_trial_reported_p99_ms" not in group
    assert report["method"]["timing_semantics"]["raw_per_cycle_timing_available"] is False
    assert "reported window" in report["method"]["timing_semantics"]["note"]


def test_writes_csv_json_and_markdown_with_fault_row(tmp_path):
    root = tmp_path / "run"
    row = _trial("adrc_straight_fault_01", fault="input_timeout", result="failed", passed=False)
    _write_json(root / "summary.json", row)
    _write_events(root / "events.jsonl")
    output = tmp_path / "report"

    write_outputs(build_comparison([root]), output)

    assert {p.name for p in output.iterdir()} == {"trials.csv", "comparison.json", "comparison.md"}
    assert "input_timeout" in (output / "trials.csv").read_text(encoding="utf-8")
    assert "不据此声明性能优势" in (output / "comparison.md").read_text(encoding="utf-8")
