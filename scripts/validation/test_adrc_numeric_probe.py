import csv
import importlib.util
import json
from pathlib import Path
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/validation/adrc_numeric_probe.py"
CONFIG = ROOT / "src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
DYNAMICS = ROOT / "src/core/shared/usv_navigation/config/robots/otter/dynamics_otter.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("adrc_numeric_probe", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_numeric_probe_records_explicit_tracking_demands_and_evidence(tmp_path):
    output = tmp_path / "numeric-evidence"
    return_code = _module().main([
        "--controller-config", str(CONFIG), "--output", str(output)
    ])

    assert return_code == 0
    result = json.loads((output / "result.json").read_text(encoding="utf-8"))
    assert result["passed"] is True
    assert result["thresholds"] == {
        "speed_tail_mae_mps_max": 0.1,
        "heading_tail_mae_deg_max": 5.0,
        "command_abs_max": 1.0,
        "duration_s": 180.0,
        "tail_duration_s": 20.0,
        "dt_s": 0.1,
    }
    assert set(result["scenarios"]) == {"speed_1mps", "heading_plus_30deg", "heading_minus_30deg"}
    assert result["config"]["sha256"]
    assert result["model_config"]["path"] == str(DYNAMICS.resolve())
    assert result["model_config"]["sha256"]
    assert result["model_config"]["model"]["mass"] == 29.84
    assert result["sources"] and all(item["sha256"] for item in result["sources"])
    assert result["timing"]["definition"].startswith("wall-clock time spent in ADRCController.compute")

    with (output / "samples.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3 * 1800
    speed = next(row for row in rows if row["scenario"] == "speed_1mps")
    heading = next(row for row in rows if row["scenario"] == "heading_plus_30deg")
    assert float(speed["reference_speed_mps"]) == 1.0
    assert float(heading["reference_speed_mps"]) == 0.3
    assert float(heading["reference_heading_deg"]) == pytest.approx(30.0)


def test_numeric_probe_requires_a_new_output_directory(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    try:
        _module().main(["--controller-config", str(CONFIG), "--output", str(output)])
    except FileExistsError:
        pass
    else:
        raise AssertionError("numeric probe overwrote an existing evidence directory")


def test_custom_dynamics_file_changes_model_without_overriding_controller(tmp_path):
    values = yaml.safe_load(DYNAMICS.read_text(encoding="utf-8"))
    values["dynamics"]["mass"] = 31.25
    custom = tmp_path / "custom_dynamics.yaml"
    custom.write_text(yaml.safe_dump(values), encoding="utf-8")

    model, controller, resolved = _module()._load_configuration(CONFIG, custom)

    assert model.mass == 31.25
    assert model.max_force == 50.0
    assert controller["force_max"] == 40.0
    assert resolved["dynamics.mass"] == 31.25
