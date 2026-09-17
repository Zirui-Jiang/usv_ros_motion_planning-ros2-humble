"""Exercise the install-only audit in subprocesses without source PYTHONPATH."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import pytest


SCRIPT = Path(__file__).with_name("verify_distributed_install.py")


def audit_module():
    spec = importlib.util.spec_from_file_location("module_layout_install_audit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("dependency", ["sim_env", "usv_control"])
def test_business_manifest_rejects_transitive_simulation_or_peer_dependency(tmp_path, dependency):
    for name, child in (("usv_decision", "usv_navigation"), ("usv_navigation", dependency)):
        manifest = tmp_path / name / "share" / name / "package.xml"
        manifest.parent.mkdir(parents=True)
        manifest.write_text(f"<package><name>{name}</name><exec_depend>{child}</exec_depend></package>")
    passed, evidence = audit_module().installed_dependencies(tmp_path, "usv_decision")
    assert passed is False
    assert evidence["forbidden_reachable"] == [dependency]


@pytest.mark.parametrize("body, expected", [
    ("from usv_perception.perception_monitor import main\nif __name__ == '__main__':\n    main()\n", True),
    ("# from usv_perception.perception_monitor import main\n", False),
    ("from usv_perception.perception_monitor import main\n", False),
    ("from old_package.perception_monitor import main\nif __name__ == '__main__':\n    main()\n", False),
])
def test_cmake_wrapper_must_actually_call_the_new_entrypoint(body, expected):
    assert audit_module().wrapper_calls_entrypoint(body, "usv_perception.perception_monitor:main") is expected


@pytest.mark.parametrize("discoverable", [False, True])
def test_removed_package_check_rejects_sourced_overlay(monkeypatch, tmp_path, discoverable):
    spec = importlib.util.spec_from_file_location("install_audit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    packages = ModuleType("ament_index_python.packages")

    class PackageNotFoundError(LookupError):
        pass

    stale_prefix = tmp_path / "old-install" / "mmwave_perception"

    def get_package_prefix(package):
        assert package == "mmwave_perception"
        if discoverable:
            return str(stale_prefix)
        raise PackageNotFoundError(package)

    packages.PackageNotFoundError = PackageNotFoundError
    packages.get_package_prefix = get_package_prefix
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)
    passed, evidence = module.removed_package_discovery("mmwave_perception")
    assert passed is not discoverable
    assert evidence["discoverable"] is discoverable
    if discoverable:
        assert evidence["prefix"] == str(stale_prefix.resolve())


def run_audit(install, output):
    environment = os.environ.copy()
    # A sourced ROS installation also uses PYTHONPATH. Keep it only for the
    # opt-in real-install test; the deliberate mismatch needs no ROS imports.
    if "USV_DISTRIBUTED_INSTALL" not in environment:
        environment.pop("PYTHONPATH", None)
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--install", str(install), "--output", str(output)],
        env=environment, capture_output=True, text=True, timeout=90,
    )


def test_mismatched_install_produces_failure_evidence(tmp_path):
    install = tmp_path / "empty-install"
    install.mkdir()
    output = tmp_path / "audit.json"
    result = run_audit(install, output)
    assert result.returncode == 1, result.stdout + result.stderr
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["install"] == str(install.resolve())
    assert report["all_passed"] is False
    failed = {row["check"] for row in report["checks"] if not row["passed"]}
    assert "usv_navigation:discovery" in failed
    assert "entrypoint:decision_node" in failed
    assert "launch:main" in failed
    assert "configuration:decision_control" in failed
    assert {row["check"] for row in report["checks"]} >= {
        "mmwave_perception:absent", "usv_radar_costmap:absent", "usv_radar_interfaces:absent"}


def test_existing_report_is_never_overwritten(tmp_path):
    output = tmp_path / "audit.json"
    output.write_text("previous evidence\n", encoding="utf-8")
    result = run_audit(tmp_path, output)
    assert result.returncode != 0
    assert output.read_text(encoding="utf-8") == "previous evidence\n"


def test_relative_install_is_rejected(tmp_path):
    result = run_audit(Path("relative-install"), tmp_path / "audit.json")
    assert result.returncode == 2
    assert "absolute" in result.stderr.lower()


@pytest.mark.skipif(not os.environ.get("USV_DISTRIBUTED_INSTALL"),
                    reason="Requires a clean shell sourced from the selected isolated install")
def test_real_sourced_install(tmp_path):
    output = tmp_path / "audit.json"
    result = run_audit(Path(os.environ["USV_DISTRIBUTED_INSTALL"]), output)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["all_passed"] is True
    assert report["checks"] and all(row["passed"] for row in report["checks"])
    assert any(row["check"] == "type_support:NavigationReference" for row in report["checks"])
    assert any(row["check"] == "configuration:decision_control" for row in report["checks"])
    absent = {row["check"] for row in report["checks"]
              if row["passed"] and row.get("discoverable") is False}
    assert absent == {"mmwave_perception:absent", "usv_radar_costmap:absent",
                      "usv_radar_interfaces:absent"}
