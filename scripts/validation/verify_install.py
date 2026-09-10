#!/usr/bin/env python3
"""Audit the sourced installation, without adding source paths to sys.path.

Run in a new ROS shell after sourcing only the selected install/setup.bash.
This audit requires colcon's default isolated installation layout.
The JSON describes installation checks, not ROS graph or closed-loop success.
"""
import argparse
from datetime import datetime, timezone
import importlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


MODULES = {
    "common": "common.models",
    "path_planner": "path_planner.path_planner_core",
    "path_processor": "path_processor.path_processor",
    "trajectory_planner": "trajectory_planner.los_trajectory_planner",
    "controller": "controller.controller_factory",
    "perception": "perception.mapping_node",
    "sim_env": "sim_env.simulation",
}
EXECUTABLES = {
    "perception": ["perception_mapping_node"],
    "sim_env": ["sim_env_node", "fake_localization_node", "costmap_support_node"],
}


def within(path, parent):
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def audit(source, install):
    from ament_index_python.packages import get_package_prefix

    rows = []

    def record(name, passed, **evidence):
        rows.append(dict(check=name, passed=bool(passed), **evidence))

    expected = {}
    for manifest in sorted((source / "src").rglob("package.xml")):
        tree = ET.parse(manifest).getroot()
        expected[tree.findtext("name")] = tree.findtext("export/build_type")
    record("source_package_inventory", bool(expected), packages=expected)
    for package, build_type in expected.items():
        try:
            prefix = Path(get_package_prefix(package)).resolve()
            installed_manifest = prefix / "share" / package / "package.xml"
            actual_type = ET.parse(installed_manifest).getroot().findtext("export/build_type")
            expected_prefix = (install / package).resolve()
            record(package + ":discovery", prefix == expected_prefix and actual_type == build_type,
                   prefix=str(prefix), expected_build_type=build_type, actual_build_type=actual_type)
        except Exception as error:
            record(package + ":discovery", False, error=str(error))
            continue
        if package in MODULES:
            try:
                module = importlib.import_module(MODULES[package])
                origin = Path(module.__file__).resolve()
                record(package + ":import", within(origin, install), origin=str(origin))
            except Exception as error:
                record(package + ":import", False, error=str(error))
        for name in EXECUTABLES.get(package, []):
            executable = prefix / "lib" / package / name
            record(package + ":" + name, executable.is_file() and os.access(executable, os.X_OK),
                   path=str(executable))
            if package == "sim_env" and name != "sim_env_node" and executable.is_file():
                links = subprocess.run(["ldd", str(executable)], capture_output=True, text=True, timeout=15)
                legacy = re.findall(r"\blib(?:common|path_planner|path_processor|trajectory_planner|controller|sim_env)\.so\b",
                                    links.stdout)
                record(name + ":linkage", links.returncode == 0 and "not found" not in links.stdout and not legacy,
                       exit_code=links.returncode, obsolete_libraries=legacy,
                       stdout=links.stdout, stderr=links.stderr)
    for package in MODULES:
        if package == "sim_env":
            continue
        prefix = install / package
        legacy = [str(p) for p in prefix.rglob("*.so*")]
        record(package + ":no_cpp_algorithm_libraries", prefix.is_dir() and not legacy,
               prefix_exists=prefix.is_dir(), libraries=legacy)
    return {"classification": "installed package/import/executable/linkage audit",
            "collected_utc": datetime.now(timezone.utc).isoformat(),
            "python_executable": sys.executable, "python_version": sys.version,
            "source": str(source), "install": str(install),
            "environment": {name: os.environ.get(name) for name in ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")},
            "checks": rows, "all_passed": bool(rows) and all(row["passed"] for row in rows)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--install", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="new JSON evidence file")
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output already exists")
    report = audit(args.source.resolve(), args.install.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps({"output": str(args.output), "all_passed": report["all_passed"],
                      "failed_checks": [row for row in report["checks"] if not row["passed"]]}, ensure_ascii=False))
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
