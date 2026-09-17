#!/usr/bin/python3
"""Reproduce the PATH mismatch, then use main.sh for existing live acceptance.

Run with system Python in `bash -ic`, with ROS and the selected install sourced.
Only the acceptance harness's owned processes are started/stopped. Its navigation
criteria, probes, commands and Gazebo physics are unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "scripts/validation"))
import ros_acceptance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    wrapper = workspace / "install/usv_perception/lib/usv_perception/perception_mapping_node"
    probe_code = ("import sys,runpy; print(sys.executable,sys.version,flush=True); "
                  f"runpy.run_path({str(wrapper)!r},run_name='mapping_import_probe'); "
                  "print('MAPPING_IMPORT_OK')")
    probes = {}
    for name, command in (("path_python", ["/usr/bin/env", "python3"]),
                          ("ros_python", [sys.executable])):
        process = subprocess.run([*command, "-c", probe_code], capture_output=True,
                                 text=True, timeout=20)
        probes[name] = dict(command=[*command, "-c", probe_code],
                            exit_code=process.returncode, stdout=process.stdout, stderr=process.stderr)
    import usv_perception.launch_support
    installed_launch = Path(usv_perception.launch_support.__file__)
    source_launch = ROOT / "src/core/perception_module/usv_perception/usv_perception/launch_support.py"
    metadata = dict(python_on_path=shutil.which("python3"), harness_python=sys.executable,
                    installed_launch=str(installed_launch),
                    launch_source_matches_install=installed_launch.read_bytes() == source_launch.read_bytes(),
                    launch_sha256=hashlib.sha256(installed_launch.read_bytes()).hexdigest(),
                    probes=probes)
    with (EVIDENCE / "interpreter-probe.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2)
    assert probes["path_python"]["exit_code"] != 0, "The original incompatible-PATH condition was not reproduced"
    assert "rclpy._rclpy_pybind11" in probes["path_python"]["stderr"]
    assert probes["ros_python"]["exit_code"] == 0, probes["ros_python"]
    assert metadata["launch_source_matches_install"]
    print(json.dumps({key: value for key, value in metadata.items() if key != "probes"}), flush=True)

    original_owned_process = ros_acceptance.OwnedProcess

    class ScriptOwnedProcess(original_owned_process):
        def __init__(self, workspace, command, logfile, environment):
            assert command[:4] == ["ros2", "launch", "sim_env", "main.launch.py"], command
            routed_command = ["bash", str(ROOT / "scripts/main.sh"), *command[4:]]
            environment = dict(environment, USV_INSTALL_DIR=str(workspace / "install"))
            route = dict(command=routed_command, USV_INSTALL_DIR=environment["USV_INSTALL_DIR"],
                         python_on_path=shutil.which("python3", path=environment["PATH"]))
            (logfile.parent / "launcher-route.json").write_text(json.dumps(route, indent=2))
            super().__init__(workspace, routed_command, logfile, environment)

    ros_acceptance.OwnedProcess = ScriptOwnedProcess
    return ros_acceptance.main([
        "--workspace", str(workspace), "--distributed", "--mode", "python",
        "--controllers", "pid", "lqr", "--scenario", "straight", "--repeats", "1",
        "--domain-id", "220", "--startup-timeout", "45", "--output", str(args.output),
    ])


if __name__ == "__main__":
    raise SystemExit(main())
