#!/bin/bash
# Build the usv_ros_motion_planning workspace with colcon (ROS 2 Humble).

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_ROOT}"

# Build all packages.
# CMAKE_EXPORT_COMPILE_COMMANDS makes each package emit a compile_commands.json
# (needed by clangd for accurate ROS 2 include resolution).
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --symlink-install

# Aggregate the per-package compile_commands.json into a single top-level one
# so clangd (configured with --compile-commands-dir=<ws>/build) finds it.
python3 - "${PROJECT_ROOT}" <<'PY'
import glob, json, os, sys
build = os.path.join(sys.argv[1], "build")
entries = []
for f in sorted(glob.glob(os.path.join(build, "*", "compile_commands.json"))):
    try:
        with open(f) as fh:
            entries.extend(json.load(fh))
    except Exception as exc:  # noqa: BLE001
        print(f"[build] skip {f}: {exc}")
with open(os.path.join(build, "compile_commands.json"), "w") as fh:
    json.dump(entries, fh, indent=2)
print(f"[build] wrote build/compile_commands.json ({len(entries)} entries)")
PY

echo
echo "[build] Done. Remember to: source install/setup.bash"
