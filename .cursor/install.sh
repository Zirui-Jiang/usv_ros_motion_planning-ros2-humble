#!/usr/bin/env bash
###############################################################################
# Repository bootstrap for the USV ROS 2 Humble navigation workspace.
#
# Runs after the source tree is checked out. Idempotent: safe to re-run. It
# resolves ROS dependencies with rosdep and builds the colcon workspace so the
# simulation and tests can run. Heavy, stable system packages are installed in
# the Dockerfile, not here.
###############################################################################
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# Resolve declared ROS package dependencies from the source tree.
# rosdep is already initialized in the official ROS images; only update here.
rosdep update --rosdistro humble || true
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y || true

# Python packages used by the pure-NumPy navigation algorithms and tests.
python3 -m pip install --no-cache-dir --break-system-packages numpy pyyaml matplotlib pytest >/dev/null 2>&1 \
    || python3 -m pip install --no-cache-dir numpy pyyaml matplotlib pytest

# Build the workspace. --base-paths src keeps package discovery inside src/ so
# stale exported copies under deliverables/ are not collected.
colcon build \
    --base-paths src \
    --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# Auto-source the freshly built workspace overlay in future interactive shells.
printf '%s\n' \
    "source /opt/ros/humble/setup.bash" \
    "[ -f '${PROJECT_ROOT}/install/setup.bash' ] && source '${PROJECT_ROOT}/install/setup.bash'" \
    > /etc/profile.d/usv_ros_overlay.sh
chmod +x /etc/profile.d/usv_ros_overlay.sh

echo "[install] Workspace build complete. Overlay: ${PROJECT_ROOT}/install/setup.bash"
