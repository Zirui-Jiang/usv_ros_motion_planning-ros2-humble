#!/usr/bin/env bash
# 多航点仿真轻量入口：复用经过验证的单航段仿真 profile，只切换安装区和 ROS 隔离域。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
single_sim="$script_dir/start_ladds_single_leg_sim.bash"
# Use the repository install even if the shell still has an old single-leg path.
LADDS_SINGLE_LEG_INSTALL="${LADDS_ROUTE_INSTALL:-$(cd -- "$script_dir/.." && pwd -P)/install}"
: "${LADDS_BASE_INSTALL:=$HOME/.local/share/usv_ros_motion_planning/base/install}"
: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
export LADDS_SINGLE_LEG_INSTALL LADDS_BASE_INSTALL ROS_DOMAIN_ID ROS_LOCALHOST_ONLY

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    cat <<EOF
Usage: bash ${0##*/} [single-leg simulation options]

Starts the ordinary navigation/Gazebo composition through:
  $single_sim

Route defaults:
  base install:  $LADDS_BASE_INSTALL
  workspace:     $LADDS_SINGLE_LEG_INSTALL
  ROS domain:    $ROS_DOMAIN_ID
  localhost:     $ROS_LOCALHOST_ONLY

Forwarded options include --world NAME|FILE, --headless true|false,
--base-install DIR, --install DIR, and extra launch arguments after --.
EOF
    exit 0
fi

[[ -r "$single_sim" ]] || { printf 'ERROR: missing simulation profile: %s\n' "$single_sim" >&2; exit 2; }
exec bash "$single_sim" "$@"
