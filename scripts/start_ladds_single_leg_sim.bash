#!/usr/bin/env bash
# 在隔离 ROS/Gazebo 域中启动普通导航主入口，供 LaDDS 0211 单航段 4 节任务验证。
set -euo pipefail
# Select Humble's system Python for this process tree, including under Conda.
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
profile_dir="$script_dir/validation/config"
base_install="${LADDS_BASE_INSTALL:-/home/usv/module-layout-20260914/install}"
profile_install="${LADDS_SINGLE_LEG_INSTALL:-/tmp/usv_ladds_single_leg_20260915/install}"
world_argument="${LADDS_SINGLE_LEG_WORLD:-ocean}"
headless="${LADDS_SINGLE_LEG_HEADLESS:-true}"
launch_arguments=()
# Gazebo's setup.sh assigns its default master URI. Capture the caller's value
# before sourcing it, or select this profile's isolated port when none exists.
requested_gazebo_master_uri="${GAZEBO_MASTER_URI:-http://127.0.0.1:11347}"

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options] [-- ROS_LAUNCH_ARGUMENTS...]

Options:
  --base-install DIR  Complete simulation/navigation install
                      (default: $base_install)
  --install DIR       LaDDS/single-leg overlay install
                      (default: $profile_install)
  --world NAME|FILE   Installed sim_env world name or readable .world file
                      (default: $world_argument)
  --headless BOOL     true or false (default: $headless)
  -h, --help          Show this help

Recommended new overlay: /tmp/usv_ladds_single_leg_20260915/install
Environment defaults, used only when the caller has not set them:
  ROS_DOMAIN_ID=42
  ROS_LOCALHOST_ONLY=1
  GAZEBO_MASTER_URI=http://127.0.0.1:11347
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --base-install) [[ $# -ge 2 ]] || fail '--base-install requires DIR'; base_install="$2"; shift 2 ;;
        --install) [[ $# -ge 2 ]] || fail '--install requires DIR'; profile_install="$2"; shift 2 ;;
        --world) [[ $# -ge 2 ]] || fail '--world requires NAME or FILE'; world_argument="$2"; shift 2 ;;
        --headless) [[ $# -ge 2 ]] || fail '--headless requires true or false'; headless="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --) shift; launch_arguments=("$@"); break ;;
        *) fail "unknown option: $1 (put ros2 launch arguments after --)" ;;
    esac
done

[[ "$headless" == true || "$headless" == false ]] || fail '--headless must be true or false'
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS Humble setup is unavailable'
[[ -r "$base_install/setup.bash" ]] || fail "base install has no setup.bash: $base_install"
[[ -r "$profile_install/setup.bash" ]] || fail "overlay install has no setup.bash: $profile_install"
[[ -r "$profile_dir/ladds_single_leg_dynamics.yaml" ]] || fail 'single-leg dynamics profile is missing'
[[ -r "$profile_dir/ladds_single_leg_world.py" ]] || fail 'single-leg world copier is missing'
for executable in python3 realpath mktemp; do
    command -v "$executable" >/dev/null || fail "$executable is unavailable"
done

# Profile-owned arguments stay fixed so all three business modules receive the
# same validated configuration and the original vessel pose is preserved.
for argument in "${launch_arguments[@]}"; do
    case "$argument" in
        world:=*|headless:=*|dynamics_config:=*|controller_config:=*|trajectory_config:=*|planner_config:=*|robot_x:=*|robot_y:=*|robot_z:=*|robot_yaw:=*)
            fail "launch argument is owned by this profile: $argument" ;;
    esac
done

# ROS setup scripts may read unset variables; restore strict nounset afterwards.
set +u
source /opt/ros/humble/setup.bash
source "$base_install/setup.bash"
source "$profile_install/setup.bash"
if [[ -r /usr/share/gazebo/setup.sh ]]; then
    source /usr/share/gazebo/setup.sh
fi
set -u

command -v ros2 >/dev/null || fail 'ros2 is unavailable after loading the selected installs'

sim_share="$(ros2 pkg prefix --share sim_env)" || fail 'sim_env is absent from the selected installs'
navigation_share="$(ros2 pkg prefix --share usv_navigation)" \
    || fail 'usv_navigation is absent from the selected installs'

if [[ -r "$world_argument" ]]; then
    source_world="$(realpath -e -- "$world_argument")"
elif [[ "$world_argument" != */* ]]; then
    world_name="${world_argument%.world}"
    source_world="$sim_share/worlds/$world_name.world"
    [[ -r "$source_world" ]] || fail "installed world is unreadable: $source_world"
else
    fail "world path is unreadable: $world_argument"
fi

runtime_dir="$(mktemp -d "${TMPDIR:-/tmp}/ladds-single-leg-world.XXXXXX")"
cleanup() { rm -rf -- "$runtime_dir"; }
trap cleanup EXIT INT TERM
runtime_world="$runtime_dir/ladds_single_leg.world"
python3 "$profile_dir/ladds_single_leg_world.py" \
    --source "$source_world" --output "$runtime_world"

# sim_env appends '.world' to this argument. Strip the suffix so an arbitrary
# absolute world path resolves to the generated independent copy exactly once.
world_for_launch="${runtime_world%.world}"
: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
GAZEBO_MASTER_URI="$requested_gazebo_master_uri"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY GAZEBO_MASTER_URI

controller_config="$navigation_share/config/robots/otter/controller_params_otter.yaml"
trajectory_config="$navigation_share/config/trajectory_planner/trajectory_planner_params.yaml"
planner_config="$navigation_share/config/path_planner/common_planner_params.yaml"
for config in "$controller_config" "$trajectory_config" "$planner_config"; do
    [[ -r "$config" ]] || fail "navigation config is unreadable: $config"
done

printf 'Single-leg simulation profile:\n'
printf '  base install: %s\n  overlay: %s\n  source world: %s\n' \
    "$base_install" "$profile_install" "$source_world"
printf '  runtime world: %s\n  ROS domain: %s\n  Gazebo master: %s\n' \
    "$runtime_world" "$ROS_DOMAIN_ID" "$GAZEBO_MASTER_URI"
printf '  speed profile: task requests 4 kn; dynamics.velocity_max=2.1 m/s; cruising_speed remains 1.0 m/s\n'

ros2 launch sim_env main.launch.py \
    "world:=$world_for_launch" "headless:=$headless" \
    "dynamics_config:=$profile_dir/ladds_single_leg_dynamics.yaml" \
    "controller_config:=$controller_config" \
    "trajectory_config:=$trajectory_config" \
    "planner_config:=$planner_config" \
    "${launch_arguments[@]}"
