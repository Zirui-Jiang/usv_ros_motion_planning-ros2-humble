#!/usr/bin/env bash
# 单机 LaDDS 联调一键入口：同一终端启动 Gazebo 仿真，以及感知、决策、航控和 LaDDS 航路适配器；Ctrl+C 一起停止。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
sim_script="$script_dir/start_ladds_split_sim.bash"
nav_script="$script_dir/start_ladds_split_nav.bash"
sim_options=()
nav_options=()
launch_arguments=()

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options] [-- ROS_LAUNCH_ARGUMENTS...]

Starts both halves of the LaDDS joint test on this machine, in one terminal:
  simulation:  $sim_script
               (Gazebo, vessel model, ground truth on /otter/odom_gt)
  navigation:  $nav_script
               (perception, decision, control, LaDDS bridge and 0211 route adapter)
Ctrl+C stops both; press it again to force the shutdown.

Simulation options:
  --world NAME|FILE     Installed sim_env world or .world file (default: ocean)
  --headless BOOL       true or false (default: true)
LaDDS options:
  --backend native|tcp  native: Linux SDK .so; tcp: Windows relay
                        start_ladds_route_relay.ps1, started beforehand
                        (default: \$LADDS_BACKEND, else native)
  --library FILE        native only: Linux SDK .so (default: \$LADDS_LIBRARY_PATH)
  --relay-host HOST     tcp only: relay address (default: IPv4 default gateway)
  --relay-port PORT     tcp only: relay port (default: 17880)
Both halves:
  --install DIR         Workspace install (default: repository install/)
  --base-install DIR    Underlay loaded before the workspace install
  -h, --help            Show this help

Launch arguments after -- go to both halves, e.g. controller_name:=lqr.
ROS_DOMAIN_ID=42 and ROS_LOCALHOST_ONLY=1 are fixed by the two scripts.
Do not also run start_ladds_route_sim.bash, start_ladds_route.bash or the
split scripts in the same ROS domain.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --world|--headless)
            [[ $# -ge 2 ]] || fail "$1 requires a value"
            sim_options+=("$1" "$2"); shift 2 ;;
        --backend|--library|--relay-host|--relay-port)
            [[ $# -ge 2 ]] || fail "$1 requires a value"
            nav_options+=("$1" "$2"); shift 2 ;;
        --install|--base-install)
            [[ $# -ge 2 ]] || fail "$1 requires DIR"
            sim_options+=("$1" "$2"); nav_options+=("$1" "$2"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --) shift; launch_arguments=("$@"); break ;;
        *) fail "unknown option: $1 (put ros2 launch arguments after --)" ;;
    esac
done

if (( ${#launch_arguments[@]} )); then
    sim_options+=(-- "${launch_arguments[@]}")
    nav_options+=(-- "${launch_arguments[@]}")
fi
for script in "$sim_script" "$nav_script"; do
    [[ -r "$script" ]] || fail "missing start script: $script"
done
command -v setsid >/dev/null || fail 'setsid is unavailable (install util-linux)'
env --default-signal=INT,QUIT true 2>/dev/null \
    || fail 'env --default-signal is unavailable (needs coreutils >= 8.31)'

pids=()
names=()
stop_signal=INT

# 向两组进程发同一信号；每组在独立会话里，组号就是启动时记下的 pid。
signal_groups() {
    local pid
    for pid in "${pids[@]}"; do
        kill "-$1" -- "-$pid" 2>/dev/null || true
    done
}

# 停止过程中再按 Ctrl+C 逐级加重：INT → TERM → KILL。
escalate() {
    case "$stop_signal" in
        INT) stop_signal=TERM ;;
        *) stop_signal=KILL ;;
    esac
    printf '\nForcing shutdown with SIG%s...\n' "$stop_signal" >&2
    signal_groups "$stop_signal"
}

# The trap only interrupts wait -n; stopping runs outside the handler so a
# second Ctrl+C can still reach escalate().
trap ':' INT TERM HUP

printf 'LaDDS all-in-one: simulation + navigation/adapter, ROS domain 42. Ctrl+C stops both.\n'
# 两组各放进独立会话：终端 Ctrl+C 只到本脚本再统一转发，后台组也不会因读写终端被挂起。
# Bash starts background commands with SIGINT/SIGQUIT ignored; restore them so
# ros2 launch and every node react to the forwarded Ctrl+C.
setsid env --default-signal=INT,QUIT bash "$sim_script" "${sim_options[@]}" </dev/null &
pids+=("$!"); names+=(simulation)
setsid env --default-signal=INT,QUIT bash "$nav_script" "${nav_options[@]}" </dev/null &
pids+=("$!"); names+=(navigation/adapter)

status=0
wait -n || status=$?
# From here on always finish stopping, even if the terminal is already gone.
set +e
exited=""
for index in "${!pids[@]}"; do
    kill -0 "${pids[$index]}" 2>/dev/null || exited="${names[$index]}"
done
if [[ -n "$exited" ]]; then
    printf '\n%s exited with status %s; stopping the other half...\n' "$exited" "$status" >&2
else
    status=130
    printf '\nStopping simulation and navigation/adapter...\n' >&2
fi

trap escalate INT TERM HUP
signal_groups INT
for pid in "${pids[@]}"; do
    # Wait for the whole group, not only its leader, so no node outlives the script.
    while kill -0 -- "-$pid" 2>/dev/null; do
        wait "$pid" 2>/dev/null || sleep 0.5
    done
done
exit "$status"
