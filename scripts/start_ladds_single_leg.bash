#!/usr/bin/env bash
# 旧入口：转发到 start_ladds_route.bash。
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ $# -ge 1 && ${1:0:1} != - ]]; then
    exec bash "$script_dir/start_ladds_route.bash" --relay-host "$1" "${@:2}"
fi
exec bash "$script_dir/start_ladds_route.bash" "$@"
