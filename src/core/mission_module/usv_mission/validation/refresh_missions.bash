#!/usr/bin/env bash
# 刷新验证采用新建工作区构建，复用 build_missions 的参数与既有结果保护。
set -euo pipefail
# A refresh is a NEW build. Validated source/install/evidence is immutable.
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/build_missions.bash" "$@"
