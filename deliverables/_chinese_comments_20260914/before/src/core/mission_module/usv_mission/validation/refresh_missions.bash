#!/usr/bin/env bash
set -euo pipefail
# A refresh is a NEW build. Validated source/install/evidence is immutable.
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/build_missions.bash" "$@"
