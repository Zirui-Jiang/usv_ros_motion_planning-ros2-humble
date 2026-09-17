#!/usr/bin/env bash

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"   # CONTAINER

docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
echo "[stop] Container ${CONTAINER} stopped and removed."
