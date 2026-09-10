#!/usr/bin/env bash

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"   # CONTAINER

if ! docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "Container ${CONTAINER} is not running, start it with: bash docker/run.sh" >&2
    exit 1
fi

exec docker exec -it "${CONTAINER}" bash
