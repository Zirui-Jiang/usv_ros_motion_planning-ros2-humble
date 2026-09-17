#!/usr/bin/env bash

###############################################################################
# Build (if missing) and start the ros_motion_planning_pro dev container.
###############################################################################
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"   # PROJECT_NAME, VERSION_NAME, IMAGE, CONTAINER, BASE_IMAGE
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE_DIR="${PROJECT_ROOT}/docker/${VERSION_NAME}"

# ---- optional confirmation (skip with -y) ---------------------------------
if [[ $# -gt 0 && $1 == "-y" ]]; then
    shift
else
    read -rp "Build or start container ${CONTAINER}, confirm with (y/Y): " ans
    [[ ! ${ans:-} =~ ^[yY]$ ]] && { echo "Cancelled."; exit 1; }
fi

# ---- build the image only if it does not exist yet -------------------------
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[run] Image ${IMAGE} not found, building..."
    docker build \
        --build-arg PROJECT_NAME="${PROJECT_NAME}" \
        --build-arg VERSION_NAME="${VERSION_NAME}" \
        --build-arg BASE_IMAGE="${BASE_IMAGE}" \
        -t "${IMAGE}" -f "${DOCKERFILE_DIR}/Dockerfile" "${PROJECT_ROOT}"
    # reclaim dangling layers left by the build
    docker image prune -f >/dev/null 2>&1 || true
else
    echo "[run] Image ${IMAGE} exists, skipping build."
fi

# ---- keep a single container instance --------------------------------------
if docker ps -a --filter "name=^/${CONTAINER}$" --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "[run] Stopping and removing existing container ${CONTAINER} ..."
    docker rm -f "${CONTAINER}" >/dev/null
fi

# ---- X11 forwarding --------------------------------------------------------
# Use a per-user xauth file: /tmp/.docker.xauth may exist as a root-owned
# directory from a prior root-level run, which we cannot touch.
DISPLAY_ENV="${DISPLAY:-:0}"
XAUTH="/tmp/.docker.xauth.${USER}"
# Copy this display's cookie into our xauth file (xauth creates it). Fall back
# to an empty file + xhost grant if there is no cookie.
if ! xauth nlist "${DISPLAY_ENV}" 2>/dev/null | sed -e 's/^..../ffff/' \
        | xauth -f "${XAUTH}" nmerge - >/dev/null 2>&1; then
    : > "${XAUTH}" 2>/dev/null || true
fi
chmod a+r "${XAUTH}" 2>/dev/null || true
xhost +local:root >/dev/null 2>&1 || true

# ---- launch ----------------------------------------------------------------
echo "[run] Starting container ${CONTAINER} ..."
docker run -it -d \
    --name "${CONTAINER}" \
    --privileged \
    --ipc=host \
    --network=host \
    -e DISPLAY="${DISPLAY_ENV}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="${XAUTH}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTH}:${XAUTH}:ro" \
    -v "${PROJECT_ROOT}:/${PROJECT_NAME}:rw" \
    -w /${PROJECT_NAME} \
    --restart=no \
    "${IMAGE}"

echo
echo "[run] Done. Enter the container with: bash docker/into.sh"
