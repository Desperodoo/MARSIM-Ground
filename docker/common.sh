#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${WS_ROOT:-${REPO_ROOT}/.docker_ws}"
IMAGE_NAME="${IMAGE_NAME:-marsim-fuel:noetic}"
HTTP_PROXY="${HTTP_PROXY:-http://172.18.196.129:7890}"
HTTPS_PROXY="${HTTPS_PROXY:-${HTTP_PROXY}}"
ALL_PROXY="${ALL_PROXY:-${HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,::1}"

prepare_workspace() {
  mkdir -p "${WS_ROOT}/src"
  if [ ! -e "${WS_ROOT}/src/MARSIM_fuel" ]; then
    ln -s "${REPO_ROOT}" "${WS_ROOT}/src/MARSIM_fuel"
  fi
}

docker_proxy_args() {
  cat <<EOF
-e HTTP_PROXY=${HTTP_PROXY}
-e HTTPS_PROXY=${HTTPS_PROXY}
-e ALL_PROXY=${ALL_PROXY}
-e NO_PROXY=${NO_PROXY}
EOF
}

x11_args() {
  if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    echo "-e DISPLAY=${DISPLAY}"
    echo "-v /tmp/.X11-unix:/tmp/.X11-unix:rw"
  fi
  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    echo "-e XAUTHORITY=${XAUTHORITY}"
    echo "-v ${XAUTHORITY}:${XAUTHORITY}:ro"
  fi
}

run_in_container() {
  prepare_workspace
  local -a args
  while IFS= read -r line; do
    [ -n "${line}" ] && args+=("${line}")
  done < <(docker_proxy_args)
  while IFS= read -r line; do
    [ -n "${line}" ] && args+=("${line}")
  done < <(x11_args)

  docker run --rm -it \
    --net=host \
    "${args[@]}" \
    -v "${WS_ROOT}:/root/marsim_ws" \
    -w /root/marsim_ws \
    "${IMAGE_NAME}" \
    bash -lc "$*"
}
