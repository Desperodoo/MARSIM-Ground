#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${WS_ROOT:-${REPO_ROOT}/.docker_ws}"
IMAGE_NAME="${IMAGE_NAME:-marsim-fuel:noetic}"
CONTAINER_REPO_ROOT="${CONTAINER_REPO_ROOT:-/root/marsim_ws/src/MARSIM_fuel}"
HTTP_PROXY="${HTTP_PROXY:-http://172.18.196.129:7890}"
HTTPS_PROXY="${HTTPS_PROXY:-${HTTP_PROXY}}"
ALL_PROXY="${ALL_PROXY:-${HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,::1}"

docker_cmd() {
  if docker info >/dev/null 2>&1; then
    echo docker
    return
  fi
  if sudo -n docker info >/dev/null 2>&1; then
    echo "sudo -n docker"
    return
  fi
  echo "sudo docker"
}

prepare_workspace() {
  mkdir -p "${WS_ROOT}/src"
  if [ -L "${WS_ROOT}/src/MARSIM_fuel" ]; then
    rm -f "${WS_ROOT}/src/MARSIM_fuel"
  fi
}

docker_proxy_args() {
  printf '%s\n' \
    "-e" "HTTP_PROXY=${HTTP_PROXY}" \
    "-e" "HTTPS_PROXY=${HTTPS_PROXY}" \
    "-e" "ALL_PROXY=${ALL_PROXY}" \
    "-e" "NO_PROXY=${NO_PROXY}"
}

x11_args() {
  if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    printf '%s\n' "-e" "DISPLAY=${DISPLAY}"
    printf '%s\n' "-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw"
  fi
  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    printf '%s\n' "-e" "XAUTHORITY=${XAUTHORITY}"
    printf '%s\n' "-v" "${XAUTHORITY}:${XAUTHORITY}:ro"
  fi
}

docker_tty_args() {
  if [ -t 0 ] && [ -t 1 ]; then
    printf '%s\n' "-it"
  fi
}

containerize_repo_path() {
  local path="$1"
  if [[ "${path}" == "${REPO_ROOT}"* ]]; then
    printf '%s\n' "${CONTAINER_REPO_ROOT}${path#${REPO_ROOT}}"
    return
  fi
  printf '%s\n' "${path}"
}

run_in_container() {
  prepare_workspace
  local docker_bin
  docker_bin="$(docker_cmd)"
  local -a args
  while IFS= read -r line; do
    [ -n "${line}" ] && args+=("${line}")
  done < <(docker_proxy_args)
  while IFS= read -r line; do
    [ -n "${line}" ] && args+=("${line}")
  done < <(x11_args)
  while IFS= read -r line; do
    [ -n "${line}" ] && args+=("${line}")
  done < <(docker_tty_args)

  ${docker_bin} run --rm \
    --net=host \
    "${args[@]}" \
    -v "${WS_ROOT}:/root/marsim_ws" \
    -v "${REPO_ROOT}:${CONTAINER_REPO_ROOT}" \
    -w /root/marsim_ws \
    "${IMAGE_NAME}" \
    bash -lc "$*"
}
