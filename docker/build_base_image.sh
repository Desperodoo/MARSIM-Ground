#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
HTTP_PROXY="${HTTP_PROXY:-http://172.18.196.129:7890}"
HTTPS_PROXY="${HTTPS_PROXY:-${HTTP_PROXY}}"
ALL_PROXY="${ALL_PROXY:-${HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,::1}"
source "${SCRIPT_DIR}/common.sh"

$(docker_cmd) build \
  --build-arg HTTP_PROXY="${HTTP_PROXY}" \
  --build-arg HTTPS_PROXY="${HTTPS_PROXY}" \
  --build-arg ALL_PROXY="${ALL_PROXY}" \
  --build-arg NO_PROXY="${NO_PROXY}" \
  -f "${SCRIPT_DIR}/Dockerfile.base" \
  -t marsim-focal-noetic:local \
  "${REPO_ROOT}"
