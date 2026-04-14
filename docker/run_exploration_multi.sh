#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
USE_GPU="${USE_GPU:-false}"
AUTOSTART="${AUTOSTART:-true}"
if [ "${ENABLE_RVIZ}" = "true" ]; then
  export DOCKER_RUN_AS_HOST_USER=true
fi
MAP_NAME="${MAP_NAME:-${REPO_ROOT}/map_generator/resource/small_forest01cutoff.pcd}"
MAP_NAME="$(containerize_repo_path "${MAP_NAME}")"

run_in_container "source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
  roslaunch exploration_manager exploration_multi_ground.launch \
  enable_rviz:=${ENABLE_RVIZ} use_gpu:=${USE_GPU} autostart:=${AUTOSTART} map_name:=${MAP_NAME}"
