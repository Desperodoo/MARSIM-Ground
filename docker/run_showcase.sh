#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
USE_GPU="${USE_GPU:-auto}"
if [ "${ENABLE_RVIZ}" = "true" ]; then
  export DOCKER_RUN_AS_HOST_USER=true
fi
if [ "${USE_GPU}" = "auto" ]; then
  if docker_has_gpu; then
    USE_GPU=true
  else
    USE_GPU=false
  fi
fi
if [ "${USE_GPU}" = "true" ]; then
  export DOCKER_ENABLE_GPU=true
fi
MAP_NAME="${MAP_NAME:-${REPO_ROOT}/map_generator/resource/small_forest01cutoff.pcd}"
MAP_NAME="$(containerize_repo_path "${MAP_NAME}")"

run_in_container "source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
  SHOWCASE_INIT_ARGS=\$(python3 src/MARSIM_fuel/test_interface/scripts/multi_robot_trajectory_player.py \
  --print-init-states --map '${MAP_NAME}' --fixed-z 0.30 | tr '\n' ' ') && \
  roslaunch exploration_manager trajectory_showcase_ground.launch \
  enable_rviz:=${ENABLE_RVIZ} use_gpu:=${USE_GPU} map_name:=${MAP_NAME} \${SHOWCASE_INIT_ARGS}"
