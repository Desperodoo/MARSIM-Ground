#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

BUILD_CMD="${BUILD_CMD:-catkin_make}"
run_in_container "source /opt/ros/noetic/setup.bash && ${BUILD_CMD}"
