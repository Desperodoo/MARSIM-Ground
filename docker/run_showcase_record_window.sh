#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SHOWCASE_LOG="${SHOWCASE_LOG:-/tmp/marsim_showcase_record.log}"
RECORD_AFTER_SECONDS="${RECORD_AFTER_SECONDS:-5}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/quad0_pcl_render_node/cloud_accum}"
POINTCLOUD_WAIT_TIMEOUT_SECONDS="${POINTCLOUD_WAIT_TIMEOUT_SECONDS:-60}"

find_existing_showcase() {
  pgrep -af 'trajectory_showcase_ground.launch|docker/run_showcase.sh' | grep -v "$$" || true
}

wait_for_pointcloud() {
  local elapsed=0

  while [ "${elapsed}" -lt "${POINTCLOUD_WAIT_TIMEOUT_SECONDS}" ]; do
    if bash -lc "set +u; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI=\${ROS_MASTER_URI:-http://localhost:11311}; timeout 2 rostopic echo -n 1 '${POINTCLOUD_TOPIC}' >/dev/null 2>&1"; then
      echo "Point cloud topic is ready: ${POINTCLOUD_TOPIC}"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Timed out waiting for point cloud topic: ${POINTCLOUD_TOPIC}" >&2
  return 1
}

EXISTING_SHOWCASE="$(find_existing_showcase)"
if [ -n "${EXISTING_SHOWCASE}" ]; then
  echo "An existing showcase-related process is already running. Please stop it before using the auto-record script." >&2
  printf '%s\n' "${EXISTING_SHOWCASE}" >&2
  exit 1
fi

"${SCRIPT_DIR}/run_showcase.sh" >"${SHOWCASE_LOG}" 2>&1 &
SHOWCASE_PID=$!

cleanup() {
  if kill -0 "${SHOWCASE_PID}" >/dev/null 2>&1; then
    echo "Stopping showcase process ${SHOWCASE_PID}"
    kill "${SHOWCASE_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

echo "Started showcase in background with PID ${SHOWCASE_PID}"
echo "Showcase log: ${SHOWCASE_LOG}"
echo "Waiting ${RECORD_AFTER_SECONDS}s for RViz to appear..."
sleep "${RECORD_AFTER_SECONDS}"
echo "Waiting for point cloud messages on ${POINTCLOUD_TOPIC}..."
wait_for_pointcloud

"${SCRIPT_DIR}/record_rviz_window.sh"
