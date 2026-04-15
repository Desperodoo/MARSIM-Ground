#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DISPLAY_NAME="${DISPLAY:-:0}"
WINDOW_NAME="${RVIZ_WINDOW_NAME:-multi_ground.rviz - RViz}"
DURATION_SECONDS="${DURATION_SECONDS:-180}"
FRAMERATE="${FRAMERATE:-20}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/recordings}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${OUTPUT_PATH:-${OUTPUT_DIR}/rviz_region_${TIMESTAMP}.mp4}"
WAIT_TIMEOUT_SECONDS="${WAIT_TIMEOUT_SECONDS:-20}"

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Missing required command: ${cmd}" >&2
    exit 1
  fi
}

extract_geometry() {
  local info="$1"
  local x y width height

  x="$(printf '%s\n' "${info}" | awk -F': *' '/Absolute upper-left X:/ {print $2; exit}')"
  y="$(printf '%s\n' "${info}" | awk -F': *' '/Absolute upper-left Y:/ {print $2; exit}')"
  width="$(printf '%s\n' "${info}" | awk -F': *' '/Width:/ {print $2; exit}')"
  height="$(printf '%s\n' "${info}" | awk -F': *' '/Height:/ {print $2; exit}')"

  if [ -z "${x}" ] || [ -z "${y}" ] || [ -z "${width}" ] || [ -z "${height}" ]; then
    return 1
  fi

  printf '%s %s %s %s\n' "${x}" "${y}" "${width}" "${height}"
}

wait_for_window() {
  local elapsed=0
  local info

  while [ "${elapsed}" -lt "${WAIT_TIMEOUT_SECONDS}" ]; do
    if info="$(xwininfo -display "${DISPLAY_NAME}" -name "${WINDOW_NAME}" 2>/dev/null)"; then
      printf '%s\n' "${info}"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "RViz window not found within ${WAIT_TIMEOUT_SECONDS}s: ${WINDOW_NAME}" >&2
  return 1
}

require_cmd ffmpeg
require_cmd xwininfo

mkdir -p "${OUTPUT_DIR}"

WINDOW_INFO="$(wait_for_window)"
read -r WINDOW_X WINDOW_Y WINDOW_WIDTH WINDOW_HEIGHT < <(extract_geometry "${WINDOW_INFO}")

echo "Recording RViz region ${WINDOW_WIDTH}x${WINDOW_HEIGHT}+${WINDOW_X}+${WINDOW_Y}"
echo "Output: ${OUTPUT_PATH}"
echo "Note: this is region capture on the host X11 desktop. If another window covers RViz, that overlay will also be recorded."

exec ffmpeg -y \
  -video_size "${WINDOW_WIDTH}x${WINDOW_HEIGHT}" \
  -framerate "${FRAMERATE}" \
  -f x11grab \
  -i "${DISPLAY_NAME}+${WINDOW_X},${WINDOW_Y}" \
  -t "${DURATION_SECONDS}" \
  -c:v libx264 \
  -preset veryfast \
  -pix_fmt yuv420p \
  "${OUTPUT_PATH}"
