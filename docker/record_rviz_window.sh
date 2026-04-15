#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DISPLAY_NAME="${DISPLAY:-:0}"
WINDOW_NAME="${RVIZ_WINDOW_NAME:-multi_ground.rviz - RViz}"
DURATION_SECONDS="${DURATION_SECONDS:-180}"
FRAMERATE="${FRAMERATE:-20}"
BITRATE_KBPS="${BITRATE_KBPS:-4000}"
WAIT_TIMEOUT_SECONDS="${WAIT_TIMEOUT_SECONDS:-30}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/recordings}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${OUTPUT_PATH:-${OUTPUT_DIR}/rviz_window_${TIMESTAMP}.mp4}"

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Missing required command: ${cmd}" >&2
    exit 1
  fi
}

find_window_id() {
  local elapsed=0
  local wininfo
  local window_id

  while [ "${elapsed}" -lt "${WAIT_TIMEOUT_SECONDS}" ]; do
    if wininfo="$(xwininfo -display "${DISPLAY_NAME}" -name "${WINDOW_NAME}" 2>/dev/null)"; then
      window_id="$(printf '%s\n' "${wininfo}" | awk '/Window id:/ {print $4; exit}')"
      if [ -n "${window_id}" ]; then
        printf '%s\n' "${window_id}"
        return 0
      fi
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "RViz window not found within ${WAIT_TIMEOUT_SECONDS}s: ${WINDOW_NAME}" >&2
  return 1
}

require_cmd gst-launch-1.0
require_cmd xwininfo
if ! gst-inspect-1.0 x264enc >/dev/null 2>&1; then
  echo "Missing required GStreamer plugin: x264enc" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

WINDOW_ID_HEX="$(find_window_id)"
WINDOW_ID_DEC="$((WINDOW_ID_HEX))"

echo "Recording RViz window ${WINDOW_NAME}"
echo "Window id: ${WINDOW_ID_HEX}"
echo "Output: ${OUTPUT_PATH}"
echo "This capture reads the RViz X11 window directly, so overlapping VS Code windows should not appear in the output."

exec gst-launch-1.0 -e \
  ximagesrc display-name="${DISPLAY_NAME}" xid="${WINDOW_ID_DEC}" use-damage=false show-pointer=false num-buffers="$((DURATION_SECONDS * FRAMERATE))" ! \
  video/x-raw,framerate="${FRAMERATE}/1" ! \
  videoconvert ! \
  x264enc speed-preset=ultrafast tune=zerolatency bitrate="${BITRATE_KBPS}" key-int-max="$((FRAMERATE * 2))" byte-stream=false aud=true ! \
  video/x-h264,stream-format=avc,alignment=au ! \
  mp4mux ! \
  filesink location="${OUTPUT_PATH}"
