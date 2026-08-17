#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COMPETITION_WS="${COMPETITION_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source "$SCRIPT_DIR/car_source_env.sh"

IMAGE_TOPIC="/left_wrist_camera/camera/color/image_raw"
PERCEPTION_NODE="/wrist_traffic_perception"
STATUS_TOPIC="/perception/traffic_rules_status"
LOG_DIR="$COMPETITION_WS/log"

topic_has_publisher() {
  ros2 topic info "$IMAGE_TOPIC" 2>/dev/null \
    | grep -Eq 'Publisher count:[[:space:]]*[1-9]'
}

perception_is_running() {
  ros2 node list 2>/dev/null | grep -Fxq "$PERCEPTION_NODE"
}

wait_until_ready() {
  local description="$1"
  local check_function="$2"
  for _ in {1..30}; do
    if "$check_function"; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for $description" >&2
  return 1
}

show_status() {
  echo "Wrist vision status: $STATUS_TOPIC"
  echo "Close this window or press Ctrl+C to stop status display only."
  exec ros2 topic echo "$STATUS_TOPIC"
}

if [[ "${1:-}" == "--status" ]]; then
  show_status
fi

if [[ -n "${1:-}" ]]; then
  echo "usage: $0" >&2
  exit 2
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: no graphical display detected; run this script in the Ubuntu desktop terminal." >&2
  exit 1
fi
if [[ ! -x /opt/ros/humble/lib/rqt_image_view/rqt_image_view ]]; then
  echo "ERROR: rqt_image_view is not installed." >&2
  exit 1
fi
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "ERROR: gnome-terminal is required for the recognition status window." >&2
  exit 1
fi

mkdir -p "$LOG_DIR"

if topic_has_publisher; then
  echo "Wrist camera is already online; reusing it."
  if ! perception_is_running; then
    PERCEPTION_LOG="$LOG_DIR/wrist_traffic_perception_manual.log"
    echo "Starting traffic recognition; log: $PERCEPTION_LOG"
    setsid -f ros2 run competition_perception wrist_traffic_node \
      --ros-args \
      --params-file "$COMPETITION_WS/config/perception/wrist_traffic_rules.yaml" \
      >"$PERCEPTION_LOG" 2>&1 </dev/null
    wait_until_ready "traffic recognition" perception_is_running
  fi
elif perception_is_running; then
  echo "ERROR: recognition is running but the wrist camera has no image publisher." >&2
  echo "Inspect the existing camera service before retrying; no duplicate camera was started." >&2
  exit 1
else
  CAMERA_LOG="$LOG_DIR/wrist_traffic_test_launch.log"
  echo "Starting wrist camera and traffic recognition; log: $CAMERA_LOG"
  setsid -f ros2 launch competition_perception wrist_traffic.launch.py \
    >"$CAMERA_LOG" 2>&1 </dev/null
  wait_until_ready "wrist camera image" topic_has_publisher
  wait_until_ready "traffic recognition" perception_is_running
fi

echo "Wrist camera is ready: $IMAGE_TOPIC"
gnome-terminal \
  --title="Wrist traffic recognition status" \
  -- "$SCRIPT_DIR/start_wrist_vision_test.sh" --status

echo "Opening the wrist camera image. Closing the image window will leave the camera online."
exec /usr/bin/python3 \
  /opt/ros/humble/lib/rqt_image_view/rqt_image_view \
  "$IMAGE_TOPIC"
