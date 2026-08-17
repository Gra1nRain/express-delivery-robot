#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COMPETITION_WS="${COMPETITION_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source "$SCRIPT_DIR/car_source_env.sh"

IMAGE_TOPIC="/left_wrist_camera/camera/color/image_raw"
DISPLAY_TOPIC="/perception/wrist_traffic_annotated"
PERCEPTION_NODE="/wrist_traffic_perception"
STATUS_TOPIC="/perception/traffic_rules_status"
TRAFFIC_LIGHT_NODE="/traffic_light_recognition"
TRAFFIC_LIGHT_ENABLE_TOPIC="/perception/traffic_light_enable"
LOG_DIR="$COMPETITION_WS/log"
WRIST_USB_DEVICE="/sys/bus/usb/devices/2-3.3.2"
STARTED_PROCESS_GROUP=""

image_stream_is_live() {
  timeout 3 ros2 topic echo --once --field header "$IMAGE_TOPIC" \
    >/dev/null 2>&1
}

perception_is_running() {
  ros2 node list 2>/dev/null | grep -Fxq "$PERCEPTION_NODE"
}

traffic_light_is_running() {
  ros2 node list 2>/dev/null | grep -Fxq "$TRAFFIC_LIGHT_NODE"
}

wait_until_ready() {
  local description="$1"
  local check_function="$2"
  for _ in {1..15}; do
    if "$check_function"; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for $description" >&2
  return 1
}

managed_launch_pids() {
  pgrep -u "$USER" -f \
    '/opt/ros/humble/bin/ros2 launch competition_perception wrist_traffic\.launch\.py$' \
    || true
}

stop_process_group() {
  local process_group_pid="$1"
  kill -TERM -- "-$process_group_pid" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "$process_group_pid" 2>/dev/null || return 0
    sleep 0.2
  done
  kill -KILL -- "-$process_group_pid" 2>/dev/null || true
}

cleanup_failed_start() {
  if [[ -n "$STARTED_PROCESS_GROUP" ]]; then
    stop_process_group "$STARTED_PROCESS_GROUP"
  fi
}

start_managed_process() {
  setsid "$@" &
  STARTED_PROCESS_GROUP=$!
  trap cleanup_failed_start EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
}

keep_started_process() {
  disown "$STARTED_PROCESS_GROUP" 2>/dev/null || true
  STARTED_PROCESS_GROUP=""
  trap - EXIT INT TERM
}

show_status() {
  echo "Wrist vision status: $STATUS_TOPIC"
  echo "Close this window or press Ctrl+C to stop status display only."
  exec ros2 topic echo "$STATUS_TOPIC"
}

set_traffic_light_enabled() {
  local value="$1"
  if ! ros2 node list 2>/dev/null | grep -Fxq "$TRAFFIC_LIGHT_NODE"; then
    echo "ERROR: traffic-light recognition node is not running." >&2
    echo "Run $0 first, then retry." >&2
    exit 1
  fi
  ros2 topic pub --once \
    "$TRAFFIC_LIGHT_ENABLE_TOPIC" \
    std_msgs/msg/Bool \
    "{data: $value}" \
    >/dev/null
  echo "Traffic-light recognition $([[ "$value" == "true" ]] && echo enabled || echo disabled)."
}

if [[ "${1:-}" == "--status" ]]; then
  show_status
fi

if [[ "${1:-}" == "--enable-light" ]]; then
  set_traffic_light_enabled true
  exit 0
fi

if [[ "${1:-}" == "--disable-light" ]]; then
  set_traffic_light_enabled false
  exit 0
fi

if [[ -n "${1:-}" ]]; then
  echo "usage: $0 [--enable-light|--disable-light]" >&2
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

mapfile -t EXISTING_LAUNCH_PIDS < <(managed_launch_pids)
if (( ${#EXISTING_LAUNCH_PIDS[@]} > 1 )); then
  echo "Found duplicate wrist vision launches; stopping them before a clean restart."
  for pid in "${EXISTING_LAUNCH_PIDS[@]}"; do
    stop_process_group "$pid"
  done
  EXISTING_LAUNCH_PIDS=()
fi

if image_stream_is_live; then
  echo "Wrist camera is already online; reusing it."
  if ! perception_is_running; then
    PERCEPTION_LOG="$LOG_DIR/wrist_traffic_perception_manual.log"
    echo "Starting traffic recognition; log: $PERCEPTION_LOG"
    start_managed_process ros2 run competition_perception wrist_traffic_node \
      --ros-args \
      --params-file "$COMPETITION_WS/config/perception/wrist_traffic_rules.yaml" \
      >"$PERCEPTION_LOG" 2>&1 </dev/null
    wait_until_ready "traffic recognition" perception_is_running
    keep_started_process
  fi
  if ! traffic_light_is_running; then
    TRAFFIC_LIGHT_LOG="$LOG_DIR/traffic_light_recognition_manual.log"
    echo "Starting on-demand traffic-light recognition; log: $TRAFFIC_LIGHT_LOG"
    start_managed_process ros2 run competition_perception traffic_light_node \
      --ros-args \
      --params-file "$COMPETITION_WS/config/perception/wrist_traffic_rules.yaml" \
      >"$TRAFFIC_LIGHT_LOG" 2>&1 </dev/null
    wait_until_ready "traffic-light recognition" traffic_light_is_running
    keep_started_process
  fi
else
  if (( ${#EXISTING_LAUNCH_PIDS[@]} == 1 )); then
    echo "Existing wrist vision launch has no live image; restarting it."
    stop_process_group "${EXISTING_LAUNCH_PIDS[0]}"
  elif perception_is_running || traffic_light_is_running; then
    echo "ERROR: recognition is running but no live wrist image is available." >&2
    echo "No managed camera launch was found, so unrelated camera processes were left untouched." >&2
    exit 1
  fi
  if [[ ! -e "$WRIST_USB_DEVICE" ]]; then
    echo "ERROR: wrist D435 is not present at USB port 2-3.3.2." >&2
    echo "Check the wrist camera cable or powered USB hub, then retry." >&2
    exit 1
  fi
  CAMERA_LOG="$LOG_DIR/wrist_traffic_test_launch.log"
  echo "Starting wrist camera and traffic recognition; log: $CAMERA_LOG"
  start_managed_process ros2 launch competition_perception wrist_traffic.launch.py \
    >"$CAMERA_LOG" 2>&1 </dev/null
  wait_until_ready "live wrist camera image" image_stream_is_live
  wait_until_ready "traffic recognition" perception_is_running
  wait_until_ready "traffic-light recognition" traffic_light_is_running
  keep_started_process
fi

echo "Wrist camera is ready: $IMAGE_TOPIC"
gnome-terminal \
  --title="Wrist traffic recognition status" \
  -- "$SCRIPT_DIR/start_wrist_vision_test.sh" --status

echo "Opening annotated recognition view: $DISPLAY_TOPIC"
echo "Closing the image window will leave the wrist camera online."
exec /usr/bin/python3 \
  /opt/ros/humble/lib/rqt_image_view/rqt_image_view \
  "$DISPLAY_TOPIC"
