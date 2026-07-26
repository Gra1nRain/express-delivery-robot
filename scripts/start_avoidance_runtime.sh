#!/usr/bin/env bash
set -euo pipefail

COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
ENV_SCRIPT="$COMPETITION_WS/scripts/car_source_env.sh"
PARAMS_FILE="$COMPETITION_WS/config/avoidance/avoidance_params.yaml"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-30}"
LOG_DIR="$COMPETITION_WS/log/avoidance_runtime"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUNTIME_LOG="$LOG_DIR/${RUN_ID}_avoidance_runtime.log"
RUNTIME_PID=""

if [[ ! -f "$ENV_SCRIPT" ]]; then
  echo "ERROR: missing environment script: $ENV_SCRIPT" >&2
  exit 1
fi
if [[ ! -f "$PARAMS_FILE" ]]; then
  echo "ERROR: missing avoidance parameters: $PARAMS_FILE" >&2
  exit 1
fi

source "$ENV_SCRIPT"
mkdir -p "$LOG_DIR"

node_exists() {
  timeout 5s ros2 node list 2>/dev/null | grep -Fxq "$1"
}

topic_publisher_count() {
  local topic_name="$1"
  local topic_info
  local publisher_count

  if ! timeout 5s ros2 topic list 2>/dev/null | grep -Fxq "$topic_name"; then
    printf '0\n'
    return 0
  fi
  topic_info="$(timeout 5s ros2 topic info "$topic_name" 2>/dev/null)"
  publisher_count="$(
    awk -F': *' '/^Publisher count:/ {print $2; exit}' <<<"$topic_info"
  )"
  if [[ ! "$publisher_count" =~ ^[0-9]+$ ]]; then
    echo "ERROR: invalid publisher count for $topic_name." >&2
    return 1
  fi
  printf '%s\n' "$publisher_count"
}

require_exact_publishers() {
  local topic_name="$1"
  local expected_count="$2"
  local actual_count
  actual_count="$(topic_publisher_count "$topic_name")"
  if (( actual_count != expected_count )); then
    echo "ERROR: $topic_name has $actual_count publisher(s); expected $expected_count." >&2
    return 1
  fi
  echo "READY topic=$topic_name publisher_count=$actual_count"
}

require_zero_publishers() {
  require_exact_publishers "$1" 0
}

wait_for_node() {
  local node_name="$1"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if node_exists "$node_name"; then
      echo "READY node=$node_name"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for node $node_name." >&2
  return 1
}

wait_for_message() {
  local topic_name="$1"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if timeout 5s ros2 topic echo "$topic_name" --once >/dev/null 2>&1; then
      echo "READY message=$topic_name"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for data on $topic_name." >&2
  return 1
}

stop_runtime() {
  [[ -n "$RUNTIME_PID" ]] || return 0
  if kill -0 "$RUNTIME_PID" 2>/dev/null; then
    kill -TERM -- "-$RUNTIME_PID" 2>/dev/null || true
    for _ in {1..25}; do
      kill -0 "$RUNTIME_PID" 2>/dev/null || break
      sleep 0.2
    done
  fi
  if kill -0 "$RUNTIME_PID" 2>/dev/null; then
    kill -KILL -- "-$RUNTIME_PID" 2>/dev/null || true
  fi
  wait "$RUNTIME_PID" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_runtime
  if [[ -n "$RUNTIME_PID" ]]; then
    echo "Stopped additive avoidance runtime. Log remains in $RUNTIME_LOG"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

for required_node in \
  /livox_lidar_publisher \
  /laser_mapping \
  /fastlio_anchor \
  /day5_map_server \
  /local_replanner \
  /mppi_control \
  /competition_safety; do
  if ! node_exists "$required_node"; then
    echo "ERROR: prerequisite node is missing: $required_node" >&2
    echo "Run start_navigation_prerequisites.sh and publish the initial pose first." >&2
    exit 2
  fi
done

require_exact_publishers /cloud_registered_body 1
require_exact_publishers /odom 1
require_exact_publishers /planning/local_trajectory 1
require_exact_publishers /cmd_vel_safe 1
require_zero_publishers /cmd_vel

if node_exists /avoidance_manager; then
  require_exact_publishers /avoidance/stop_request 1
  require_exact_publishers /avoidance/local_costmap 1
  require_exact_publishers /avoidance/scan 1
  echo "AVOIDANCE_RUNTIME_READY already_running=true"
  exit 0
fi

for topic_name in \
  /avoidance/stop_request \
  /avoidance/local_costmap \
  /avoidance/scan \
  /avoidance/status \
  /avoidance/objects \
  /avoidance/corridor_update; do
  require_zero_publishers "$topic_name"
done

TF_OUTPUT="$(timeout 5s ros2 run tf2_ros tf2_echo map body 2>/dev/null || true)"
if ! grep -q "Translation:" <<<"$TF_OUTPUT"; then
  echo "ERROR: map -> body is unavailable; publish the initial pose before avoidance." >&2
  exit 2
fi

setsid ros2 launch competition_avoidance vehicle_avoidance_bringup.launch.py \
  dry_run:=true \
  enable_chassis_output:=false \
  operation_mode:=dry_run \
  avoidance_params_file:="$PARAMS_FILE" >"$RUNTIME_LOG" 2>&1 &
RUNTIME_PID=$!

wait_for_node /avoidance_manager
wait_for_message /avoidance/status
wait_for_message /avoidance/local_costmap
wait_for_message /avoidance/scan

require_exact_publishers /avoidance/stop_request 1
require_exact_publishers /avoidance/local_costmap 1
require_exact_publishers /avoidance/scan 1
require_exact_publishers /planning/local_trajectory 1
require_exact_publishers /cmd_vel_safe 1
require_zero_publishers /cmd_vel

echo
echo "AVOIDANCE_RUNTIME_READY already_running=false"
echo "runtime_pid=$RUNTIME_PID"
echo "runtime_log=$RUNTIME_LOG"
echo "Safety gates remain closed: dry_run=true and /cmd_vel publisher_count=0."
echo "Keep this terminal open. Press Ctrl+C to stop only the additive avoidance runtime."
echo

set +e
wait "$RUNTIME_PID"
status=$?
set -e
if (( status == 0 )); then
  status=1
fi
exit "$status"
