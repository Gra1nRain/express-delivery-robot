#!/usr/bin/env bash
set -euo pipefail

COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
ENV_SCRIPT="$COMPETITION_WS/scripts/car_source_env.sh"
DAY5_SEQUENCE_SCRIPT="$COMPETITION_WS/scripts/day5_sequential_bringup.sh"
RVIZ_CONFIG="$COMPETITION_WS/install/competition_bringup/share/competition_bringup/rviz/day5_localization.rviz"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-60}"

if [[ ! -f "$ENV_SCRIPT" ]]; then
  echo "ERROR: missing environment script: $ENV_SCRIPT" >&2
  exit 1
fi
if [[ ! -x "$DAY5_SEQUENCE_SCRIPT" ]]; then
  echo "ERROR: missing executable Day5 sequence: $DAY5_SEQUENCE_SCRIPT" >&2
  exit 1
fi
if [[ ! -f "$RVIZ_CONFIG" ]]; then
  echo "ERROR: missing RViz configuration: $RVIZ_CONFIG" >&2
  exit 1
fi

source "$ENV_SCRIPT"

USER_RUNTIME_DIR="/run/user/$(id -u)"
RVIZ_DISPLAY="${RVIZ_DISPLAY:-${DISPLAY:-:1}}"
RVIZ_XAUTHORITY="${RVIZ_XAUTHORITY:-${XAUTHORITY:-$USER_RUNTIME_DIR/gdm/Xauthority}}"
RVIZ_XDG_RUNTIME_DIR="${RVIZ_XDG_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-$USER_RUNTIME_DIR}}"
RVIZ_DBUS_SESSION_BUS_ADDRESS="${RVIZ_DBUS_SESSION_BUS_ADDRESS:-${DBUS_SESSION_BUS_ADDRESS:-unix:path=$USER_RUNTIME_DIR/bus}}"

LOG_DIR="$COMPETITION_WS/log/navigation_prerequisites"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
DAY5_LOG="$LOG_DIR/${RUN_ID}_day5_sequence.log"
ODOMETRY_ADAPTER_LOG="$LOG_DIR/${RUN_ID}_odometry_adapter.log"
RVIZ_LOG="$LOG_DIR/${RUN_ID}_rviz.log"
DAY5_PID=""
ODOMETRY_ADAPTER_PID=""
RVIZ_PID=""
ODOM_PUBLISHER_COUNT=""
REUSED_DAY5=false
mkdir -p "$LOG_DIR"

node_exists() {
  local node_name="$1"
  timeout 5s ros2 node list 2>/dev/null | grep -Fxq "$node_name"
}

topic_publisher_count() {
  local topic_name="$1"
  local topics
  local topic_info
  local publisher_count

  if ! topics="$(timeout 5s ros2 topic list 2>/dev/null)"; then
    echo "ERROR: failed to query the ROS topic graph." >&2
    return 1
  fi
  if ! grep -Fxq "$topic_name" <<<"$topics"; then
    printf '0\n'
    return 0
  fi
  if ! topic_info="$(timeout 5s ros2 topic info "$topic_name" 2>/dev/null)"; then
    echo "ERROR: failed to inspect topic $topic_name." >&2
    return 1
  fi
  publisher_count="$(
    awk -F': *' '/^Publisher count:/ {print $2; exit}' <<<"$topic_info"
  )"
  if [[ ! "$publisher_count" =~ ^[0-9]+$ ]]; then
    echo "ERROR: invalid publisher count for $topic_name." >&2
    return 1
  fi
  printf '%s\n' "$publisher_count"
}

topic_subscription_count() {
  local topic_name="$1"
  local topic_info
  local subscription_count

  if ! topic_info="$(timeout 5s ros2 topic info "$topic_name" 2>/dev/null)"; then
    echo "ERROR: failed to inspect topic $topic_name." >&2
    return 1
  fi
  subscription_count="$(
    awk -F': *' '/^Subscription count:/ {print $2; exit}' <<<"$topic_info"
  )"
  if [[ ! "$subscription_count" =~ ^[0-9]+$ ]]; then
    echo "ERROR: invalid subscription count for $topic_name." >&2
    return 1
  fi
  printf '%s\n' "$subscription_count"
}

publisher_count_is_one() {
  local publisher_count
  publisher_count="$(topic_publisher_count "$1")" || return 1
  (( publisher_count == 1 ))
}

compatible_day5_chain_running() {
  node_exists /livox_lidar_publisher &&
  node_exists /laser_mapping &&
    node_exists /fastlio_anchor &&
    node_exists /day5_map_server &&
    node_exists /local_replanner &&
    node_exists /mppi_control &&
    node_exists /competition_safety &&
    publisher_count_is_one /cloud_registered_body &&
    publisher_count_is_one /Odometry &&
    publisher_count_is_one /map
}

require_zero_publishers() {
  local topic_name="$1"
  local publisher_count
  publisher_count="$(topic_publisher_count "$topic_name")"
  if (( publisher_count != 0 )); then
    echo "ERROR: $topic_name has $publisher_count publisher(s); refusing dry-run startup." >&2
    return 1
  fi
  echo "SAFE topic=$topic_name publisher_count=0. Publisher count: 0 is safe even when Ranger subscribes."
}

refuse_if_process_running() {
  local process_pattern="$1"
  local process_label="$2"
  if pgrep -f -- "$process_pattern" >/dev/null; then
    echo "ERROR: partial or incompatible $process_label process is already running." >&2
    echo "Stop the incomplete chain before starting a fresh Day5 prerequisite chain." >&2
    exit 2
  fi
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
  echo "ERROR: timed out waiting for node $node_name" >&2
  return 1
}

wait_for_topic() {
  local topic_name="$1"
  local publisher_count
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if publisher_count="$(topic_publisher_count "$topic_name")" &&
      (( publisher_count > 0 )); then
      echo "READY topic=$topic_name publisher_count=$publisher_count"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for topic $topic_name" >&2
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
  echo "ERROR: timed out waiting for data on $topic_name" >&2
  return 1
}

wait_for_subscription() {
  local topic_name="$1"
  local subscription_count
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if subscription_count="$(topic_subscription_count "$topic_name")" &&
      (( subscription_count > 0 )); then
      echo "READY rviz_subscription=$topic_name count=$subscription_count"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: RViz did not subscribe to $topic_name" >&2
  return 1
}

stop_process_group() {
  local process_group_pid="$1"
  [[ -n "$process_group_pid" ]] || return 0
  if kill -0 "$process_group_pid" 2>/dev/null; then
    kill -TERM -- "-$process_group_pid" 2>/dev/null || true
    for _ in {1..25}; do
      kill -0 "$process_group_pid" 2>/dev/null || break
      sleep 0.2
    done
  fi
  if kill -0 "$process_group_pid" 2>/dev/null; then
    kill -KILL -- "-$process_group_pid" 2>/dev/null || true
  fi
  wait "$process_group_pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -z "$DAY5_PID$ODOMETRY_ADAPTER_PID$RVIZ_PID" ]]; then
    exit "$status"
  fi
  echo "Stopping only the processes started by this invocation..."
  stop_process_group "$RVIZ_PID"
  stop_process_group "$ODOMETRY_ADAPTER_PID"
  stop_process_group "$DAY5_PID"
  echo "Stopped. Logs remain in $LOG_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

require_zero_publishers /cmd_vel

if compatible_day5_chain_running; then
  REUSED_DAY5=true
  echo "Reusing compatible Day5 prerequisite chain; no sensor process will be duplicated."
else
  refuse_if_process_running \
    "ros2 launch competition_bringup day1_mapping.launch.py" \
    "Day5 sensor"
  refuse_if_process_running \
    "/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node" \
    "Livox driver"
  refuse_if_process_running \
    "/fast_lio/lib/fast_lio/fastlio_mapping" \
    "FAST-LIO"
  refuse_if_process_running \
    "ros2 run competition_avoidance avoidance_manager_node" \
    "avoidance manager"
  refuse_if_process_running \
    "/competition_avoidance/lib/competition_avoidance/avoidance_manager_node" \
    "avoidance manager"

  echo "Starting the 2026-07-24 sensor-first Day5 prerequisite baseline..."
  setsid "$DAY5_SEQUENCE_SCRIPT" "navigation_prerequisites_$RUN_ID" \
    start_base:=false \
    start_chassis_adapter:=false \
    command_output_topic:=/cmd_vel_safe \
    start_proximity_stop:=false \
    start_local_replanner:=true \
    replanning_enabled:=true \
    rviz:=false >"$DAY5_LOG" 2>&1 &
  DAY5_PID=$!

  wait_for_node /livox_lidar_publisher
  wait_for_node /laser_mapping
  wait_for_node /fastlio_anchor
  wait_for_node /day5_map_server
  wait_for_topic /cloud_registered_body
  wait_for_message /cloud_registered_body
  wait_for_topic /Odometry
  wait_for_message /Odometry
  wait_for_topic /map
  wait_for_message /map
fi

ODOM_PUBLISHER_COUNT="$(topic_publisher_count /odom)"
if (( ODOM_PUBLISHER_COUNT == 0 )); then
  echo "Starting FAST-LIO odometry interface adapter..."
  setsid ros2 run competition_avoidance odometry_adapter_node \
    --ros-args \
    -p input_topic:=/Odometry \
    -p output_topic:=/odom >"$ODOMETRY_ADAPTER_LOG" 2>&1 &
  ODOMETRY_ADAPTER_PID=$!
  wait_for_node /odometry_adapter
  wait_for_topic /odom
elif (( ODOM_PUBLISHER_COUNT == 1 )); then
  echo "Using existing /odom publisher; odometry adapter will not be started."
else
  echo "ERROR: more than one /odom publisher already exists; refusing duplicate topology." >&2
  exit 1
fi
wait_for_message /odom

require_zero_publishers /cmd_vel

if node_exists /rviz2_day5_localization; then
  echo "READY node=/rviz2_day5_localization (already running)"
else
  if pgrep -f -- "/rviz2/lib/rviz2" >/dev/null; then
    echo "ERROR: another RViz process is running." >&2
    echo "Close the old RViz window, then run this script again." >&2
    exit 2
  fi

  echo "Starting Day5 map + live body cloud RViz on display $RVIZ_DISPLAY..."
  setsid env \
    DISPLAY="$RVIZ_DISPLAY" \
    XAUTHORITY="$RVIZ_XAUTHORITY" \
    XDG_RUNTIME_DIR="$RVIZ_XDG_RUNTIME_DIR" \
    DBUS_SESSION_BUS_ADDRESS="$RVIZ_DBUS_SESSION_BUS_ADDRESS" \
    ros2 run rviz2 rviz2 \
    -d "$RVIZ_CONFIG" \
    --ros-args \
    -r __node:=rviz2_day5_localization >"$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!

  wait_for_node /rviz2_day5_localization
fi

wait_for_subscription /map
wait_for_subscription /cloud_registered_body
require_zero_publishers /cmd_vel

echo
echo "NAVIGATION_PREREQUISITES_READY"
echo "day5_chain=$([[ "$REUSED_DAY5" == true ]] && echo reused || echo started)"
echo "day5_pid=${DAY5_PID:-not_started} odometry_adapter_pid=${ODOMETRY_ADAPTER_PID:-not_started} rviz_pid=${RVIZ_PID:-already_running}"
echo "day5_log=$DAY5_LOG"
echo "odometry_adapter_log=$ODOMETRY_ADAPTER_LOG"
echo "rviz_log=$RVIZ_LOG"
echo "Safety gates: start_base=false, start_chassis_adapter=false, command_output_topic=/cmd_vel_safe."
echo "Verified /cmd_vel publisher_count=0; an existing Ranger subscription is allowed."
echo "RViz shows /map immediately. Publish 2D Pose Estimate once to create map -> camera_init -> body; live Body Cloud then overlays the map."
echo "Keep this terminal open when this invocation started processes. Press Ctrl+C to stop only those owned processes."
echo

WAIT_PIDS=()
[[ -n "$DAY5_PID" ]] && WAIT_PIDS+=("$DAY5_PID")
[[ -n "$ODOMETRY_ADAPTER_PID" ]] && WAIT_PIDS+=("$ODOMETRY_ADAPTER_PID")
[[ -n "$RVIZ_PID" ]] && WAIT_PIDS+=("$RVIZ_PID")

if (( ${#WAIT_PIDS[@]} == 0 )); then
  exit 0
fi

set +e
wait -n "${WAIT_PIDS[@]}"
status=$?
set -e
echo "ERROR: a process owned by this invocation exited unexpectedly." >&2
if (( status == 0 )); then
  status=1
fi
exit "$status"
