#!/usr/bin/env bash
set -euo pipefail

COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
ENV_SCRIPT="$COMPETITION_WS/scripts/car_source_env.sh"
AVOIDANCE_PARAMS="$COMPETITION_WS/config/avoidance/avoidance_params.yaml"
RVIZ_CONFIG="$COMPETITION_WS/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz"
STARTUP_TIMEOUT_S=45

if [[ ! -f "$ENV_SCRIPT" ]]; then
  echo "ERROR: missing environment script: $ENV_SCRIPT" >&2
  exit 1
fi
if [[ ! -f "$AVOIDANCE_PARAMS" ]]; then
  echo "ERROR: missing avoidance parameters: $AVOIDANCE_PARAMS" >&2
  exit 1
fi
if [[ ! -f "$RVIZ_CONFIG" ]]; then
  echo "ERROR: missing RViz configuration: $RVIZ_CONFIG" >&2
  exit 1
fi

source "$ENV_SCRIPT"
RVIZ_DISPLAY="${RVIZ_DISPLAY:-${DISPLAY:-:1}}"

LOG_DIR="$COMPETITION_WS/log/navigation_prerequisites"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
MAPPING_LOG="$LOG_DIR/${RUN_ID}_mapping.log"
AVOIDANCE_LOG="$LOG_DIR/${RUN_ID}_avoidance.log"
ODOMETRY_ADAPTER_LOG="$LOG_DIR/${RUN_ID}_odometry_adapter.log"
RVIZ_LOG="$LOG_DIR/${RUN_ID}_rviz.log"
MAPPING_PID=""
AVOIDANCE_PID=""
ODOMETRY_ADAPTER_PID=""
ODOM_PUBLISHER_COUNT=""
RVIZ_PID=""
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

require_single_publisher() {
  local topic_name="$1"
  local publisher_count
  publisher_count="$(topic_publisher_count "$topic_name")"
  if (( publisher_count != 1 )); then
    echo "ERROR: $topic_name must have exactly one publisher; found $publisher_count." >&2
    return 1
  fi
  echo "SAFE topic=$topic_name publisher_count=1"
}

refuse_if_process_running() {
  local process_pattern="$1"
  local process_label="$2"
  if pgrep -f -- "$process_pattern" >/dev/null; then
    echo "ERROR: $process_label is already running; refusing duplicate startup." >&2
    echo "Stop the existing prerequisite chain before running this script." >&2
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
  if [[ -z "$MAPPING_PID$AVOIDANCE_PID$ODOMETRY_ADAPTER_PID$RVIZ_PID" ]]; then
    exit "$status"
  fi
  echo "Stopping RViz and navigation prerequisite nodes..."
  stop_process_group "$RVIZ_PID"
  stop_process_group "$AVOIDANCE_PID"
  stop_process_group "$ODOMETRY_ADAPTER_PID"
  stop_process_group "$MAPPING_PID"
  echo "Stopped. Logs remain in $LOG_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

refuse_if_process_running \
  "ros2 launch competition_bringup day1_mapping.launch.py" \
  "day1_mapping"
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
refuse_if_process_running \
  "ros2 run competition_avoidance livox_latest_frame_adapter_node" \
  "Livox latest-frame adapter"
refuse_if_process_running \
  "/competition_avoidance/lib/competition_avoidance/livox_latest_frame_adapter_node" \
  "Livox latest-frame adapter"
refuse_if_process_running \
  "ros2 run competition_avoidance odometry_adapter_node" \
  "odometry adapter"
refuse_if_process_running \
  "/competition_avoidance/lib/competition_avoidance/odometry_adapter_node" \
  "odometry adapter"
refuse_if_process_running \
  "(^|/)rviz2( |$)" \
  "RViz"

require_zero_publishers /cmd_vel

echo "Starting direct 10 Hz Livox, FAST-LIO and pointcloud_to_laserscan..."
setsid ros2 launch competition_bringup day1_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  livox_publish_frequency_hz:=10.0 \
  livox_raw_packet_queue_limit:=256 \
  fast_lio_config:=fast_lio_mid360_day1.yaml \
  start_fast_lio:=true \
  start_base:=false \
  start_scan:=true \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false >"$MAPPING_LOG" 2>&1 &
MAPPING_PID=$!

wait_for_node /livox_lidar_publisher
wait_for_node /laser_mapping
wait_for_topic /cloud_registered_body
wait_for_message /cloud_registered_body
wait_for_topic /Odometry
wait_for_message /Odometry

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

echo "Starting additive avoidance in dry-run mode..."
setsid ros2 run competition_avoidance avoidance_manager_node \
  --ros-args \
  --params-file "$AVOIDANCE_PARAMS" \
  -p dry_run:=true \
  -p enable_chassis_output:=false \
  -p operation_mode:=dry_run \
  -p odometry_topic:=/Odometry \
  -p map_frame:=camera_init >"$AVOIDANCE_LOG" 2>&1 &
AVOIDANCE_PID=$!

wait_for_node /avoidance_manager
wait_for_topic /avoidance/stop_request
wait_for_message /avoidance/stop_request
wait_for_topic /avoidance/local_costmap
require_single_publisher /avoidance/stop_request
require_single_publisher /avoidance/local_costmap

require_zero_publishers /cmd_vel

echo "Starting project RViz configuration on display $RVIZ_DISPLAY..."
setsid env DISPLAY="$RVIZ_DISPLAY" ros2 run rviz2 rviz2 \
  -d "$RVIZ_CONFIG" \
  --ros-args \
  -r __node:=rviz2_day5_motion_control >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!

wait_for_node /rviz2_day5_motion_control
require_zero_publishers /cmd_vel

echo
echo "NAVIGATION_PREREQUISITES_READY"
echo "mapping_pid=$MAPPING_PID odometry_adapter_pid=${ODOMETRY_ADAPTER_PID:-not_started} avoidance_pid=$AVOIDANCE_PID rviz_pid=$RVIZ_PID"
echo "mapping_log=$MAPPING_LOG"
echo "odometry_adapter_log=$ODOMETRY_ADAPTER_LOG"
echo "avoidance_log=$AVOIDANCE_LOG"
echo "rviz_log=$RVIZ_LOG"
echo "Safety gates: start_base=false, start_chassis_adapter=false, command_output_topic=/cmd_vel_safe."
echo "Verified /cmd_vel publisher_count=0; an existing Ranger subscription is allowed."
echo "Keep this terminal open. Press Ctrl+C to stop RViz and all nodes started here."
echo

set +e
WAIT_PIDS=("$MAPPING_PID" "$AVOIDANCE_PID" "$RVIZ_PID")
if [[ -n "$ODOMETRY_ADAPTER_PID" ]]; then
  WAIT_PIDS+=("$ODOMETRY_ADAPTER_PID")
fi
wait -n "${WAIT_PIDS[@]}"
status=$?
set -e
echo "ERROR: a prerequisite process exited unexpectedly." >&2
if (( status == 0 )); then
  status=1
fi
exit "$status"
