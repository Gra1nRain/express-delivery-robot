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
LIVOX_ADAPTER_LOG="$LOG_DIR/${RUN_ID}_livox_latest_frame_adapter.log"
ODOMETRY_ADAPTER_LOG="$LOG_DIR/${RUN_ID}_odometry_adapter.log"
RVIZ_LOG="$LOG_DIR/${RUN_ID}_rviz.log"
MAPPING_PID=""
AVOIDANCE_PID=""
LIVOX_ADAPTER_PID=""
ODOMETRY_ADAPTER_PID=""
RVIZ_PID=""
mkdir -p "$LOG_DIR"

node_exists() {
  local node_name="$1"
  timeout 5s ros2 node list 2>/dev/null | grep -Fxq "$node_name"
}

topic_exists() {
  local topic_name="$1"
  timeout 5s ros2 topic list 2>/dev/null | grep -Fxq "$topic_name"
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
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if topic_exists "$topic_name"; then
      echo "READY topic=$topic_name"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for topic $topic_name" >&2
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
  if [[ -z "$MAPPING_PID$AVOIDANCE_PID$LIVOX_ADAPTER_PID$ODOMETRY_ADAPTER_PID$RVIZ_PID" ]]; then
    exit "$status"
  fi
  echo "Stopping RViz and navigation prerequisite nodes..."
  stop_process_group "$RVIZ_PID"
  stop_process_group "$AVOIDANCE_PID"
  stop_process_group "$ODOMETRY_ADAPTER_PID"
  stop_process_group "$MAPPING_PID"
  stop_process_group "$LIVOX_ADAPTER_PID"
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

echo "Starting additive latest-frame LiDAR adapter..."
setsid ros2 run competition_avoidance livox_latest_frame_adapter_node \
  --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/avoidance/livox_latest \
  -p publish_frequency_hz:=10.0 \
  -p maximum_input_age_s:=0.40 >"$LIVOX_ADAPTER_LOG" 2>&1 &
LIVOX_ADAPTER_PID=$!

wait_for_node /livox_latest_frame_adapter

echo "Starting Livox, FAST-LIO and pointcloud_to_laserscan..."
setsid ros2 launch competition_bringup day1_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  livox_publish_frequency_hz:=20.0 \
  fast_lio_config:=fast_lio_mid360_avoidance_latest.yaml \
  start_fast_lio:=true \
  start_base:=false \
  start_scan:=true \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false >"$MAPPING_LOG" 2>&1 &
MAPPING_PID=$!

wait_for_node /livox_lidar_publisher
wait_for_topic /avoidance/livox_latest
wait_for_node /laser_mapping
wait_for_topic /cloud_registered_body
wait_for_topic /Odometry

echo "Starting FAST-LIO odometry interface adapter..."
setsid ros2 run competition_avoidance odometry_adapter_node \
  --ros-args \
  -p input_topic:=/Odometry \
  -p output_topic:=/odom >"$ODOMETRY_ADAPTER_LOG" 2>&1 &
ODOMETRY_ADAPTER_PID=$!

wait_for_node /odometry_adapter
wait_for_topic /odom

echo "Starting additive avoidance in dry-run mode..."
setsid ros2 run competition_avoidance avoidance_manager_node \
  --ros-args \
  --params-file "$AVOIDANCE_PARAMS" \
  -p odometry_topic:=/Odometry \
  -p map_frame:=camera_init >"$AVOIDANCE_LOG" 2>&1 &
AVOIDANCE_PID=$!

wait_for_node /avoidance_manager
wait_for_topic /avoidance/stop_request

if topic_exists /cmd_vel; then
  echo "ERROR: unexpected /cmd_vel topic detected; stopping prerequisites." >&2
  exit 1
fi

echo "Starting project RViz configuration on display $RVIZ_DISPLAY..."
setsid env DISPLAY="$RVIZ_DISPLAY" ros2 run rviz2 rviz2 \
  -d "$RVIZ_CONFIG" \
  --ros-args \
  -r __node:=rviz2_day5_motion_control >"$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!

wait_for_node /rviz2_day5_motion_control

echo
echo "NAVIGATION_PREREQUISITES_READY"
echo "mapping_pid=$MAPPING_PID livox_adapter_pid=$LIVOX_ADAPTER_PID odometry_adapter_pid=$ODOMETRY_ADAPTER_PID avoidance_pid=$AVOIDANCE_PID rviz_pid=$RVIZ_PID"
echo "mapping_log=$MAPPING_LOG"
echo "livox_adapter_log=$LIVOX_ADAPTER_LOG"
echo "odometry_adapter_log=$ODOMETRY_ADAPTER_LOG"
echo "avoidance_log=$AVOIDANCE_LOG"
echo "rviz_log=$RVIZ_LOG"
echo "Safety gates: start_base=false, no chassis adapter, /cmd_vel absent."
echo "Keep this terminal open. Press Ctrl+C to stop RViz and all nodes started here."
echo

set +e
wait -n "$MAPPING_PID" "$LIVOX_ADAPTER_PID" "$ODOMETRY_ADAPTER_PID" "$AVOIDANCE_PID" "$RVIZ_PID"
status=$?
set -e
echo "ERROR: a prerequisite process exited unexpectedly." >&2
if (( status == 0 )); then
  status=1
fi
exit "$status"
