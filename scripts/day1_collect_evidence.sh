#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/day1_common.sh"

STAMP="$(date '+%Y%m%d_%H%M%S')"
OUT_DIR="$COMPETITION_WS/recordings/day1/${STAMP}_snapshot"
mkdir -p "$OUT_DIR"

{
  date
  hostname
  uname -a
  printf 'COMPETITION_WS=%s\n' "$COMPETITION_WS"
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
} >"$OUT_DIR/system.txt"

env | grep -E '^(ROS_|RMW_|AMENT_|COLCON_|COMPETITION_WS)' | sort >"$OUT_DIR/ros_env.txt" || true
ros2 node list >"$OUT_DIR/nodes.txt" 2>"$OUT_DIR/nodes.err" || true
ros2 topic list -t >"$OUT_DIR/topics.txt" 2>"$OUT_DIR/topics.err" || true

mapfile -t SNAPSHOT_TOPICS < <(read_topic_file "$DAY1_SNAPSHOT_TOPICS_FILE")

for topic in "${SNAPSHOT_TOPICS[@]}"; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  {
    echo "## ros2 topic info $topic"
    ros2 topic info "$topic" -v 2>&1 || true
    echo
    echo "## ros2 topic echo --once $topic"
    timeout "$DAY1_TOPIC_ECHO_TIMEOUT_SEC" ros2 topic echo "$topic" --once 2>&1 || true
  } >"$OUT_DIR/topic_${safe_name}.txt"
done

(
  cd "$OUT_DIR"
  timeout "$DAY1_TF_VIEW_TIMEOUT_SEC" ros2 run tf2_tools view_frames >tf_view_frames.log 2>&1 || true
)

echo "Day 1 snapshot written to: $OUT_DIR"
