#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/day1_common.sh"

echo "== System =="
hostname
uname -a
lsb_release -a 2>/dev/null || true

echo "== ROS =="
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
printf 'ROS_VERSION=%s\n' "${ROS_VERSION:-}"
command -v ros2
command -v colcon

echo "== Required packages =="
for pkg in fast_lio livox_ros_driver2 ranger_bringup pointcloud_to_laserscan slam_toolbox nav2_map_server tf2_tools; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    printf '%s: %s\n' "$pkg" "$(ros2 pkg prefix "$pkg")"
  else
    printf '%s: MISSING\n' "$pkg"
  fi
done

echo "== CAN =="
ip -br link show can3 2>/dev/null || true
ip -details link show can3 2>/dev/null || true

echo "== Existing ROS graph =="
ros2 node list 2>/dev/null || true
ros2 topic list -t 2>/dev/null || true

echo "== Day 1 files =="
test -f "$COMPETITION_WS/config/mapping/fast_lio_mid360_day1.yaml"
test -f "$COMPETITION_WS/config/mapping/pointcloud_to_laserscan_day1.yaml"
test -f "$COMPETITION_WS/config/mapping/slam_toolbox_day1.yaml"
test -f "$DAY1_SETTINGS_FILE"
test -f "$DAY1_SNAPSHOT_TOPICS_FILE"
test -f "$DAY1_BAG_TOPICS_FILE"
test -f "$COMPETITION_WS/src/competition_bringup/package.xml"
echo "Snapshot topics:"
read_topic_file "$DAY1_SNAPSHOT_TOPICS_FILE"
echo "Bag topics:"
read_topic_file "$DAY1_BAG_TOPICS_FILE"
echo "Day 1 preflight completed."
