#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/day1_common.sh"

STAMP="$(date '+%Y%m%d_%H%M%S')"
OUT_DIR="$COMPETITION_WS/recordings/day1/${STAMP}_chassis_audit"
mkdir -p "$OUT_DIR"

{
  echo "== CAN can3 =="
  ip -br link show can3 2>/dev/null || true
  ip -details link show can3 2>/dev/null || true

  echo
  echo "== Ranger/system topics =="
  for topic in /system_state /motion_state /cmd_vel /cmd_vel_safe /cmd_vel_nav /cmd_vel_dock; do
    echo "## $topic"
    ros2 topic info "$topic" -v 2>&1 || true
    timeout "$DAY1_SYSTEM_ECHO_TIMEOUT_SEC" ros2 topic echo "$topic" --once 2>&1 || true
    echo
  done

  echo "== Ranger base params =="
  ros2 param list /ranger_base_node 2>&1 || true

  echo
  echo "== Twist interface =="
  ros2 interface show geometry_msgs/msg/Twist

  echo
  echo "UNVERIFIED: /cmd_vel.linear.y effect requires a supervised low-speed motion test."
  echo "UNVERIFIED: four-wheel steering or wheel-angle interface must be confirmed with the chassis driver owner."
} >"$OUT_DIR/chassis_audit.txt"

echo "Day 1 chassis audit written to: $OUT_DIR"
