#!/usr/bin/env bash
set -euo pipefail

AGILEX_WS="${AGILEX_WS:-/home/agilex/agilex_ws}"
COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
DRIVER_PROCESS="/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
FLAGS_FILE="$AGILEX_WS/build/livox_ros_driver2/CMakeFiles/livox_ros_driver2_node.dir/flags.make"
QUEUE_PATCH="$COMPETITION_WS/patches/livox_ros_driver2_bounded_packet_queue.patch"
SENSOR_QOS_PATCH="$COMPETITION_WS/patches/livox_ros_driver2_sensor_qos.patch"
DRIVER_SOURCE="$AGILEX_WS/src/livox_ros_driver2"

if [[ ! -d "$AGILEX_WS/src/livox_ros_driver2" ]]; then
  echo "ERROR: Livox source is missing from $AGILEX_WS/src" >&2
  exit 1
fi

if pgrep -f "$DRIVER_PROCESS" >/dev/null; then
  echo "ERROR: stop the Livox driver before rebuilding it." >&2
  exit 1
fi

apply_patch_once() {
  local patch_path="$1"
  local patch_label="$2"

  if [[ ! -f "$patch_path" ]]; then
    echo "ERROR: missing Livox patch: $patch_path" >&2
    exit 1
  fi
  if git -C "$DRIVER_SOURCE" apply --reverse --check "$patch_path" \
       >/dev/null 2>&1; then
    echo "Livox $patch_label patch is already applied."
  elif git -C "$DRIVER_SOURCE" apply --check "$patch_path"; then
    git -C "$DRIVER_SOURCE" apply "$patch_path"
    echo "Applied Livox $patch_label patch."
  else
    echo "ERROR: Livox $patch_label patch does not apply cleanly." >&2
    exit 3
  fi
}

apply_patch_once "$QUEUE_PATCH" "bounded raw-packet queue"
apply_patch_once "$SENSOR_QOS_PATCH" "latest-sample sensor QoS"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$AGILEX_WS"

colcon build \
  --packages-select livox_ros_driver2 \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DROS_EDITION=ROS2

if [[ ! -f "$FLAGS_FILE" ]] ||
   ! grep -q -- "-O3" "$FLAGS_FILE" ||
   ! grep -q -- "-DNDEBUG" "$FLAGS_FILE"; then
  echo "ERROR: Livox was not compiled with the expected Release flags." >&2
  exit 2
fi

echo "LIVOX_RELEASE_BUILD_READY"
grep "CXX_FLAGS =" "$FLAGS_FILE"
