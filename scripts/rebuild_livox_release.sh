#!/usr/bin/env bash
set -euo pipefail

AGILEX_WS="${AGILEX_WS:-/home/agilex/agilex_ws}"
DRIVER_PROCESS="/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
FLAGS_FILE="$AGILEX_WS/build/livox_ros_driver2/CMakeFiles/livox_ros_driver2_node.dir/flags.make"

if [[ ! -d "$AGILEX_WS/src/livox_ros_driver2" ]]; then
  echo "ERROR: Livox source is missing from $AGILEX_WS/src" >&2
  exit 1
fi

if pgrep -f "$DRIVER_PROCESS" >/dev/null; then
  echo "ERROR: stop the Livox driver before rebuilding it." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
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
