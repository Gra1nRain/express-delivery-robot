#!/usr/bin/env bash
set -euo pipefail

AGILEX_WS="${AGILEX_WS:-/home/agilex/agilex_ws}"
COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
FAST_LIO_SOURCE="$AGILEX_WS/src/FAST_LIO"
FAST_LIO_PROCESS="/fast_lio/lib/fast_lio/fastlio_mapping"
FLAGS_FILE="$AGILEX_WS/build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make"
QOS_PATCH="$COMPETITION_WS/patches/fast_lio_latest_lidar_qos.patch"

if [[ ! -d "$FAST_LIO_SOURCE" ]]; then
  echo "ERROR: FAST-LIO source is missing from $FAST_LIO_SOURCE" >&2
  exit 1
fi

if pgrep -f "$FAST_LIO_PROCESS" >/dev/null; then
  echo "ERROR: stop FAST-LIO before rebuilding it." >&2
  exit 1
fi

if [[ ! -f "$QOS_PATCH" ]]; then
  echo "ERROR: missing FAST-LIO QoS patch: $QOS_PATCH" >&2
  exit 1
fi

if git -C "$FAST_LIO_SOURCE" apply --reverse --check "$QOS_PATCH" >/dev/null 2>&1; then
  echo "FAST-LIO latest-sample QoS patch is already applied."
elif git -C "$FAST_LIO_SOURCE" apply --check "$QOS_PATCH"; then
  git -C "$FAST_LIO_SOURCE" apply "$QOS_PATCH"
  echo "Applied FAST-LIO latest-sample QoS patch."
else
  echo "ERROR: FAST-LIO QoS patch does not apply cleanly." >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
source "$AGILEX_WS/install/setup.bash"
set -u
cd "$AGILEX_WS"

colcon build \
  --packages-select fast_lio \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release

if [[ ! -f "$FLAGS_FILE" ]] ||
   ! grep -q -- "-O3" "$FLAGS_FILE" ||
   ! grep -q -- "-DNDEBUG" "$FLAGS_FILE"; then
  echo "ERROR: FAST-LIO was not compiled with the expected Release flags." >&2
  exit 2
fi

echo "FAST_LIO_RELEASE_BUILD_READY"
grep "CXX_FLAGS =" "$FLAGS_FILE"
