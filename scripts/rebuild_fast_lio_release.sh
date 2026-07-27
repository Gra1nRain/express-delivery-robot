#!/usr/bin/env bash
set -euo pipefail

AGILEX_WS="${AGILEX_WS:-/home/agilex/agilex_ws}"
COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
FAST_LIO_SOURCE="$AGILEX_WS/src/FAST_LIO"
FAST_LIO_PROCESS="/fast_lio/lib/fast_lio/fastlio_mapping"
FLAGS_FILE="$AGILEX_WS/build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make"
QOS_PATCH="$COMPETITION_WS/patches/fast_lio_latest_lidar_qos.patch"
INTERNAL_BUFFER_PATCH="$COMPETITION_WS/patches/fast_lio_latest_internal_buffer.patch"
TIMER_RATE_PATCH="$COMPETITION_WS/patches/fast_lio_mapping_timer_rate.patch"

if [[ ! -d "$FAST_LIO_SOURCE" ]]; then
  echo "ERROR: FAST-LIO source is missing from $FAST_LIO_SOURCE" >&2
  exit 1
fi

if pgrep -f "$FAST_LIO_PROCESS" >/dev/null; then
  echo "ERROR: stop FAST-LIO before rebuilding it." >&2
  exit 1
fi

apply_patch_once() {
  local patch_path="$1"
  local patch_label="$2"

  if [[ ! -f "$patch_path" ]]; then
    echo "ERROR: missing FAST-LIO patch: $patch_path" >&2
    exit 1
  fi
  if git -C "$FAST_LIO_SOURCE" apply --reverse --check "$patch_path" \
       >/dev/null 2>&1; then
    echo "FAST-LIO $patch_label patch is already applied."
  elif git -C "$FAST_LIO_SOURCE" apply --check "$patch_path"; then
    git -C "$FAST_LIO_SOURCE" apply "$patch_path"
    echo "Applied FAST-LIO $patch_label patch."
  else
    echo "ERROR: FAST-LIO $patch_label patch does not apply cleanly." >&2
    exit 3
  fi
}

apply_patch_once "$QOS_PATCH" "latest-sample QoS"
apply_patch_once "$INTERNAL_BUFFER_PATCH" "latest internal buffer"
apply_patch_once "$TIMER_RATE_PATCH" "mapping timer rate"

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
