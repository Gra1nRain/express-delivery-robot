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
EXECUTOR_PATCH="$COMPETITION_WS/patches/fast_lio_executor_callback_groups.patch"
PREPROCESS_LOCK_PATCH="$COMPETITION_WS/patches/fast_lio_preprocess_lock_scope.patch"
BODY_CLOUD_PATCH="$COMPETITION_WS/patches/fast_lio_independent_body_cloud_publish.patch"
BODY_CLOUD_DOWNSAMPLE_PATCH="$COMPETITION_WS/patches/fast_lio_body_cloud_respects_dense_flag.patch"
RUNTIME_HEALTH_PATCH="$COMPETITION_WS/patches/fast_lio_runtime_health.patch"
DEDICATED_EXECUTORS_PATCH="$COMPETITION_WS/patches/fast_lio_dedicated_sensor_executors.patch"
HUMBLE_CALLBACK_GROUP_PATCH="$COMPETITION_WS/patches/fast_lio_humble_default_callback_group.patch"
IMU_QOS_PATCH="$COMPETITION_WS/patches/fast_lio_best_effort_imu_qos.patch"
STALE_SYNC_PATCH="$COMPETITION_WS/patches/fast_lio_stale_sync_recovery.patch"

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
  local patch_marker="$3"
  local source_file="$FAST_LIO_SOURCE/src/laserMapping.cpp"

  if [[ ! -f "$patch_path" ]]; then
    echo "ERROR: missing FAST-LIO patch: $patch_path" >&2
    exit 1
  fi
  if grep -Fq "$patch_marker" "$source_file"; then
    echo "FAST-LIO $patch_label patch marker is already present."
  elif git -C "$FAST_LIO_SOURCE" apply --reverse --check "$patch_path" \
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

apply_patch_once "$QOS_PATCH" "latest-sample QoS" "lidar_qos.keep_last(1);"
apply_patch_once "$INTERNAL_BUFFER_PATCH" "latest internal buffer" \
  "while (lidar_buffer.size() > 1)"
apply_patch_once "$TIMER_RATE_PATCH" "mapping timer rate" \
  'declare_parameter<int>("mapping.timer_hz", 100);'
apply_patch_once "$EXECUTOR_PATCH" "executor callback groups" \
  "lidar_options.callback_group = lidar_callback_group_;"
apply_patch_once "$PREPROCESS_LOCK_PATCH" "preprocess lock scope" \
  "FAST_LIO_PREPROCESS_OUTSIDE_SYNC_LOCK"
apply_patch_once "$BODY_CLOUD_PATCH" "independent body cloud publishing" \
  "FAST_LIO_INDEPENDENT_BODY_CLOUD_PUBLISH"
apply_patch_once "$BODY_CLOUD_DOWNSAMPLE_PATCH" "body cloud downsampling" \
  "FAST_LIO_BODY_CLOUD_RESPECTS_DENSE_FLAG"
apply_patch_once "$RUNTIME_HEALTH_PATCH" "runtime health" \
  "FAST_LIO_HEALTH lidar_start_hz"
apply_patch_once "$DEDICATED_EXECUTORS_PATCH" "dedicated sensor executors" \
  "FAST_LIO_DEDICATED_SENSOR_EXECUTORS"
apply_patch_once "$HUMBLE_CALLBACK_GROUP_PATCH" "Humble callback group API" \
  "node_base->get_default_callback_group()"
apply_patch_once "$IMU_QOS_PATCH" "best-effort IMU QoS" \
  "FAST_LIO_BEST_EFFORT_IMU_QOS"
apply_patch_once "$STALE_SYNC_PATCH" "stale sync recovery" \
  "stale_sync_drop_count.fetch_add"

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
