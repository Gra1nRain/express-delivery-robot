#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/car_source_env.sh"

if [[ "$#" -lt 1 ]]; then
  echo "usage: $0 LABEL [day5_motion_control launch arguments...]" >&2
  exit 2
fi

LABEL="$1"
shift
if [[ ! "$LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "LABEL may contain only letters, digits, dot, underscore, and dash" >&2
  exit 2
fi

FAST_LIO_CONFIG="fast_lio_mid360_day5_control.yaml"
for argument in "$@"; do
  case "$argument" in
    start_fast_lio:=*)
      echo "start_fast_lio is managed by this script and must not be overridden" >&2
      exit 2
      ;;
    start_livox:=false|start_livox:=False|start_livox:=0)
      echo "sequential bringup requires start_livox:=true" >&2
      exit 2
      ;;
    fast_lio_config:=*)
      FAST_LIO_CONFIG="${argument#fast_lio_config:=}"
      ;;
  esac
done

LOG_DIR="$COMPETITION_WS/log"
mkdir -p "$LOG_DIR"
BRINGUP_LOG="$LOG_DIR/${LABEL}_bringup.log"
FAST_LIO_LOG="$LOG_DIR/${LABEL}_fast_lio.log"
BRINGUP_PID=""
FAST_LIO_PID=""

stop_process_group() {
  local process_group_pid="$1"
  [[ -n "$process_group_pid" ]] || return 0
  if kill -0 "$process_group_pid" 2>/dev/null; then
    kill -TERM -- "-$process_group_pid" 2>/dev/null || true
    for _ in {1..20}; do
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
  stop_process_group "$FAST_LIO_PID"
  stop_process_group "$BRINGUP_PID"
  exit "$status"
}
trap cleanup EXIT INT TERM

setsid ros2 launch competition_bringup day5_motion_control.launch.py \
  start_fast_lio:=false \
  "$@" >"$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

python3 "$SCRIPT_DIR/day5_sensor_freshness_gate.py" \
  --mode livox \
  --timeout-s 45 \
  --max-p95-age-s 0.45 \
  --sample-count 20

setsid ros2 launch fast_lio mapping.launch.py \
  config_path:="$COMPETITION_WS/config/mapping" \
  config_file:="$FAST_LIO_CONFIG" \
  rviz:=false >"$FAST_LIO_LOG" 2>&1 &
FAST_LIO_PID=$!

python3 "$SCRIPT_DIR/day5_sensor_freshness_gate.py" \
  --mode cloud \
  --timeout-s 45 \
  --max-p95-age-s 0.35 \
  --sample-count 20

echo "DAY5_SENSORS_READY label=$LABEL bringup_pid=$BRINGUP_PID fast_lio_pid=$FAST_LIO_PID"
echo "No chassis relay was enabled by this script."

set +e
wait -n "$BRINGUP_PID" "$FAST_LIO_PID"
status=$?
set -e
exit "$status"
