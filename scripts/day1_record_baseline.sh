#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/day1_common.sh"

DURATION_SEC="${1:-$DAY1_DEFAULT_BAG_DURATION_SEC}"
STAMP="$(date '+%Y%m%d_%H%M%S')"
BAG_DIR="$COMPETITION_WS/recordings/day1/${STAMP}_baseline_bag"

mapfile -t BAG_TOPICS < <(read_topic_file "$DAY1_BAG_TOPICS_FILE")

mkdir -p "$(dirname "$BAG_DIR")"

echo "Recording Day 1 baseline bag for ${DURATION_SEC}s"
echo "Output: $BAG_DIR"
set +e
timeout --signal=INT "${DURATION_SEC}" ros2 bag record -o "$BAG_DIR" "${BAG_TOPICS[@]}"
status=$?
set -e
if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
  exit "$status"
fi
echo "Baseline bag finished: $BAG_DIR"
