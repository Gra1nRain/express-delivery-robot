#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/car_source_env.sh"

if [[ "$#" -ne 1 ]]; then
  echo "usage: $0 LABEL" >&2
  exit 2
fi
LABEL="$1"
if [[ ! "$LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "LABEL may contain only letters, digits, dot, underscore, and dash" >&2
  exit 2
fi

TOPIC_FILE="$COMPETITION_WS/config/day5/bag_topics.txt"
QOS_FILE="$COMPETITION_WS/config/day5/bag_qos_overrides.yaml"
BAG_DIR="$COMPETITION_WS/recordings/$LABEL"

mapfile -t BAG_TOPICS < <(
  sed -e 's/#.*$//' -e '/^[[:space:]]*$/d' "$TOPIC_FILE"
)
mkdir -p "$(dirname "$BAG_DIR")"

echo "Recording Day5 motion evidence"
echo "Output: $BAG_DIR"
echo "Stop with Ctrl+C after the supervised run."
exec ros2 bag record \
  --qos-profile-overrides-path "$QOS_FILE" \
  -o "$BAG_DIR" \
  "${BAG_TOPICS[@]}"
