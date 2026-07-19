#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/car_source_env.sh"

DAY1_CONFIG_DIR="${DAY1_CONFIG_DIR:-$COMPETITION_WS/config/day1}"
DAY1_SETTINGS_FILE="${DAY1_SETTINGS_FILE:-$DAY1_CONFIG_DIR/evidence_settings.env}"
DAY1_SNAPSHOT_TOPICS_FILE="${DAY1_SNAPSHOT_TOPICS_FILE:-$DAY1_CONFIG_DIR/snapshot_topics.txt}"
DAY1_BAG_TOPICS_FILE="${DAY1_BAG_TOPICS_FILE:-$DAY1_CONFIG_DIR/bag_topics.txt}"

if [[ ! -f "$DAY1_SETTINGS_FILE" ]]; then
  echo "Missing Day 1 settings file: $DAY1_SETTINGS_FILE" >&2
  exit 2
fi

source "$DAY1_SETTINGS_FILE"

read_topic_file() {
  local path=$1
  if [[ ! -f "$path" ]]; then
    echo "Missing Day 1 topic file: $path" >&2
    exit 2
  fi
  tr -d '\r' <"$path" | sed -E 's/[[:space:]]*#.*$//' | sed -E '/^[[:space:]]*$/d'
}
