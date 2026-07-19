#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/day1_common.sh"

MAP_PREFIX="${1:-$COMPETITION_WS/maps/debug/map}"
mkdir -p "$(dirname "$MAP_PREFIX")"

ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX"
echo "2D map saved with prefix: $MAP_PREFIX"
