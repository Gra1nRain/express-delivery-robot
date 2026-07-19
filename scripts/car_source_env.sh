#!/usr/bin/env bash
set -eo pipefail

export COMPETITION_WS="${COMPETITION_WS:-$HOME/competition_ws}"

set +u
source /opt/ros/humble/setup.bash

if [[ -f "$HOME/agilex_ws/install/setup.bash" ]]; then
  source "$HOME/agilex_ws/install/setup.bash"
fi

if [[ -f "$COMPETITION_WS/install/setup.bash" ]]; then
  source "$COMPETITION_WS/install/setup.bash"
fi
set -u
