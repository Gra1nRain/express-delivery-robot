#!/usr/bin/env bash

export COMPETITION_WS="${COMPETITION_WS:-$HOME/competition_ws}"

_car_source_env_restore_nounset=0
case "$-" in
  *u*) _car_source_env_restore_nounset=1 ;;
esac
set +u
source /opt/ros/humble/setup.bash

if [[ -f "$HOME/agilex_ws/install/setup.bash" ]]; then
  source "$HOME/agilex_ws/install/setup.bash"
fi

if [[ -f "$COMPETITION_WS/install/setup.bash" ]]; then
  source "$COMPETITION_WS/install/setup.bash"
fi
if [[ "$_car_source_env_restore_nounset" -eq 1 ]]; then
  set -u
fi
unset _car_source_env_restore_nounset
