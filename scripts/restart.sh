#!/usr/bin/env bash
set -Eeuo pipefail

export COMPETITION_WS="${COMPETITION_WS:-/home/agilex/competition_ws}"
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME
unset PYTHONPATH
hash -r

if [[ "$(id -un)" != "agilex" ]]; then
  echo "restart.sh must be run as the agilex user" >&2
  exit 1
fi
if [[ ! -x "$COMPETITION_WS/scripts/day5_sequential_bringup.sh" ]]; then
  echo "missing $COMPETITION_WS/scripts/day5_sequential_bringup.sh" >&2
  exit 1
fi

source "$COMPETITION_WS/scripts/car_source_env.sh"

if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "ROS must use /usr/bin/python3, got $(command -v python3)" >&2
  exit 1
fi

stop_existing_stack() {
  local current_pgid
  current_pgid="$(ps -o pgid= -p "$$" | tr -d " ")"
  declare -A process_groups=()

  while read -r pid pgid arguments; do
    pgid="${pgid// /}"
    [[ -n "$pgid" && "$pgid" != "$current_pgid" ]] || continue
    case "$arguments" in
      *"scripts/day5_sequential_bringup.sh"* | \
      *"ros2 launch competition_bringup day5_motion_control.launch.py"* | \
      *"ros2 launch competition_bringup day1_mapping.launch.py"* | \
      *"ros2 launch fast_lio mapping.launch.py"* | \
      *"rviz2 -d "*"day5_motion_control.rviz"*)
        process_groups["$pgid"]=1
        ;;
    esac
  done < <(ps -eo pid=,pgid=,args=)

  if [[ "${#process_groups[@]}" -eq 0 ]]; then
    echo "No previous Day5 process groups found."
    return
  fi

  for pgid in "${!process_groups[@]}"; do
    echo "Stopping previous Day5 process group $pgid"
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done

  for _ in {1..25}; do
    local any_alive=false
    for pgid in "${!process_groups[@]}"; do
      if kill -0 -- "-$pgid" 2>/dev/null; then
        any_alive=true
        break
      fi
    done
    [[ "$any_alive" == "false" ]] && return
    sleep 0.2
  done

  echo "A previous Day5 process group did not stop cleanly; inspect it before retrying." >&2
  return 1
}

stop_existing_stack

label="restart_hybrid_astar_$(date +%Y%m%d_%H%M%S)"
echo "Starting Day5 sensors, localization, Hybrid A*, control, safety, and RViz."
echo "The final chassis adapter remains disabled; /cmd_vel will have no publisher."

exec bash "$COMPETITION_WS/scripts/day5_sequential_bringup.sh" "$label" \
  rviz:=true \
  start_base:=true \
  start_map_server:=true \
  start_proximity_stop:=true \
  start_local_replanner:=true \
  replanning_enabled:=true \
  command_output_topic:=/cmd_vel_safe \
  start_chassis_adapter:=false \
  trajectory_file:="$COMPETITION_WS/docs/evidence/day5/debug_control_validation_trajectory.yaml" \
  route_file:="$COMPETITION_WS/config/routes/debug_control_validation_route.yaml" \
  semantic_map_file:="$COMPETITION_WS/maps/debug/semantic_map_control_validation.yaml"
