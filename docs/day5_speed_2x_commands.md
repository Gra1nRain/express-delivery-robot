# Day 5 提速档必要命令

## 终端5：主控制栈

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 launch competition_bringup day5_motion_control.launch.py \
  start_livox:=false \
  start_fast_lio:=false \
  rviz:=false \
  start_base:=true \
  start_map_server:=true \
  start_proximity_stop:=true \
  start_local_replanner:=true \
  replanning_enabled:=true \
  command_output_topic:=/cmd_vel_safe \
  start_chassis_adapter:=false \
  trajectory_file:=/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml \
  route_file:=/home/agilex/competition_ws/config/routes/debug_indoor_one_lap_route.yaml \
  semantic_map_file:=/home/agilex/competition_ws/maps/debug/semantic_map.yaml \
  planning_params_file:=/home/agilex/competition_ws/config/planning/planning_params.yaml \
  optimizer_params_file:=/home/agilex/competition_ws/config/planning/optimizer_params_day5_speed_2x.yaml \
  control_params_file:=/home/agilex/competition_ws/config/control/control_params_day5_speed_2x.yaml \
  safety_params_file:=/home/agilex/competition_ws/config/safety/safety_params.yaml \
  dock_params_file:=/home/agilex/competition_ws/config/docking/debug_dock_params.yaml
```

## 发布初始位姿

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 topic pub --once \
  --wait-matching-subscriptions 3 \
  --keep-alive 2.0 \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -0.416, y: 0.464, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0003046]}}"
```

## 发车前最小检查

```bash
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

python3 -c 'import yaml; print(yaml.safe_load(open("/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml"))["duration_s"])'
ros2 param get /mppi_control max_speed_mps
ros2 param get /competition_safety max_speed_mps
ros2 topic echo /control/state_valid --once
ros2 topic info /cmd_vel
timeout 3 ros2 run tf2_ros tf2_echo map body
```

应看到轨迹时长 `122.62`、两个速度上限均为 `0.20`、`state_valid=true`、`/cmd_vel` 发布者为0。

## 终端8：发车中继

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

mapfile -t day5_launch_pids < <(
  pgrep -f '/opt/ros/humble/bin/ros2 launch competition_bringup day5_motion_control.launch.py'
)

if [ "${#day5_launch_pids[@]}" -ne 1 ]; then
  echo "拒绝发车：主控制launch数量=${#day5_launch_pids[@]}，应为1"
  printf 'PID: %s\n' "${day5_launch_pids[@]}"
else
  /usr/bin/python3 scripts/day5_full_route_relay.py \
    --label "state_machine_speed_2x_$(date +%Y%m%d_%H%M%S)" \
    --launch-pid "${day5_launch_pids[0]}" \
    --skip-initialpose \
    --route-file /home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml \
    --enable-segmented-route \
    --adapt-ranger-twist \
    --watchdog-timeout-s 0 \
    --no-progress-timeout-s 120 \
    --max-command-mps 0.23 \
    --max-odom-mps 0.28
fi
```
