# Day5 连续一圈路线与任务点停靠手动测试

本文用于当前连续一圈路线的室内实车测试，基准提交为 `a14bfeb`。

不要运行 `restart.sh`，也不要照抄旧文档中的 `debug_control_validation_trajectory.yaml` 或单独启动 `ranger_twist_adapter_node`。每个“终端”都要新开一个窗口；正在运行节点的终端保持打开。

如果任何检查不符合本文要求，停止操作，不要继续到发车步骤。

## 当前测试内容

当前全局运动几何是一条连续轨迹：

```text
start
  -> traffic_light_stop_line（只经过，不停）
  -> random_obstacle_entry / exit
  -> pickup_front（停）
  -> pickup_rear（停）
  -> cone_lane_change_entry / exit
  -> drop_front（停）
  -> drop_rear（停）
  -> finish_park（终点停车）
```

使用文件：

```text
轨迹：/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory.yaml
路线：/home/agilex/competition_ws/config/routes/debug_indoor_one_lap_route.yaml
语义地图：/home/agilex/competition_ws/maps/debug/semantic_map.yaml
```

### 与旧手动测试文档的区别

旧文档终端5使用：

```text
debug_control_validation_trajectory.yaml
debug_control_validation_route.yaml
semantic_map_control_validation.yaml
```

它们是早期短距离控制/避障验证文件，不是当前一圈路线。

旧文档终端7只启动 `ranger_twist_adapter_node`。当前路线还需要显式发布 `/mission/route_enable=true`，并监控路线完成、控制故障、速度上限和传感器新鲜度，因此改用 `day5_full_route_relay.py`。

### 对避障功能的影响

没有替换或关闭避障链。当前仍为：

```text
/cloud_registered_body
  -> /day5_pointcloud_to_laserscan
  -> /proximity_stop（Scan与膨胀代价图）
  -> /local_replanner（reference-aware Hybrid A*）
  -> /mppi_control
  -> /competition_safety
  -> /cmd_vel_safe
  -> 测试中继
  -> /cmd_vel
```

事实：本次路线修改没有调整雷达频率、障碍物膨胀、Hybrid A*参数、MPPI参数或Safety参数。

事实：局部规划器仍持有同一条完整全局路径，但每次只开放到当前任务停靠点的路径前缀。停稳保持完成后，状态机开放同一路径的下一部分。这是为了防止车辆穿过 `pickup_front` 等任务点。

可能影响：接近任务点时，局部规划目标会变成当前任务点，因此局部路径形状和规划可行性可能与不停靠测试不同。这不会绕过障碍物或降低Safety门限，但停靠精度及停稳后继续规划的行为仍需本轮实车验证。

## 0. 测试前准备

1. 小车上电，清空行驶区域，确认物理急停随时可按。
2. 确认底盘CAN-USB线和雷达线连接正常。
3. 确认小车位于起点附近。
4. 如果已有旧节点，回到相应终端依次按 `Ctrl+C`，不要重复启动同名节点。
5. 此时不要启动任何向 `/cmd_vel` 发布的节点。

## 终端1：配置并检查CAN

```bash
sudo ip link set can3 down 2>/dev/null || true
sudo ip link set can3 type can bitrate 500000 restart-ms 100
sudo ip link set can3 up

ip -details -statistics link show can3
```

必须看到：

```text
state UP
can state ERROR-ACTIVE
```

如果为 `DOWN`、`STOPPED` 或没有 `can3`，不要继续。

## 终端2：启动Livox

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 launch competition_bringup day1_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  start_fast_lio:=false \
  start_base:=false \
  start_scan:=false \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false
```

保持终端2运行。

## 终端3：检查Livox

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

sleep 5

/usr/bin/python3 scripts/day5_sensor_freshness_gate.py \
  --mode livox \
  --timeout-s 20 \
  --max-p95-age-s 0.45 \
  --sample-count 20
```

必须看到 `"status": "ready"`。

## 终端4：启动FAST-LIO

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/agilex/competition_ws/config/mapping \
  config_file:=fast_lio_mid360_day5_control.yaml \
  rviz:=false
```

保持终端4运行。

## 回到终端3：连续检查两次点云

```bash
sleep 5

/usr/bin/python3 scripts/day5_sensor_freshness_gate.py \
  --mode cloud \
  --timeout-s 20 \
  --max-p95-age-s 0.35 \
  --sample-count 20

sleep 3

/usr/bin/python3 scripts/day5_sensor_freshness_gate.py \
  --mode cloud \
  --timeout-s 20 \
  --max-p95-age-s 0.35 \
  --sample-count 20
```

两次都必须看到 `"status": "ready"`。正常 `p95_age_s` 通常约为 `0.03～0.10`；如果接近 `1.4` 或超时，不要继续。

## 终端5：启动连续路线主控制栈

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
  trajectory_file:=/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory.yaml \
  route_file:=/home/agilex/competition_ws/config/routes/debug_indoor_one_lap_route.yaml \
  semantic_map_file:=/home/agilex/competition_ws/maps/debug/semantic_map.yaml
```

保持终端5运行。日志应包含：

```text
continuous_points=226
mission_checkpoints=5
```

此时命令只输出到 `/cmd_vel_safe`，小车不应运动。

## 终端6：启动RViz并发布初始位姿

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000

rviz2 -d /home/agilex/competition_ws/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz
```

在RViz中使用 `2D Pose Estimate` 发布初始位姿。可以反复调整，以最后一次为准。确认雷达点云与静态地图重合后等待至少1秒。

## 终端7：启动被动诊断

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

/usr/bin/python3 scripts/day5_trip_diagnostics.py \
  --output /tmp/day5_continuous_one_lap_trace.jsonl
```

保持终端7运行。该程序只订阅状态并写日志，不发布控制命令。

## 回到终端3：发车前总检查

```bash
echo '===== 1. 点云必须 ready ====='
/usr/bin/python3 scripts/day5_sensor_freshness_gate.py \
  --mode cloud \
  --timeout-s 20 \
  --max-p95-age-s 0.35 \
  --sample-count 20

echo '===== 2. 底盘状态 ====='
ros2 topic echo /system_state --once

echo '===== 3. 定位状态必须 true ====='
ros2 topic echo /control/state_valid --once

echo '===== 4. 避障停车必须 false ====='
ros2 topic echo /avoidance/stop_request --once

echo '===== 5. 控制状态 ====='
ros2 topic echo /control/status --once

echo '===== 6. 局部规划状态 ====='
ros2 topic echo /planning/local_replan_status --once

echo '===== 7. Safety状态 ====='
ros2 topic echo /safety/event --once

echo '===== 8. 发车前 /cmd_vel 发布者必须为0 ====='
ros2 topic info /cmd_vel

echo '===== 9. 发车前车速必须为0 ====='
ros2 topic echo /odom --once --field twist.twist.linear.x
```

只有同时满足以下条件才能发车：

- 点云为 `ready`；
- `/system_state` 中 `control_mode: 1`、`error_code: 0`，电池电压不是 `0.0`；
- `/control/state_valid` 为 `true`；
- `/avoidance/stop_request` 为 `false`；
- `/control/status` 没有 `INVALID_STATE`、`FAULT_HOLD` 或定位跳变原因；
- `/planning/local_replan_status` 的 `stop_requested` 为 `false`；
- `/safety/event` 没有拦截原因；
- `/cmd_vel` 的 `Publisher count: 0`；
- 里程计线速度为 `0.0` 或非常接近0。

## 终端8：启动状态机测试中继（执行后小车会运动）

执行前必须满足：

1. 场地清空，人员远离；
2. 物理急停可随时按下；
3. RViz点云与地图已经对齐；
4. 发车前总检查全部通过；
5. 已取得本轮明确的发车许可。

复制完整代码块：

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
    --label "state_machine_one_lap_$(date +%Y%m%d_%H%M%S)" \
    --launch-pid "${day5_launch_pids[0]}" \
    --skip-initialpose \
    --route-file /home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory.yaml \
    --enable-segmented-route \
    --adapt-ranger-twist \
    --watchdog-timeout-s 420
fi
```

说明：脚本中的 `--route-file` 参数用于读取待执行轨迹的点数、终点和预计时长，因此这里传入连续轨迹YAML。

发车期间保持终端8运行，不要启动第二个中继或 `ranger_twist_adapter_node`。

## 发车后的观察重点

1. `pickup_front`：必须减速到零并保持，不得直接穿过。
   距检查点 0.50 m 内的预期前进速度为 `0.05-0.08 m/s`；航向尚未进入
   `±4°` 时保持 `0.05 m/s`，不应再出现毫米级持续微挪。
2. 保持完成后：当前检查点应推进为 `pickup_rear`，再继续行驶。
3. `pickup_rear`、`drop_front`、`drop_rear`：依次停车。
4. `finish_park`：完成一圈后停车并结束。
5. 遇到局部规划失败或临时状态过期：应发布零速度进入 `SAFETY_HOLD`；数据或路径恢复后才继续。
6. 遇到定位跳变或不可恢复控制故障：应进入 `FAULT_HOLD` 并结束中继。
7. 若未满足精停姿态便穿过检查点平面，应立即显示
   `CHECKPOINT_PLANE_HOLD` 并零速；确认越过超过 `0.02 m` 后锁存为
   `DOCK_OVERSHOOT_HOLD`。这两种状态都不会自动倒车追回；结束终端8并
   人工回到检查点之前，不要重新使能路线。

可在任意新终端查看当前状态：

```bash
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 topic echo /control/status --once
ros2 topic echo /planning/local_replan_status --once
ros2 topic info /cmd_vel
```

## 停车方法

正常人工停车：在终端8按 `Ctrl+C`。中继退出后应停止向 `/cmd_vel` 发布。

异常情况：先按物理急停，再在终端8按 `Ctrl+C`。

停车后检查：

```bash
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

ros2 topic info /cmd_vel
ros2 topic echo /odom --once --field twist.twist.linear.x
```

必须看到 `/cmd_vel` 的 `Publisher count: 0`，里程计线速度为 `0.0` 或非常接近0。

## 关闭节点顺序

在对应终端依次按 `Ctrl+C`：

1. 终端8：状态机测试中继；
2. 终端7：被动诊断；
3. 终端6：RViz；
4. 终端5：主控制栈；
5. 终端4：FAST-LIO；
6. 终端2：Livox。
