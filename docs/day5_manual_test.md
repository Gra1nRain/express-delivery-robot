# Day5 小车手动启动与实车测试

严格按照本文件从上到下操作。每个“终端”都要新开一个终端窗口；正在运行节点的终端不要关闭，也不要在里面继续输入下一步命令。

如果任一步的检查结果不符合文档，立即停止，不要继续运行发车命令。

## 当前避障拓扑与启动方式

当前唯一支持的避障链是：

```text
/cloud_registered_body
  -> /day5_pointcloud_to_laserscan
  -> /proximity_stop（二维Scan与紫色膨胀代价图）
  -> /local_replanner（局部Hybrid A*）
  -> /mppi_control
  -> /competition_safety
  -> /cmd_vel_safe
```

旧 `competition_avoidance` demo 已停用，不要运行：

```bash
ros2 launch competition_avoidance vehicle_avoidance_bringup.launch.py
ros2 run competition_avoidance avoidance_manager_node
ros2 run competition_avoidance adaptive_local_replanner_node
```

这些旧节点会与当前 `/proximity_stop`、`/local_replanner` 重复发布避障话题。

日常测试推荐使用 `/home/agilex/agilex_ws/restart.sh`。它等价于按顺序执行本文的终端2、终端4、终端5和终端6，并且会在启动主控制栈前自动检查Livox和FAST-LIO点云新鲜度。

两种启动方式只能选择一种：

- **日常推荐**：终端1配置CAN，然后运行 `restart.sh`。
- **故障定位备用**：不运行 `restart.sh`，改为手动执行终端2至终端6。

禁止先运行 `restart.sh`，再重复执行终端2、终端4、终端5或终端6，否则会产生重复节点和重复发布者。

## 0. 测试前准备

1. 小车上电，确认物理急停随时可按。
2. 确认底盘CAN-USB线和雷达线已连接。
3. 如果上一次的节点还在运行，回到对应终端逐个按 `Ctrl+C`。不要复制旧PID执行 `kill`。
4. 此时不要启动 `ranger_twist_adapter_node`。

## 终端1：启动底盘CAN

复制下面完整代码块：

```bash
sudo ip link set can3 down 2>/dev/null || true
sudo ip link set can3 type can bitrate 500000 restart-ms 100
sudo ip link set can3 up

ip -details -statistics link show can3
```

必须看到类似：

```text
state UP
can state ERROR-ACTIVE
```

如果显示 `state DOWN`、`can state STOPPED` 或没有 `can3`，不要继续。

## 日常推荐：一键启动无运动主栈

终端1检查通过后，在小车Ubuntu桌面新开终端，执行：

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/agilex_ws

bash restart.sh
```

该脚本会启动Livox、FAST-LIO、底盘驱动、地图、二维Scan、紫色膨胀代价图、局部Hybrid A*、MPPI、Safety和RViz，但不会启动 `ranger_twist_adapter`，因此不会向 `/cmd_vel` 发布运动命令。

必须看到：

```text
DAY5_SENSORS_READY
No chassis relay was enabled by this script.
```

然后确认RViz已经出现，使用 `2D Pose Estimate` 发布并校准初始位姿。初始位姿可以反复发布，以最后一次为准。

使用一键启动后：

1. **不要再执行下面的终端2、终端4、终端5和终端6。**
2. 直接执行“终端8：启动被动行程诊断”。
3. 再执行“终端3：执行发车前总检查”。
4. 所有检查通过后，最后才允许执行终端7。

下面的逐终端启动流程只用于 `restart.sh` 失败后的故障定位。失败时先保留 `restart.sh` 输出和 `/home/agilex/competition_ws/log/` 日志，不要紧接着盲目运行另一套启动流程。

## 备用：逐终端手动启动

## 终端2：启动Livox雷达

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

## 终端3：检查Livox和点云

先检查Livox：

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

必须看到：

```text
"status": "ready"
```

看到 `ready` 后执行：

```bash
sleep 5
```

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

回到终端3，连续检查两次点云：

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

两次都必须看到：

```text
"status": "ready"
```

正常的 `p95_age_s` 通常约为 `0.03～0.10`。如果接近 `1.4` 或出现 `timeout`，不要继续。

## 终端5：启动主控制栈

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
  trajectory_file:=/home/agilex/competition_ws/docs/evidence/day5/debug_control_validation_trajectory.yaml \
  route_file:=/home/agilex/competition_ws/config/routes/debug_control_validation_route.yaml \
  semantic_map_file:=/home/agilex/competition_ws/maps/debug/semantic_map_control_validation.yaml
```

保持终端5运行。此时底盘适配器仍未启动，小车不应运动。

## 终端6：启动RViz

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000

rviz2 -d /home/agilex/competition_ws/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz
```

在RViz中使用 `2D Pose Estimate` 发布初始位姿。可以反复调整；最后一次发布完成后等待1秒。

## 终端8：启动被动行程诊断

该终端只订阅状态并写入小型时间线，不发布控制消息，也不保存点云、Scan或代价图原始帧。

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

/usr/bin/python3 scripts/day5_trip_diagnostics.py \
  --output /tmp/day5_trip_trace.jsonl
```

保持终端8运行。测试结束后按 `Ctrl+C`，终端会显示保存路径。该文件位于 `/tmp`，重启后可以自动清理；它只用于定位每一次停顿来自规划、控制、安全链还是底盘命令链。

## 终端3：执行发车前总检查

回到终端3，复制下面完整代码块：

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

echo '===== 7. Safety不能有拦截原因 ====='
ros2 topic echo /safety/event --once

echo '===== 8. 发车前 /cmd_vel 发布者必须为0 ====='
ros2 topic info /cmd_vel
```

只有同时满足以下条件才能继续：

- 点云为 `"status": "ready"`
- `/system_state` 中 `control_mode: 1`
- `/system_state` 中 `battery_voltage` 为真实电压，不能是 `0.0`
- `/control/state_valid` 为 `data: true`
- `/avoidance/stop_request` 为 `data: false`
- `/control/status` 没有 `INVALID_STATE` 或 `position_jump`
- `/planning/local_replan_status` 中 `stop_requested` 为 `false`
- `/safety/event` 没有 `remote_not_ready`、`avoidance_stop` 或其他拦截原因
- `/cmd_vel` 的 `Publisher count: 0`

如果出现以下任意情况，禁止发车：

- `remote_not_ready`
- `control_mode: 0`
- `battery_voltage: 0.0`
- `state DOWN` 或 `can state STOPPED`
- `stop_requested: true`
- `avoidance_stop`
- `INVALID_STATE`

## 终端7：确认安全后允许发车

执行本节命令会立即允许小车运动。确认场地清空、人员远离、物理急停可用后，再复制下面完整代码块：

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh

set +e
ros2 run competition_control ranger_twist_adapter_node --ros-args \
  -r __node:=ranger_twist_adapter \
  -p input_topic:=/cmd_vel_safe \
  -p output_topic:=/cmd_vel \
  -p wheelbase_m:=0.494 \
  -p track_width_m:=0.364 \
  -p driver_min_turn_radius_m:=0.47644

adapter_rc=$?
echo "ranger_twist_adapter 已退出，返回码：$adapter_rc"
echo "终端7保持打开；请保留上方错误信息，不要连续重试。"
```

注意：每一行末尾的反斜杠 `\` 都必须保留。发车期间终端7必须一直保持运行；节点异常退出或按 `Ctrl+C` 后，终端窗口仍会保留。

发车后，在终端3检查：

```bash
ros2 node list | grep '^/ranger_twist_adapter$'
ros2 topic info /cmd_vel
```

必须看到：

```text
/ranger_twist_adapter
Publisher count: 1
```

## 停车方法

正常停车：在终端7按 `Ctrl+C`，停止底盘适配器。

异常情况：立即按物理急停，然后在终端7按 `Ctrl+C`。

不要重复启动第二个 `ranger_twist_adapter_node`。

## 关闭全部节点

按以下顺序在对应终端按 `Ctrl+C`：

1. 终端7：底盘适配器
2. 终端6：RViz
3. 终端5：主控制栈
4. 终端4：FAST-LIO
5. 终端2：Livox
