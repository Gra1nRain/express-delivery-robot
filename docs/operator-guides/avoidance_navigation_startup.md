# RANGER MINI 3 建图、定位、导航与避障启动手册

适用目录：`/home/agilex/competition_ws`

适用场景：从小车开机开始，依次启动 Livox、FAST-LIO、避障、原导航链和 RViz，完成无运动联调与初始位姿发布。

## 0. 安全边界

本手册默认只做无运动联调：

- `start_base:=false`
- `start_chassis_adapter:=false`
- `command_output_topic:=/cmd_vel_safe`
- 不启动 Ranger CAN 驱动
- 不产生 `/cmd_vel`

启动期间保持硬件急停可用。没有完成初始位姿、TF、避障和安全状态检查前，不得自行把上述两个开关改成 `true`。

Livox ROS2 驱动必须使用 Release 优化构建，并应用有界原始包队列补丁。未优化构建或无界队列会导致原始包持续积压，进而让 FAST-LIO 和避障使用过期点云。队列满时只丢弃最旧包、保留最新包；不得通过增大超时参数绕过 `SAFE_HOLD / stale_cloud_timestamp`。

## 1. 小车开机后检查

1. 确认 Livox、工控机和路由器/交换机供电正常。
2. 确认测试区域无人、车轮周围没有杂物，硬件急停和遥控器随时可用。
3. 电脑连接小车所在局域网。
4. 在 Windows PowerShell 中连接小车：

```powershell
ssh agilex@10.148.238.9
```

如果电脑已经配置 SSH 别名，也可以使用：

```powershell
ssh ranger-mini
```

连接后检查主机：

```bash
hostname
pwd
```

预期主机名为 `ubuntu`，工作区为 `/home/agilex/competition_ws`。

首次部署或重编译 Livox 后，先确认驱动使用 Release 优化：

```bash
/home/agilex/competition_ws/scripts/rebuild_livox_release.sh
```

该命令只对 `livox_ros_driver2` 应用仓库中受控的
`livox_ros_driver2_bounded_packet_queue.patch`，随后进行 Release 重编译。
重复执行不会重复应用补丁。必须先停止已有 Livox 驱动。

## 2. 每个车载终端都要加载环境

下面会使用四个终端。每打开一个新的车载终端，都先执行：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u
cd /home/agilex/competition_ws
```

启动前查看已有节点，避免重复启动：

```bash
ros2 node list
```

如果目标节点已经存在，不要再次运行对应启动命令。

## 3. 终端 1：启动 Livox 和 FAST-LIO

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u
cd /home/agilex/competition_ws

ros2 launch competition_bringup day1_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  livox_publish_frequency_hz:=20.0 \
  livox_raw_packet_queue_limit:=256 \
  start_fast_lio:=true \
  start_base:=false \
  start_scan:=true \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false
```

保持该终端运行，不要关闭。

另开一个临时终端检查：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u

ros2 node list
ros2 topic list -t | grep -E '^/livox/|^/cloud_registered|^/Odometry$'
timeout 5s ros2 topic hz /cloud_registered_body --window 20
```

至少应看到：

```text
/livox_lidar_publisher
/laser_mapping
/pointcloud_to_laserscan
```

主要话题：

```text
/livox/lidar
/livox/imu
/cloud_registered
/cloud_registered_body
/Odometry
```

导航前执行两分钟实时性门禁：

```bash
python3 /home/agilex/competition_ws/scripts/livox_latency_acceptance.py
```

只有结果为 `pass` 才能继续。默认要求两路话题均满足：

- `p95 < 0.30 s`
- 最大延迟 `< 0.50 s`
- 120 秒内至少接收 100 个样本

脚本中的 `callback_rate_hz` 是 Python 监测回调速率，不等同于驱动真实发布频率。FAST-LIO 实际处理频率应另外通过运行日志或独立频率检查确认接近原始 `10 Hz`。

## 4. 终端 2：启动新增避障节点

FAST-LIO 实际发布 `/Odometry`，定位坐标系为 `camera_init -> body`，因此本次用运行参数进行接口适配，不修改源码或配置文件：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u
cd /home/agilex/competition_ws

ros2 run competition_avoidance avoidance_manager_node \
  --ros-args \
  --params-file /home/agilex/competition_ws/config/avoidance/avoidance_params.yaml \
  -p odometry_topic:=/Odometry \
  -p map_frame:=camera_init
```

保持该终端运行。

检查避障接口：

```bash
ros2 node list | grep avoidance
ros2 topic list -t | grep '^/avoidance/'
ros2 topic info -v /avoidance/stop_request
ros2 topic echo --once /avoidance/status
ros2 topic echo --once /avoidance/stop_request
```

必须满足：

- `/avoidance/stop_request` 只有一个发布者 `avoidance_manager`
- 点云、里程计或 TF 不健康时，`stop_request` 必须为 `true`

## 5. 终端 3：启动原导航、局部规划、MPPI 和安全节点

该命令复用终端 1 已运行的 Livox/FAST-LIO，不会重复启动传感器和定位。

旧的 `proximity_stop` 必须关闭，由新增避障节点独占 `/avoidance/stop_request` 和 `/avoidance/local_costmap`。

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u
cd /home/agilex/competition_ws

ros2 launch competition_bringup day5_motion_control.launch.py \
  start_livox:=false \
  start_fast_lio:=false \
  start_proximity_stop:=false \
  start_local_replanner:=true \
  start_map_server:=true \
  start_base:=false \
  start_chassis_adapter:=false \
  command_output_topic:=/cmd_vel_safe \
  rviz:=false
```

保持该终端运行。

该启动入口当前默认加载：

```text
地图：/home/agilex/competition_ws/maps/debug/map.yaml
全局轨迹：/home/agilex/competition_ws/docs/evidence/day5/debug_continuous_trajectory.yaml
路线：/home/agilex/competition_ws/config/routes/debug_route.yaml
```

检查节点：

```bash
ros2 node list
```

至少应新增：

```text
/fastlio_anchor
/local_replanner
/mppi_control
/competition_safety
/day5_map_server
/day5_map_lifecycle_manager
```

检查地图服务：

```bash
ros2 lifecycle get /day5_map_server
```

预期：

```text
active [3]
```

## 6. 项目专用 RViz

`start_navigation_prerequisites.sh` 会在前置节点全部就绪后自动使用
`day5_motion_control.rviz` 打开 RViz。不要同时运行系统默认 RViz。

如果 RViz 被单独关闭，或者需要手动恢复，再在小车 Ubuntu 桌面的终端中执行：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
set +u
cd /home/agilex/competition_ws

ros2 run rviz2 rviz2 \
  -d /home/agilex/competition_ws/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz \
  --ros-args \
  -r __node:=rviz2_day5_motion_control
```

这个配置已经设置：

- Fixed Frame：`map`
- 地图：`/map`
- 地图 QoS：`Transient Local`
- 局部代价地图：`/avoidance/local_costmap`
- 局部轨迹：`/planning/local_trajectory`
- 执行轨迹：`/control/executed_path`
- 初始位姿：`/initialpose`

如果必须通过 SSH 在当前小车桌面打开 GUI，先查看桌面显示号：

```bash
who
```

当前车辆通常为 `:1`。确认显示号后可以执行：

```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus

ros2 run rviz2 rviz2 \
  -d /home/agilex/competition_ws/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz \
  --ros-args \
  -r __node:=rviz2_day5_motion_control
```

不要使用默认 RViz 配置。默认 Map 显示通常使用 `Volatile` QoS，如果 RViz 晚于地图服务器启动，会出现 `No map received`。

## 7. 在 RViz 发布初始位姿

1. 确认 RViz 中已经显示静态地图。
2. 点击工具栏中的 `2D Pose Estimate`。
3. 在地图中的车辆实际位置按下鼠标并拖动。
4. 箭头方向必须与车头实际朝向一致。
5. 发布后先不要点击 `2D Goal Pose`。

在检查终端确认定位 TF：

```bash
timeout 10s ros2 run tf2_ros tf2_echo map camera_init
```

能够连续输出平移和旋转数据，说明 `map -> camera_init` 已建立。

继续检查：

```bash
timeout 10s ros2 run tf2_ros tf2_echo map body
ros2 topic echo --once /control/status
ros2 topic echo --once /planning/local_replan_status
ros2 topic echo --once /avoidance/status
ros2 topic echo --once /avoidance/stop_request
```

## 8. 无运动联调的最终安全检查

检查安全出口：

```bash
ros2 topic echo --once /cmd_vel_safe
```

在当前未完成全部健康检查时，预期为全零：

```text
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

检查底盘出口：

```bash
ros2 topic info /cmd_vel
```

无运动联调时预期：

```text
Unknown topic '/cmd_vel'
```

检查规范发布者：

```bash
ros2 topic info -v /avoidance/stop_request
ros2 topic info -v /avoidance/local_costmap
ros2 topic info -v /planning/local_trajectory
ros2 topic info -v /cmd_vel_safe
```

每个话题都必须只有一个发布者。

## 9. 正常关闭顺序

不要直接关机。依次回到对应终端按 `Ctrl+C`：

1. 终端 4：关闭 RViz
2. 终端 3：关闭导航、局部规划、MPPI 和安全节点
3. 终端 2：关闭避障节点
4. 终端 1：最后关闭 Livox 和 FAST-LIO

关闭后检查：

```bash
ros2 node list
```

## 10. 异常处理

如果出现非预期运动：

1. 立即按硬件急停或用遥控器接管。
2. 在导航终端和底盘终端按 `Ctrl+C`。
3. 保留终端输出和 `/home/agilex/.ros/log/` 日志。
4. 不要连续重复启动。

如果 RViz 没有地图：

```bash
ros2 lifecycle get /day5_map_server
ros2 topic info -v /map
```

地图服务器应为 `active`，RViz 的 `/map` 订阅 QoS 应为 `Transient Local`。

如果 RViz 提示 `Fixed Frame [map] does not exist`：

- 检查 `/fastlio_anchor` 是否存在。
- 使用 `2D Pose Estimate` 发布 `/initialpose`。
- 再检查 `map -> camera_init` TF。

如果避障提示 `stale_cloud_timestamp`：

```bash
timeout 8s ros2 topic delay /livox/lidar --window 20
timeout 8s ros2 topic delay /cloud_registered_body --window 20
timeout 8s ros2 topic delay /Odometry --window 20
```

保留检测结果，不要通过增大 `maximum_cloud_age_s` 绕过安全停车。
