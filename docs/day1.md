# Day 1 接口确认与实验室建图

## 目标

当天结束时要留下可复查证据：ROS2 topic/TF 基线、FAST-LIO 点云地图、可加载的 2D 栅格地图、第一版语义点记录，以及一包基线 rosbag。

## 当前事实

- 小车系统是 Ubuntu 22.04 + ROS2 Humble。
- 小车已有硬件依赖工作区：`/home/agilex/agilex_ws`。
- 新比赛工作区是：`/home/agilex/competition_ws`。
- `competition_bringup` 是新项目 Day 1 的最小 ROS2 包。
- Day 1 launch 只启动传感器/建图链路，不发送运动目标。
- 当前 Livox 驱动为 FAST-LIO 发布 `livox_ros_driver2/msg/CustomMsg`；2D 建图使用 FAST-LIO 输出的 `/cloud_registered_body` 转 `/scan`，坐标链路为 `map -> camera_init -> body`。

## 启动前检查

```bash
cd ~/competition_ws
./scripts/day1_preflight.sh
```

如果要启用底盘里程计，先确认 `can3`。只有在操作员确认急停、遥控接管和场地安全后，才允许把 `start_base` 设为 `true`。

## 建图启动入口

默认启动 Livox、FAST-LIO 和 `/scan` 转换，不启动底盘：

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
source ~/competition_ws/install/setup.bash
ros2 launch competition_bringup day1_mapping.launch.py
```

需要底盘 CAN 审计和 2D 栅格建图同时运行时：

```bash
ros2 launch competition_bringup day1_mapping.launch.py \
  start_base:=true \
  start_slam:=true \
  port_name:=can3
```

## 采集证据

在建图链路运行时，另开终端执行：

```bash
cd ~/competition_ws
./scripts/day1_collect_evidence.sh
./scripts/day1_chassis_audit.sh
./scripts/day1_record_baseline.sh 60
```

输出目录在：

```text
~/competition_ws/recordings/day1/
```

录包和快照不进入 Git，也不自动同步回电脑。

## 保存 2D 栅格地图

`start_slam:=true` 且 RViz 中 `/map` 已经稳定后，执行：

```bash
cd ~/competition_ws
./scripts/day1_save_2d_map.sh
```

这会生成：

```text
~/competition_ws/maps/debug/map.yaml
~/competition_ws/maps/debug/map.pgm
```

## 地图产物命名

Day 1 地图产物统一放：

```text
~/competition_ws/maps/debug/
```

建议命名：

```text
day1_fast_lio.pcd
map.yaml
map.pgm
semantic_map.yaml
```

第一版语义地图可以先从模板复制：

```bash
cp ~/competition_ws/maps/debug/semantic_map.template.yaml \
   ~/competition_ws/maps/debug/semantic_map.yaml
```

然后根据 RViz 标点填入起点、车道中心线、停止线、取货点、投放点和终点区。

## 验收标准

- RViz 能稳定看到点云、2D 地图、机器人位姿和 TF。
- `map -> camera_init -> body` 及传感器 TF 没有冲突发布者。
- 2D 地图能被 map server 或 RViz 正常加载。
- 基线 rosbag 至少覆盖 `/tf`、`/tf_static`、`/odom`、`/cloud_registered`、`/cloud_registered_body`、`/scan`、`/map`。
- 第一版语义点足够支撑 Day 2 配置细化。

## 未验证内容

- D435 topic、机械臂 topic 和 `camera_link` 已纳入 Day 1 快照检查，但需要实车节点运行后确认。
- `/cmd_vel.linear.y` 是否生效、四轮模式或底层转角接口已纳入底盘审计记录，但需要监督下低速运动测试确认。
- FAST-LIO 保存 PCD 的触发方式需在实车运行时确认。
