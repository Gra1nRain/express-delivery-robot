# Day 2 语义地图、路线 profile 与相机配置

## 目标与范围

Day 2 使用 Day 1 已选定的 `maps/debug/map.yaml`，只整理实际有效的实验室调试区域，不重新建图。实验室物理摆放可以复用空间，但配置中的任务 step 仍使用官方语义名称。

本次交付范围：

- 第一版 `maps/debug/semantic_map.yaml`。
- 路线表对语义点、dock pose 和规划走廊的引用。
- `debug_site_profile` 对地图、语义地图、路线、ROI 和 dock 配置的统一入口。
- 小车前向相机 `front_camera` 的视觉配置命名确认。
- 车端同步、AMCL/TF 初始标点、FAST-LIO 路线复核和配置引用完整性验收。

## 已确认事实

| 项目 | 当前值 | 证据 |
|---|---|---|
| 2D 地图 | `maps/debug/map.yaml` + `map.pgm` | Day 1 Git 产物 |
| 地图坐标系 | `map` | `maps/debug/map.yaml` |
| 地图分辨率 | `0.03 m/pixel` | `maps/debug/map.yaml` |
| 地图原点 | `[-2.39, -18.30, 0.0]` | `maps/debug/map.yaml` |
| 车上视觉相机 | `front_camera` | `config/perception/camera_devices.yaml` |
| 车上相机选择器 | `usb_port_id=2-3.1.1.1` | 相机设备配置与 Day 1 记录 |
| 腕部相机 | `left_wrist_camera` | 相机设备配置与 Day 1 记录 |

## 语义地图设计

`semantic_map.yaml` 使用 `semantic_map_v1` 草案格式，包含：

- `effective_area`：实验室实际调试区域的多边形边界。
- `points`：起点、红绿灯停止线、取货 dock、投放 dock、终点停车位及障碍区出入口。
- `lane_centerlines`：起点到取货、取货到投放、投放返程和取货到终点四条调试走廊。
- `lane_boundaries`：有效区域外边界。
- `no_go_zones`：把有效区域外声明为派生 keep-out，防止规划越出调试区域；不伪造未测量的实体障碍轮廓。
- `stop_lines`：红绿灯、取货和投放的停止线，随语义点同步更新。
- `obstacle_zones`：随机障碍区和锥桶主动变道区，入口/出口实测，边界由 2.40 m 调试走廊宽度派生。
- `dock_poses`：取货、投放和终点停车的语义 dock 引用。
- `route_corridors`：路线 step 到走廊的配置化映射。

### 坐标状态

事实：坐标写入 `map` frame，单位为米，航向单位为弧度。

事实：本次先用 Day 1 地图预览生成草案点，随后在小车端加载 `map_server`、`amcl`、Livox、FAST-LIO 和 `/scan` 完成早期标点；定位修复后，再使用 FAST-LIO 与 `fastlio_anchor_node` 从 `traffic_light_stop_line` 锚定全局 `map`，由现场操作员遥控低速复核整条路线，Codex 读取 `map -> body` TF 均值并更新语义地图。

事实：以下调试点已按车端 FAST-LIO 路线复核结果写入，并标记为 `measured`。

| 语义点 | x | y | yaw(rad) | 说明 |
|---|---:|---:|---:|---|
| `start` | 0.64 | -1.71 | 0.34 | 起点，由起点锚定采样与红绿灯重锚定关系反算 |
| `traffic_light_stop_line` | 2.98 | -0.77 | 0.43 | 红绿灯停车线 |
| `random_obstacle_entry` | 4.58 | -0.07 | 0.40 | 随机障碍入口 |
| `random_obstacle_exit` | 7.09 | 1.01 | 0.43 | 随机障碍出口 |
| `pickup_dock` | 9.11 | 1.74 | 0.41 | 取货点，两轮采样均值 |
| `cone_lane_change_entry` | 6.86 | 3.68 | -2.71 | 锥桶变道入口，采用第一轮标准入口；第二轮偏上点不写入 |
| `cone_lane_change_exit` | 3.35 | 2.30 | -2.76 | 锥桶变道出口，两轮采样均值 |
| `drop_dock` | 1.55 | 2.14 | -2.77 | 卸货点，两轮采样基本一致 |
| `finish_park` | -0.94 | 0.91 | -2.73 | 终点停车，按现场确认的稳定终点写入 |

经验：随机障碍区和锥桶变道区边界不是逐角实测障碍物轮廓，而是根据已测入口/出口和 2.40 m 调试走廊宽度派生，因此状态为 `derived_draft`。

未验证：本次完成的是人工遥控低速路线复核和语义点重写，不等同于自动导航闭环验收；未统计自动运行的到点误差和返程误差。

## 路线与 profile

`config/routes/debug_route.yaml` 保留 15 个既有 step 及两轮任务顺序：

```text
WAIT_FLAG
→ RUN_SEGMENT(traffic light)
→ RUN_SEGMENT(random obstacle)
→ DOCK/ARM_TASK(PICKUP, cycle 1)
→ CONE_LANE_CHANGE
→ DOCK/ARM_TASK(DROP, cycle 1)
→ RUN_SEGMENT(return)
→ DOCK/ARM_TASK(PICKUP, cycle 2)
→ CONE_LANE_CHANGE
→ DOCK/ARM_TASK(DROP, cycle 2)
→ FINISH_PARK
```

每个需要空间目标的 step 都通过 `target_ref`、`dock_pose_ref`、`entry_ref`、`exit_ref` 或 `corridor_ref` 引用语义地图，不在状态机中写死实验室物理路段。

`config/profiles/debug_site_profile.yaml` 已统一引用：

```text
map.yaml
semantic_map.yaml
debug_route.yaml
planning / optimizer / control 参数
avoidance 接口
camera_devices.yaml
debug_vision_rois.yaml
debug_dock_params.yaml
safety_params.yaml
```

正式场地仍使用独立的 `competition_site_profile.yaml`，本次未修改正式场地配置。

## 相机与视觉 ROI

事实：小车视觉任务统一使用 `front_camera`、命名空间 `/front_camera` 和颜色图像 topic `/front_camera/camera/color/image_raw`。设备按 RealSense 序列号记录、按稳定 USB 路径选择，不使用 `/dev/video*` 编号。

事实：定位修复后，车端按 `usb_port_id=2-3.1.1.1` 启动 `front_camera` color-only 节点，`/front_camera/camera/color/image_raw` 可发布 `424x240 rgb8` 图像。

建议：待起跑旗、红绿灯和停车牌实际进入前向相机画面后，再把 `start_flag`、`traffic_light` 和 `parking_sign` 的 ROI 填为 `normalized_image` 坐标。

未验证：本次抓到的前向相机画面中没有明确的起跑旗、红绿灯或停车牌目标，因此 `config/perception/debug_vision_rois.yaml` 的三个 ROI 仍保持 `null`，状态为 `pending_camera_view`；本次不虚构未采集的像素范围。

## Day 2 验收记录

验收时间：2026-07-20（车端同步与人工低速标点验收）。

| 验收项 | 结果 | 说明 |
|---|---|---|
| 语义地图 YAML 可解析 | 通过 | `semantic_map.yaml` 解析成功，9 个点、4 条中心线、2 个障碍区 |
| 地图路径与 frame | 通过 | 引用 `maps/debug/map.yaml`，frame 为 `map` |
| 车端地图加载 | 通过 | `nav2_bringup localization_launch.py` 成功加载 `maps/debug/map.yaml`，AMCL active |
| 车端扫描输入 | 通过 | `/scan` 发布约 9-11 Hz |
| 初始定位 | 通过 | 起点发布 `/initialpose` 后 `/amcl_pose` 和 `map -> body` TF 正常输出 |
| 语义点引用完整 | 通过 | lane、dock、route corridor 引用均可解析 |
| 路线 step 顺序 | 通过 | 15 个 step，覆盖两轮取放和终点停车 |
| profile 总入口 | 通过 | `debug_site_profile.yaml` 引用 Day 2 四类配置产物 |
| 相机命名与 topic | 通过 | ROI 配置与 `front_camera` 设备配置交叉校验一致；车端 `/front_camera/camera/color/image_raw` 抓帧通过 |
| 低速逐点到位 | 通过 | 语义标点阶段由现场操作员遥控到红绿灯、随机障碍、取货、锥桶、卸货和终点；该阶段 Codex 未发送底盘速度 |
| FAST-LIO 路线复核 | 通过 | 定位修复后从红绿灯停车线锚定 `map -> camera_init`，人工低速复核整条路线并重写 9 个语义点 |
| 障碍区边界 | 通过 | 随机障碍区、锥桶变道区由实测入口/出口派生，所有边界点在 `debug_effective_area` 内 |
| RViz 图形界面 | 部分通过 | `rviz2` 进程可启动，但远程截图为黑屏；本次以 AMCL/TF 采样为准 |
| 往返定位误差 | 未验证 | 未执行自动往返和误差统计 |
| 底盘自动闭环控制 | 未纳入 Day 2 | 只做过一次临时 `/cmd_vel` 低速初测，到红绿灯附近约 0.18-0.23 m 后因航向误差安全停车；用户确认今天不继续实现这块 |

本次配置验收命令及结果：

```text
semantic_map_yaml: valid
points: 9 lanes: 4 obstacle_zones: 2
measured_points: traffic_light_stop_line, pickup_dock, drop_dock, finish_park,
  start, random_obstacle_entry, random_obstacle_exit,
  cone_lane_change_entry, cone_lane_change_exit
derived_zones: random_obstacle_zone_1, cone_lane_change_zone
debug_route_yaml: valid
steps: 15 refs checked
fastlio_route_review: points: 9 measured, refs checked, all points inside effective_area
car_sync: semantic_map.yaml sha256 matched
debug_vision_rois_yaml: valid; image_topic_validation: passed; roi_status: pending_camera_view
```

## 定位稳定性补充验收

验收时间：2026-07-20（车端 FAST-LIO 定位稳定性修复）。

事实：AMCL 在本实验室调试现场的 `map -> body` 结果不稳定；后续导航前置定位改为以 FAST-LIO 的 `camera_init -> body` 为主，并通过可选锚定节点从 `/initialpose` 派生 `map -> camera_init`。锚定节点不能和 AMCL 同时发布同一条全局定位链。

本次定位修复只改动 Day 1 FAST-LIO 调试配置和可选锚定入口：

- `filter_size_surf` / `filter_size_map` 从 `0.15` 恢复为 `0.5`。
- `extrinsic_est_en` 设为 `false`，使用配置中的固定 LiDAR-IMU 外参。
- 新增 `fastlio_anchor_node`，默认不启动；需要全局 map 位姿时，通过 `start_anchor:=true` 启动并发布 `/initialpose`。

验证事实：

| 验收项 | 结果 | 证据 |
|---|---|---|
| FAST-LIO 实时性 | 通过 | 0.15 体素时日志 `ave total` 约 0.108 s；0.5 体素后约 0.041 s，低于 10 Hz LiDAR 帧周期 |
| 静止漂移 | 通过 | 0.5 体素后静止 25 s：x/y/z span 分别约 0.006/0.012/0.014 m，净漂移约 0.002/0.003/-0.004 m |
| 短距离运动后稳定 | 通过 | `cmd_vel` 0.05 m/s 持续 0.8 s，FAST-LIO 平面位移约 0.0418 m；停稳后 15 s x/y span 约 0.011/0.011 m |
| 全局锚定链路 | 通过 | 临时 `/initialpose` 后 `map -> body` 与 `camera_init -> body` 平面误差约 0.0015 m，yaw 误差约 0.0003 rad |
| TF 冲突控制 | 通过 | `ranger_base` 以 `publish_odom_tf:=false` 启动，底盘里程计不抢 FAST-LIO 定位 TF |

未验证：本次没有执行完整 Nav2 自动路线、避障闭环和到点误差统计；正式验收仍需在正式场地重新采图、重标语义点并做自动运行验收。

## 交付与下一步

已交付：

- `maps/debug/semantic_map.yaml`
- `config/routes/debug_route.yaml` 的语义引用
- `config/perception/debug_vision_rois.yaml` 的相机命名与 ROI 状态
- `config/mapping/fast_lio_mid360_day1.yaml` 的定位稳定性配置
- `fastlio_anchor_node` 可选 FAST-LIO 全局锚定节点
- 本文档的 Day 2 事实、验收和遗留风险记录

仍需在小车端完成：

1. 摆放起跑旗、红绿灯和停车牌并采集前向相机画面，把 `debug_vision_rois.yaml` 的 `start_flag`、`traffic_light` 和 `parking_sign` 从 `pending_camera_view` 更新为实际 ROI。
2. 后续导航日再实现正式底盘控制/Nav2 路线闭环，并在安全监督下统计到点误差、停车误差和终点误差。
3. 正式场地开放后重新采集地图和语义点，更新正式 profile、semantic map、mission route 和 vision ROI；不要把实验室临时物理布局写死到正式配置。

安全边界：本次 Codex 没有发送机械臂使能/运动命令。语义标点阶段车辆移动由现场操作员遥控完成；定位修复阶段在用户确认空旷安全后，Codex 仅发送两次短距离低速 `cmd_vel` 验证命令，并在脚本结束时连续发送零速。未删除车上录包或日志。
