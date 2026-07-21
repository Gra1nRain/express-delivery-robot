# Day 3 全局路径规划记录

## 事实

- Day3 当前已完成 occupancy-grid A* 全局优化路径发布闭环。
- 当前全局规划模块只输出路径，不输出速度命令，不控制底盘，不触发机械臂动作。
- 可规划 step 为带 `corridor_ref` 的 `RUN_SEGMENT`、`CONE_LANE_CHANGE`、`FINISH_PARK`。
- 离线入口为 `offline_global_plan`，ROS2 发布入口为 `semantic_global_path_node`，默认发布 `go_traffic_light_1` 到 `/planning/global_path`；其他当前 step 通过 `step_id` 参数切换。当前 debug 配置 `global_planner.plugin=occupancy_grid_astar`，保留 `semantic_corridor` fallback。
- `day3_global_planning.launch.py` 当前默认 `show_all_steps:=true`，会额外按 step 发布 6 个分段路径到 `/planning/global_paths/<step_id>`，用于 RViz 同时查看完整可规划路线。
- 小车端当前运行的是 RViz 诊断可视化栈；该栈只发布路径、地图和静态可视化 TF，不启动控制器、底盘驱动或机械臂动作。

## 验证

- 本地 `python -m unittest tests.test_semantic_planner`：9 tests passed。
- 本地 `python -m unittest discover -s tests`：11 tests passed。
- 本地 `python -m compileall src/competition_planning tests/test_semantic_planner.py`：passed。
- 离线 debug route 规划：6 个可规划 step，0 个失败。
- 离线 artifact：`docs/evidence/day3/debug_global_plan.yaml`。
- 小车端 `colcon build --symlink-install --packages-select competition_planning competition_bringup`：passed。
- 小车端 `ros2 run competition_planning offline_global_plan ...`：6 个可规划 step，0 个失败；artifact 为 `docs/evidence/day3/debug_global_plan_car.yaml`。
- 小车端 `/planning/global_path` topic 检查：类型 `nav_msgs/msg/Path`，publisher count 1，`header.frame_id=map`。
- 小车端 `/planning/global_paths/{go_traffic_light_1,random_obstacle_1,cone_lane_change_1,return_to_pickup_area,cone_lane_change_2,finish_park}` topic 检查：每个 topic 类型均为 `nav_msgs/msg/Path`，publisher count 1，RViz subscriber count 1。
- 离线路径可视化证据已保留为当前 optimizer 栅格叠图：`docs/evidence/day3/debug_global_plan_on_map_diagnosis.png`。
- RViz 配置已加入 `competition_bringup`：`day3_global_planning.launch.py rviz:=true` 默认加载 `maps/debug/map.yaml` 到 `/map`，叠加当前 step `/planning/global_path`，并叠加 6 个全路线分段 topic；launch 默认发布 `map -> day3_viz_anchor` 静态 TF，避免纯规划可视化时 RViz 报 `map` frame 不存在。
- 小车端 RViz 可视化已验证：`/map` 栅格地图、`/planning/global_path` 和 6 个 `/planning/global_paths/<step_id>` 可叠加显示，Global Status 和 Map Status OK；旧 RViz 截图已清理，当前保留的路线视觉证据为 `docs/evidence/day3/debug_global_plan_on_map_diagnosis.png`。
- 诊断事实：小车端 `/map` 元数据与 `maps/debug/map.yaml` 一致，`resolution=0.03`、`width=977`、`height=1374`、`origin=(-2.39,-18.3,0)`。
- 历史诊断事实：按 `map.yaml` 投影规划点到 `map.pgm` 时，旧 `traffic_light_stop_line=(2.98,-0.77)` 落在占用栅格；用户将车停到红绿灯停止线后，12 个 `map -> body` 采样均值为 `(2.9882,-0.8088,yaw=0.4303)`，与旧点平面差约 `0.040m`，该阶段曾将停止线更新为 `(2.99,-0.81,yaw=0.43)`。
- 诊断事实：校准前车端 `/Laser_map` frame 为 `camera_init`，实时 TF 为 `camera_init -> body`；当时未发现 `map -> camera_init` 锚定 TF。该事实说明校准前 FAST-LIO 实时定位链和 RViz 静态 occupancy map/path 链路尚未连通。
- 运行校准事实：用户将车停在起点后，车端启动 `competition_localization/fastlio_anchor_node`，并按当时 `semantic_map.yaml` 中 `start=(0.64,-1.71,yaw=0.34)` 发布 `/initialpose`；锚定后 `map -> body` 复核样本相对起点的平面误差约 `0.11m`、yaw 误差约 `0.008rad`，即实时 FAST-LIO 车体 frame 已对齐到起点附近。
- RViz 诊断配置已启用 TF display；锚定后可同时看到 `map`、`camera_init`、`body` 和路径分段，截图为 `docs/evidence/day3/day3_rviz_anchor_tf_window.png`。
- 本次整体对齐事实：用户在 RViz `/clicked_point` 中按顺序标记 `start`、`traffic_light_stop_line`、`random_obstacle_entry`，点击坐标分别为 `(0.010,-0.006)`、`(2.546,-0.016)`、`(4.018,0.002)`；`semantic_map.yaml` 已用 3 锚点 SE(2) 最小二乘整体校准，变换为 `rotation=-22.338099deg`、`translation=(-0.011642567,1.832068285)m`，锚点残差 `rms=0.130m`、`max=0.182m`。
- 本次整体对齐后，小车端 `/planning/global_path` 实际发布端点为 `first=(-0.07,0.01)`、`last=(2.45,-0.05)`，6 个分段 topic 均为 `frame=map` 且 RViz 有 subscriber；本地栅格检查显示 6 条路径采样点均为 FREE，最小占用栅格距离范围为 `0.420m` 到 `0.680m`。叠图证据为 `docs/evidence/day3/debug_global_plan_on_map_diagnosis.png`。
- Optimizer 实现事实：`occupancy_grid_astar` 读取 `map.yaml/map.pgm`，按 `grid_inflation_radius_m` 生成膨胀障碍，逐语义 waypoint 分段 A* 搜索并按 `path_sample_spacing_m` 重采样；实现保留语义锚点。
- 本地 optimizer 验证事实：debug 配置 `grid_inflation_radius_m=0.30m` 时，6 条路径均由 `occupancy_grid_astar` 生成，所有采样点都不在膨胀障碍内；原始栅格最小裕量为 `0.420m`，发生在 `pickup_dock` 附近。
- 当前实车运行事实：本次部署后车端 ROS 图只检测到规划、地图、TF/RViz 相关 topic，未检测到 `/cmd_vel`、`/cmd_vel_safe` 或 `/odom`；因此当前状态只能验证 optimizer Path 发布，不能直接闭环实车行驶。

## 当前车端规划结果

| step | target | plugin | points | length_m | time_ms |
|---|---|---|---:|---:|---:|
| `go_traffic_light_1` | `traffic_light_stop_line` | `occupancy_grid_astar` | 12 | 2.521 | 8.058 |
| `random_obstacle_1` | `random_obstacle_exit` | `occupancy_grid_astar` | 20 | 4.482 | 22.510 |
| `cone_lane_change_1` | `drop_dock` | `occupancy_grid_astar` | 37 | 8.544 | 34.398 |
| `return_to_pickup_area` | `pickup_dock` | `occupancy_grid_astar` | 37 | 8.544 | 28.767 |
| `cone_lane_change_2` | `drop_dock` | `occupancy_grid_astar` | 37 | 8.544 | 42.635 |
| `finish_park` | `finish_park` | `occupancy_grid_astar` | 13 | 2.786 | 6.222 |

## 经验

- 先用 `lane_centerlines` 和 route corridor 做确定性路径，可以快速验证 route、semantic map、allowed_steps、有效区域和禁行区字段是否一致。
- `CONE_LANE_CHANGE` 当前 route 没有 `target_ref`，实现上使用 corridor 末端作为目标，同时校验 `entry_ref` / `exit_ref` 必须落在该段路径上。
- occupancy-grid A* 对当前 debug map 未产生明显绕行，因为校准后的语义中心线本身都在自由栅格内；它的价值主要是把 map 读取、障碍膨胀、碰撞检查、路径发布接口先打通。

## 建议

- RViz 验收前先在车端构建并运行 `day3_global_planning.launch.py`；若在小车图形桌面操作，可加 `rviz:=true` 直接打开 RViz 配置。默认会同时启动 `/map`，若只想看路径可加 `map:=false`。
- 如果路径贴边，先调整语义地图中的 centerline、width 或规划 margin，不先改控制器。
- 不建议通过 RViz offset 或静态平移“修正”路径显示。若路线在 occupancy map 上位置不对，应先重新校准 `semantic_map.yaml` 的语义点，或启动 `fastlio_anchor_node` 后用 `/initialpose` 建立 `map -> camera_init` 锚定，再重新采样/核对语义点。
- 若要提高安全裕量，优先逐点重标 `pickup_dock` 等贴边语义点，再提高 `grid_inflation_radius_m`；不要只靠降低膨胀半径让规划通过。
- `grid_inflation_radius_m` 不宜直接按保守外廓放得过大；debug 取货/卸货点靠近货架或边界，过大的全局膨胀层会把 dock 目标判为不可达。精准取货/卸货应交给后续 DOCK/精停链路处理。
- Hybrid A*/State Lattice 后续可替换 `occupancy_grid_astar` 搜索 backend，不改 route/topic 接口。
- 真正低速试跑前应先启动底盘驱动/安全出口，并由现场确认急停与场地安全。

## 未验证

- 当前 footprint/clearance 已接入规划参数，debug 默认值为 `footprint_radius_m=0.45`、`clearance_m=0.20`；实车外廓仍需复核后冻结。
- 当前 `min_turning_radius_m=0.20` 只是 debug 路线合法性保护阈值，尚未绑定 RANGER 实车最小转弯能力；后续底盘能力确认后必须更新并重新验收曲率。
- `random_obstacle_exit` 到 `pickup_dock` 的末端接近动作当前属于后续 DOCK/精停链路，Day3 不在全局规划里改 route 语义。
- 语义点与 occupancy map 的最终几何一致性仍需用户目视和实车低速复核；本次 3 锚点 SE(2) 已改善整体旋转/平移偏差，但 `pickup_dock` 附近路径最小栅格裕量约 `0.420m`，低于 debug `footprint_radius_m + clearance_m = 0.650m` 的保守目标。
- 当前 `map -> camera_init` 锚定是运行态进程；若重启车端、停止 `fastlio_anchor_node` 或重新启动 FAST-LIO 后，需要在起点或另一个可靠锚点重新发布 `/initialpose`。
