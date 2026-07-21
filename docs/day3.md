# Day 3 全局路径规划记录

## 事实

- Day3 范围已收敛为先完成确定性语义走廊全局规划，再评估 Hybrid A*/State Lattice。
- 当前全局规划模块只输出路径，不输出速度命令，不控制底盘，不触发机械臂动作。
- 可规划 step 为带 `corridor_ref` 的 `RUN_SEGMENT`、`CONE_LANE_CHANGE`、`FINISH_PARK`。
- 离线入口为 `offline_global_plan`，ROS2 发布入口为 `semantic_global_path_node`，默认发布 `go_traffic_light_1` 到 `/planning/global_path`；其他当前 step 通过 `step_id` 参数切换。
- 小车端已短时启动 `day3_global_planning.launch.py` 验证 topic；验证结束后已停止测试节点。

## 验证

- 本地 `python -m unittest tests.test_semantic_planner`：6 tests passed。
- 本地 `python -m unittest discover -s tests`：8 tests passed。
- 本地 `python -m compileall src/competition_planning tests/test_semantic_planner.py`：passed。
- 离线 debug route 规划：6 个可规划 step，0 个失败。
- 离线 artifact：`docs/evidence/day3/debug_global_plan.yaml`。
- 小车端 `colcon build --symlink-install --packages-select competition_planning competition_bringup`：passed。
- 小车端 `ros2 run competition_planning offline_global_plan ...`：6 个可规划 step，0 个失败；artifact 为 `docs/evidence/day3/debug_global_plan_car.yaml`。
- 小车端 `/planning/global_path` topic 检查：类型 `nav_msgs/msg/Path`，publisher count 1，`header.frame_id=map`。
- 离线路径可视化证据已生成：`docs/evidence/day3/debug_global_plan_all.png` 和 6 张单 step PNG。

## 当前车端规划结果

| step | target | points | length_m | time_ms |
|---|---|---:|---:|---:|
| `go_traffic_light_1` | `traffic_light_stop_line` | 12 | 2.522 | 0.419 |
| `random_obstacle_1` | `random_obstacle_exit` | 19 | 4.479 | 0.418 |
| `cone_lane_change_1` | `drop_dock` | 37 | 8.550 | 0.705 |
| `return_to_pickup_area` | `pickup_dock` | 37 | 8.550 | 0.677 |
| `cone_lane_change_2` | `drop_dock` | 37 | 8.550 | 0.673 |
| `finish_park` | `finish_park` | 13 | 2.777 | 0.210 |

## 经验

- 先用 `lane_centerlines` 和 route corridor 做确定性路径，可以快速验证 route、semantic map、allowed_steps、有效区域和禁行区字段是否一致。
- `CONE_LANE_CHANGE` 当前 route 没有 `target_ref`，实现上使用 corridor 末端作为目标，同时校验 `entry_ref` / `exit_ref` 必须落在该段路径上。

## 建议

- RViz 验收前先在车端构建并运行 `day3_global_planning.launch.py`，确认 `/planning/global_path` topic 可见。
- 如果路径贴边，先调整语义地图中的 centerline、width 或规划 margin，不先改控制器。

## 未验证

- RViz 可视化截图尚未采集。
- 当前 footprint/clearance 已接入规划参数，debug 默认值为 `footprint_radius_m=0.45`、`clearance_m=0.20`；实车外廓仍需复核后冻结。
- 当前 `min_turning_radius_m=0.20` 只是 debug 路线合法性保护阈值，尚未绑定 RANGER 实车最小转弯能力；后续底盘能力确认后必须更新并重新验收曲率。
- `random_obstacle_exit` 到 `pickup_dock` 的末端接近动作当前属于后续 DOCK/精停链路，Day3 不在全局规划里改 route 语义。
