# Day 3 路径合法性检查报告

检查时间：2026-07-22

## 检查对象

- 规划 artifact：`docs/evidence/day3/debug_global_plan_car.yaml`
- 语义地图：`maps/debug/semantic_map.yaml`
- 栅格地图：`maps/debug/map.yaml`
- 规划参数：`config/planning/planning_params.yaml`

## 检查口径

事实：

- 地图分辨率 `0.03 m/cell`，尺寸 `977 x 1374`，origin `(-2.39, -18.30, 0)`。
- `map.yaml` 为 `mode: trinary`、`occupied_thresh: 0.65`、`free_thresh: 0.25`。
- 本报告按规划输出点采样检查，不等同于连续车体扫掠碰撞证明。
- 语义走廊检查按对应 `lane_centerline.width_m / 2` 作为半宽，当前 debug lane 半宽为 `1.20 m`。

建议：

- Day4/Day5 做轨迹跟踪前，应继续补充车体 footprint 连续扫掠检查、曲率检查和速度/时间戳合法性检查。

## 结果

总览：

- `ok: true`
- 规划段数：`6`
- 失败数：`0`
- 最大规划耗时：`28.670 ms`
- 规划耗时阈值：`500 ms`

| step | points | length_m | time_ms | planner | smoother | blocked | unknown | outside_map | outside_effective | max_corridor_offset_m | corridor_half_width_m | min_boundary_clearance_m | status |
|---|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| `go_traffic_light_1` | 12 | 2.593 | 27.504 | `occupancy_grid_astar` | `none_insufficient_anchors` | 0 | 0 | 0 | 0 | 0.000 | 1.200 | 1.816 | PASS |
| `random_obstacle_1` | 36 | 6.673 | 28.550 | `occupancy_grid_astar` | `cubic_bezier` | 0 | 0 | 0 | 0 | 0.024 | 1.200 | 3.857 | PASS |
| `cone_lane_change_1` | 53 | 9.049 | 28.670 | `occupancy_grid_astar` | `cubic_bezier` | 0 | 0 | 0 | 0 | 0.509 | 1.200 | 2.844 | PASS |
| `return_to_pickup_area` | 59 | 10.143 | 23.332 | `occupancy_grid_astar` | `cubic_bezier` | 0 | 0 | 0 | 0 | 0.466 | 1.200 | 2.658 | PASS |
| `cone_lane_change_2` | 53 | 9.049 | 18.634 | `occupancy_grid_astar` | `cubic_bezier` | 0 | 0 | 0 | 0 | 0.509 | 1.200 | 2.844 | PASS |
| `finish_park` | 13 | 2.823 | 6.581 | `occupancy_grid_astar` | `none_insufficient_anchors` | 0 | 0 | 0 | 0 | 0.000 | 1.200 | 0.309 | PASS |

## 失败原因负例

事实：

- 使用临时 route 文件把 `go_traffic_light_1.target_ref` 从 `traffic_light_stop_line` 改为 `missing_debug_target`。
- 离线规划返回 `ok=False`、`plans=5`、`failures=1`。
- 失败项：
  - `step_id: go_traffic_light_1`
  - `reason: route_ref_not_on_centerline`
  - `detail: cannot connect start to missing_debug_target on start_to_pickup_lane`

结论：

- 当前离线规划入口能在 route 引用错误时明确输出失败原因。
- 尚未覆盖地图膨胀堵死、目标落障碍、规划超时等全部负例。
