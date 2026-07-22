# Day 4 优化轨迹记录

## 事实

- Day4 仍限定为离线工作：本次没有新增 ROS2 自定义消息、轨迹跟踪器、底盘速度命令或实车运动能力。
- `competition_planning.trajectory_parameterizer` 会把 Day3 的 `StepPlan` 几何路径转换为带 `x`、`y`、`yaw`、`s`、`curvature`、`v`、`yaw_rate`、`t` 字段的轨迹。
- 当前推荐的模块接口是 `optimize_route_trajectory(route, semantic_map, planning_config, optimizer_config)`。离线 CLI 和后续 ROS adapter 应优先调用这个接口，而不是在外部重复编排 `plan_route -> parameterize_route_plan`。
- 停车点从 `semantic_map.yaml` 的 `stop_lines[].point_ref` 和 `dock_poses[].point_ref` 推导。避障区域入口和出口锚点默认按通过点处理，除非它们同时出现在停车语义来源中。
- 当前 debug 限速配置在 `config/planning/optimizer_params.yaml` 中：普通路段 `0.50m/s`，`RANDOM_OBSTACLE` 和 `CONE_LANE_CHANGE` 区域 `0.30m/s`，最大加速度 `0.30m/s^2`，最大减速度 `0.50m/s^2`，最大横向加速度 `0.20m/s^2`。
- `offline_optimized_trajectory` 会生成整条路线的 YAML artifact、每个轨迹 step 一份 CSV、Markdown 摘要和 SVG 俯视图。
- 当前 debug artifact 位于 `docs/evidence/day4/`。

## 验证

- `python -m unittest tests.test_trajectory_parameterizer`：通过，4 个测试。
- `debug_route` 离线优化轨迹生成结果：6 段轨迹，0 个失败。
- 摘要 artifact：`docs/evidence/day4/debug_optimized_trajectory_summary.md`。
- YAML artifact：`docs/evidence/day4/debug_optimized_trajectory.yaml`。
- SVG artifact：`docs/evidence/day4/debug_optimized_trajectory.svg`。
- 分段 CSV：`docs/evidence/day4/csv/*.csv`。

## 当前离线结果

| step | points | length_m | duration_s | max_v_mps |
|---|---:|---:|---:|---:|
| `go_traffic_light_1` | 12 | 2.593 | 5.693 | 0.500 |
| `random_obstacle_1` | 36 | 6.673 | 18.900 | 0.500 |
| `cone_lane_change_1` | 53 | 9.049 | 25.225 | 0.500 |
| `return_to_pickup_area` | 59 | 10.143 | 27.891 | 0.500 |
| `cone_lane_change_2` | 53 | 9.049 | 25.225 | 0.500 |
| `finish_park` | 13 | 2.823 | 7.001 | 0.500 |

## 避障总体计划调整

### 建议

- 避障组可以继续负责算法调研，但主控项目必须负责本车部署适配和验证。
- 避障交付物应包含可运行 baseline 源码、准确 repo/version、模型权重（如有）、license、输入输出格式、依赖和算力假设、启动命令、离线样例数据和已知失败场景。
- 能力建议分阶段推进：第一阶段先支持障碍物或行人检测触发减速/停车，并把输入断流映射到 `SAFE_HOLD`；第二阶段再做随机静态障碍物局部绕行和回到全局路径；锥桶变道当前仍按语义规划走廊处理。

### 未验证

- Day4 本次没有部署或验证任何避障算法。
- 运行时绕障、行人停车以外的主动处理、锥桶感知接入仍属于后续工作。

## 未验证

- 当前生成物只是离线 artifact；ROS2 发布、tracker 消费、安全层兜底和实车运动都尚未验证。
- 曲率限速已经按曲率和横向加速度实现，但 debug Bezier 几何在部分路段仍有较高局部曲率。后续提高速度前必须结合 tracker 和实车验证。
- 路线起点不会被强制设为零速，除非它在语义配置中也是停车点。实际执行时应由 tracker/safety 层结合车辆当前速度处理起步状态。
