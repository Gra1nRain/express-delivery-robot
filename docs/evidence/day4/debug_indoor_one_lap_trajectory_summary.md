# Day 4 优化轨迹摘要

## 事实

- route_name: debug_indoor_one_lap_route
- frame_id: map
- ok: True
- trajectory_count: 5
- failure_count: 0

## 配置限值

- max_speed_mps: 0.5
- max_acceleration_mps2: 0.3
- max_deceleration_mps2: 0.5
- max_lateral_acceleration_mps2: 0.2
- obstacle_zone_speed_limits_mps: {'RANDOM_OBSTACLE': 0.3, 'CONE_LANE_CHANGE': 0.3}

## 分段摘要

| step_id | points | length_m | duration_s | max_v_mps | max_abs_curvature |
|---|---:|---:|---:|---:|---:|
| pickup_transit_1 | 89 | 8.821 | 23.797 | 0.500 | 0.677021 |
| pickup_1_rear | 7 | 0.601 | 2.539 | 0.448 | 1.008917 |
| drop_transit_1 | 95 | 9.428 | 26.638 | 0.500 | 1.666667 |
| drop_1_rear | 6 | 0.581 | 2.501 | 0.426 | 0.558228 |
| finish_park | 25 | 2.400 | 6.141 | 0.500 | 0.462968 |

## 未验证

- 当前只是离线 artifact；ROS2 发布、tracker 行为和实车运动尚未验证。
