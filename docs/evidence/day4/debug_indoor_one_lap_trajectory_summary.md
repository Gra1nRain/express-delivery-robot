# Day 4 优化轨迹摘要

## 事实

- route_name: debug_indoor_one_lap_route
- frame_id: map
- ok: True
- trajectory_count: 8
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
| go_traffic_light_1 | 27 | 2.600 | 5.706 | 0.500 | 0.000000 |
| random_obstacle_1 | 45 | 4.400 | 13.587 | 0.500 | 0.308642 |
| pickup_approach_1 | 18 | 1.700 | 4.104 | 0.500 | 1.041685 |
| pickup_1_rear | 7 | 0.600 | 2.538 | 0.447 | 0.000000 |
| cone_lane_change_1 | 84 | 8.299 | 22.953 | 0.500 | 1.234568 |
| drop_approach_1 | 19 | 1.799 | 4.519 | 0.500 | 1.458360 |
| drop_1_rear | 6 | 0.500 | 2.317 | 0.424 | 0.000000 |
| finish_park | 25 | 2.400 | 6.141 | 0.500 | 0.462968 |

## 未验证

- 当前只是离线 artifact；ROS2 发布、tracker 行为和实车运动尚未验证。
