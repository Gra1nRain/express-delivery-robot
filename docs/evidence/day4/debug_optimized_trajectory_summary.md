# Day 4 优化轨迹摘要

## 事实

- route_name: debug_route
- frame_id: map
- ok: True
- trajectory_count: 10
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
| random_obstacle_1 | 63 | 6.200 | 17.829 | 0.500 | 0.771613 |
| pickup_1_rear | 7 | 0.600 | 2.538 | 0.447 | 0.000000 |
| cone_lane_change_1 | 103 | 10.198 | 27.567 | 0.500 | 1.234568 |
| drop_1_rear | 6 | 0.500 | 2.317 | 0.424 | 0.000000 |
| return_to_pickup_area | 109 | 10.799 | 29.039 | 0.500 | 1.234568 |
| pickup_2_rear | 7 | 0.600 | 2.538 | 0.447 | 0.000000 |
| cone_lane_change_2 | 103 | 10.198 | 27.567 | 0.500 | 1.234568 |
| drop_2_rear | 6 | 0.500 | 2.317 | 0.424 | 0.000000 |
| finish_park | 25 | 2.400 | 6.141 | 0.500 | 0.462968 |

## 未验证

- 当前只是离线 artifact；ROS2 发布、tracker 行为和实车运动尚未验证。
