# Day 4 优化轨迹摘要

## 事实

- route_name: debug_route
- frame_id: map
- ok: True
- trajectory_count: 6
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
| go_traffic_light_1 | 12 | 2.593 | 5.693 | 0.500 | 0.000000 |
| random_obstacle_1 | 36 | 6.673 | 18.900 | 0.500 | 0.388479 |
| cone_lane_change_1 | 53 | 9.049 | 25.225 | 0.500 | 5.252711 |
| return_to_pickup_area | 59 | 10.143 | 27.891 | 0.500 | 6.634781 |
| cone_lane_change_2 | 53 | 9.049 | 25.225 | 0.500 | 5.252711 |
| finish_park | 13 | 2.823 | 7.001 | 0.500 | 0.000000 |

## 未验证

- 当前只是离线 artifact；ROS2 发布、tracker 行为和实车运动尚未验证。
