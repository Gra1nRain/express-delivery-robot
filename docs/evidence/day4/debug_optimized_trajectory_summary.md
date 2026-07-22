# Day 4 Optimized Trajectory Summary

## Facts

- route_name: debug_route
- frame_id: map
- ok: True
- trajectory_count: 6
- failure_count: 0

## Configured Limits

- max_speed_mps: 0.5
- max_acceleration_mps2: 0.3
- max_deceleration_mps2: 0.5
- max_lateral_acceleration_mps2: 0.2
- obstacle_zone_speed_limits_mps: {'RANDOM_OBSTACLE': 0.3, 'CONE_LANE_CHANGE': 0.3}

## Per-step Summary

| step_id | points | length_m | duration_s | max_v_mps | max_abs_curvature |
|---|---:|---:|---:|---:|---:|
| go_traffic_light_1 | 12 | 2.593 | 5.693 | 0.500 | 0.000000 |
| random_obstacle_1 | 36 | 6.673 | 18.900 | 0.500 | 0.388479 |
| cone_lane_change_1 | 53 | 9.049 | 25.225 | 0.500 | 5.252711 |
| return_to_pickup_area | 59 | 10.143 | 27.891 | 0.500 | 6.634781 |
| cone_lane_change_2 | 53 | 9.049 | 25.225 | 0.500 | 5.252711 |
| finish_park | 13 | 2.823 | 7.001 | 0.500 | 0.000000 |

## Unverified

- This is an offline artifact only; ROS2 publication, tracker behavior, and real-car motion are not validated here.
