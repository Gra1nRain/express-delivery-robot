# Day 5 global tracking left-bypass continuation summary

- source_artifact: docs/evidence/day5/debug_continuous_trajectory_global_tracking_from_idx87.yaml
- derived_start: last close-pass exec pose (8.3947, 0.1173, yaw=-0.1960)
- source_progress_index: approx from_idx87 target_index=2 / full index 89
- point_count: 359
- first_xy_yaw: (8.3947, 0.1173, -0.0983)
- local_bypass: left offset ramps from current offset to 0.18 m, then fades to 0 by source index 31
- estimated_right_front_obstacle_map_xy: (9.009, -0.331)
- nearest_path_distance_to_estimated_obstacle_m_first_40_points: 0.481 at point 6
- max_abs_stored_curvature_1pm: 1.234568
- chassis_curvature_limit_1pm: 1.234568
- path_length_m_from_start: 35.5669
- duration_s_from_start: 177.8346

未验证：该 artifact 是针对当前右前 proximity 截停点的临时左偏 continuation；仍需低速实车验证，且不代表通用在线局部避障已经完成。
