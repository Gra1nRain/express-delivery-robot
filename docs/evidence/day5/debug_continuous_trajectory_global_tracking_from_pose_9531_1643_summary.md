# Day 5 pose-rejoin continuation summary

- source_artifact: docs/evidence/day5/debug_continuous_trajectory_global_tracking_arc_rejoin_from_pose_9801_0688.yaml
- derived_start: (9.5308, 1.6429, yaw=1.9929 / 114.19 deg)
- issue_addressed: previous arc-rejoin run stalled because heading_error≈-20.45 deg exceeded the 20 deg safety gate while proximity remained clear
- rejoin_source_index: 13
- first_bridge_segment_length_m: 0.258
- first_bridge_segment_heading_deg: 118.49
- heading_error_to_first_point_deg: 0.002
- heading_error_to_next_source_yaw_deg: -20.447
- max_abs_stored_curvature_1pm: 1.234568
- chassis_curvature_limit_1pm: 1.234568
- point_count: 327
- path_length_m_from_start: 32.7549
- duration_s_from_start: 163.7747

未验证：该 artifact 需要配合 heading30 safety profile 做低速实车验证。
