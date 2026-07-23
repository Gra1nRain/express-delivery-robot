# Day 5 arc-rejoin continuation summary

- source_artifact: docs/evidence/day5/debug_continuous_trajectory_global_tracking_left_bypass_from_idx89.yaml
- derived_start: last left-bypass exec pose (9.8011, 0.6883, yaw=1.0873 / 62.30 deg)
- issue_addressed: left-bypass cleared the right-front obstacle but stalled at heading_error≈-20.18 deg because safety output became zero
- synthetic_arc_curvature_1pm: 1.200
- synthetic_arc_length_m: 1.15
- rejoin_source_index: 33
- join_segment_length_m: 0.227
- join_segment_heading_deg: 140.89
- max_abs_stored_curvature_1pm: 1.234568
- chassis_curvature_limit_1pm: 1.234568
- initial_heading_error_deg_if_initialized_with_current_yaw: -0.001
- point_count: 339
- path_length_m_from_start: 33.8729
- duration_s_from_start: 169.3644

未验证：该 artifact 是针对当前进弯姿态滞后的临时圆弧接回路线；仍需低速实车验证。
