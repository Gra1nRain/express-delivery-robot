# Day 5 drop_dock collision analysis

Date: 2026-07-22

## Artifacts

- Code under test: `a7b40b6 Add bounded pose delay compensation`
- Bag on car: `/home/agilex/competition_ws/recordings/day5_drop_dock_a7b40b6_20260722_195338`
- Monitor log on car: `/home/agilex/competition_ws/log/day5_drop2_motion_monitor_a7b40b6.txt`

## Facts from the run

- The Day5 motion launch published chassis commands through `competition_safety` to `/cmd_vel`.
- The run entered `TRACKING` 1.77 s after `/initialpose`.
- Before the first `remote_not_ready` event after tracking:
  - max `/cmd_vel.linear.x`: `0.198731 m/s`
  - max `/odom.twist.twist.linear.x`: `0.201000 m/s`
  - max absolute lateral tracking error: `0.043749 m`
- The first `remote_not_ready` event after tracking occurred about `50.85 s` after `TRACKING`; `/cmd_vel` was zero about `0.05 s` later.
- After that event, `/odom` reported motion up to `1.0 m/s`, while `/cmd_vel` stayed zero. This period includes operator takeover, collision interaction, or chassis behavior outside the autonomous `/cmd_vel` command stream.
- The first `lateral_error_exceeded` event occurred about `82.36 s` after `TRACKING`; `RECOVERY_REQUIRED` began about `83.19 s` after `TRACKING`.

## Facts from code/config

- `day5_motion_control.launch.py` included Day1 mapping with `start_scan=false`.
- `fast_lio_mid360_day5_control.yaml` did not expose `/cloud_registered_body` for a local safety check; later no-motion validation showed the master `scan_publish_en` switch must also be enabled for FAST-LIO point cloud outputs.
- `SafetyNode` subscribed to `/avoidance/stop_request`, but Day5 launch did not start any publisher for that topic.
- `SafetySupervisor` treated a missing avoidance source as `avoidance_stop=False`; there was no freshness requirement for avoidance input.
- The offline planner did use static occupancy-grid inflation when generating the fixed trajectory, but that inflation only affected trajectory generation. It did not run as an online local costmap or obstacle layer during the field run.

## Conclusion

The collision was not caused by a Nav2-style online inflation layer failing to trigger. There was no online costmap/inflation/local obstacle layer in the Day5 field-control graph.

The MPPI controller was not replaying a precomputed list of `/cmd_vel` commands. It recomputed a tracking command at 20 Hz from the current `map -> body` pose and `/odom` velocity. However, it was tracking a fixed offline trajectory and did not use real-time LiDAR returns to re-optimize a local trajectory or stop for a newly observed obstacle.

The missing hard safety behavior was that the real-time LiDAR was used for FAST-LIO localization only, not for near-field obstacle stopping. In addition, the safety exit did not require a fresh avoidance/proximity source before allowing `SAFE_ACTIVE`.

## Fix implemented

- Add `avoidance_ready` to `SafetyContext`; missing or stale avoidance input now produces `SAFE_HOLD` with reason `avoidance_stale`.
- Add a conservative `proximity_stop_node` that reads body-frame point cloud input and publishes `/avoidance/stop_request`.
  It uses the same basic safety idea found in the previous `ranger_delivery_mission`
  docking/clearance code: a front sector with `0.55 m` stop distance, `0.4363 rad`
  half-angle, `3` minimum returns, and `0.5 s` sensor freshness.
- Enable FAST-LIO body-frame cloud output for Day5 control by setting both `scan_publish_en=true` and `scan_bodyframe_pub_en=true`.
- Start `proximity_stop_node` by default in Day5 motion launch.
- Add regression tests:
  - missing avoidance source forces safe hold;
  - Day5 launch/config includes a live obstacle-stop source;
  - points inside the configured near-field stop sector trigger a stop request.

## Remaining risk

This fix is a basic safety stop gate, not a complete obstacle-avoidance module. It can stop for a near-field obstacle or for missing point-cloud input, but it does not plan a path around obstacles. Full local avoidance still requires a local planner/costmap or avoidance module with validated sensor calibration, object filtering, and recovery behavior.
