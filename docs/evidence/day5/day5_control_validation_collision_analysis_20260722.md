# Day 5 control_validation collision analysis

Date: 2026-07-22

## Artifacts

- Code under test: `b2fd838 Add Day5 control validation route`
- Bag on car: `/home/agilex/competition_ws/recordings/day5_control_validation_b2fd838_`
- Motion launch log on car: `/home/agilex/competition_ws/log/day5_control_validation_motion_b2fd838.log`

## Facts from the run

- The run used `debug_control_validation_trajectory.yaml` with matching `debug_control_validation_route.yaml` and `semantic_map_control_validation.yaml` source-manifest inputs.
- The bag contains about `219.921s` of data and `80226` messages, including `/tf`, `/odom`, `/Odometry`, `/system_state`, `/motion_state`, `/control/*`, `/safety/event`, `/avoidance/stop_request`, `/avoidance/proximity_status`, and `/cmd_vel`.
- `/initialpose` was published 5 times at `x=-0.376, y=0.112, yaw=0.020rad`.
- The controller entered `TRACKING` at about `t=67.031s`.
- During the autonomous phase immediately before operator takeover, around `t=115-124s`:
  - `/control/status` stayed `TRACKING`.
  - `/safety/event` stayed `SAFE_ACTIVE`.
  - lateral tracking error was about `0.012-0.018m`.
  - heading error was about `0.008-0.012rad`.
  - `/cmd_vel.linear.x` was about `0.19m/s`.
  - `/avoidance/proximity_status` stayed `{"stop": false, "reason": "clear", "point_count": 0}` with fresh cloud ages.
- At about `t=124.52s`, `/system_state.control_mode` changed from CAN control to mode `3`; safety switched to `SAFE_HOLD` with reason `remote_not_ready`, and `/cmd_vel` went to zero.
- After takeover, `/odom.twist.twist.linear.x` reached about `-0.63m/s` while `/cmd_vel` stayed zero. This period is outside the autonomous `/cmd_vel` command stream.
- The first `lateral_error_exceeded` safety reason occurred after takeover, at about `t=130.02s`.
- The maximum lateral tracking error in the whole bag was about `0.511m`, also after takeover.
- The control-validation trajectory index `84-88` corresponds to about `x=8.02-8.42, y=0.37` in the planned map frame. Offline static-map clearance near that segment looked safe against the recorded occupancy map; the nearest occupied cells were roughly `0.79-0.84m` from the trajectory center.

## Conclusion

The second collision was not primarily a low-level trajectory tracking failure. In the autonomous phase before takeover, the vehicle was tracking the frozen trajectory closely and safety was active.

The exposed safety gap is that the real-time near-field stop gate did not detect the shelf:

- `proximity_stop_node` used a narrow forward sector and a `0.55m` stop distance.
- The Day5 FAST-LIO control config has `preprocess.blind: 0.5`, so the previous stop distance left only a very small usable detection band beyond FAST-LIO's blind range.
- The detector did not cover the full front-side vehicle safety corridor; points that are outside the `0.4363rad` forward sector but still inside the vehicle-width safety envelope could be ignored.
- The static offline map/footprint check is not sufficient for moved shelves, map alignment error, or obstacle geometry missing from the map.

## Fix implemented

- Extend `ProximityStopConfig` with `lateral_half_width_m`.
- Count an obstacle point if it falls in either:
  - the original forward sector, or
  - the rectangular front vehicle corridor `x_min_m <= x <= stop_distance_m` and `abs(y) <= lateral_half_width_m`.
- Set Day5 `proximity_stop.stop_distance_m` to `0.85m`, which is at least `0.25m` beyond the FAST-LIO `0.5m` blind range.
- Set Day5 `proximity_stop.lateral_half_width_m` to `0.45m`, matching the earlier offline sweep envelope of `0.50m` vehicle width plus `0.20m` clearance.
- Pass the new parameter through `day5_motion_control.launch.py`.
- Add regression tests:
  - side-front points inside the vehicle corridor trigger stop;
  - Day5 proximity stop distance must extend beyond FAST-LIO blind range;
  - Day5 proximity lateral half-width must cover vehicle half-width plus clearance.

## Verification

- `python -m unittest tests.test_proximity_stop` -> `3 tests OK`
- `python -m unittest tests.test_day5_obstacle_safety_topology` -> `1 test OK`
- `python -m unittest discover -s tests` -> `49 tests OK`

## Remaining risk

This is still a hard stop gate, not a full online local planner. It can stop for near-field points inside the configured safety corridor, but it cannot plan around shelves or prove that the static map matches the current room layout. Any further real motion should first do a no-motion hardware check that validates `/avoidance/proximity_status.point_count` rises when a real obstacle is placed in the expanded corridor.
