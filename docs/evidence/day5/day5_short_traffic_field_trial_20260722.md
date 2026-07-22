# Day 5 Short Traffic Segment Field Trial

## Facts

- Date: 2026-07-22
- Vehicle pose before run: at debug `start`
- Route artifact: `docs/evidence/day5/debug_traffic_light_trajectory.yaml`
- Trajectory length: `2.600m`
- Launch command output: `/cmd_vel`
- FAST-LIO config: `config/mapping/fast_lio_mid360_day5_control.yaml`
- Safety confirmation before run: estop valid, remote takeover available, front 3m clear
- Bag path on car: `/home/agilex/competition_ws/recordings/day5_short_traffic_804336e_20260722_191755`
- Bag size: `9.6MiB`
- Monitor log on car: `/home/agilex/competition_ws/log/day5_short_motion_monitor_804336e.txt`
- Launch log on car: `/home/agilex/competition_ws/log/day5_short_motion_804336e.log`

## Results

- `/cmd_vel` topology before `/initialpose`: one publisher `competition_safety`, subscribers `ranger_base_node` and `rosbag2_recorder`.
- Pre-anchor command and odometry were zero; controller reported `INVALID_STATE`, safety reported `SAFE_HOLD`.
- `/initialpose` publisher saw one subscriber before publish.
- First `TRACKING`: `1.006s` after monitor start.
- First `GOAL_REACHED`: `21.573s` after monitor start.
- Max `/cmd_vel.linear.x`: `0.1979m/s`.
- Max `/odom.twist.twist.linear.x`: `0.1990m/s`.
- Final `/cmd_vel`: `(0.0, 0.0)`.
- Final `/odom` velocity: `(0.0, 0.0)`.
- Final tracking error: lateral `0.013m`, heading `0.016rad`, target index `26`.
- Control counts: `INVALID_STATE=6`, `TRACKING=410`, `GOAL_REACHED=85`.
- Safety counts: `SAFE_HOLD=4`, `SAFE_ACTIVE=395`, `SAFE_LIMITED=25`, `SAFE_STOP=74`.
- State-valid counts: `false=6`, `true=495`.

## Notes

- The short segment completed and stopped under the Day5 MPPI + SafetySupervisor chain.
- The initial transient `INVALID_STATE` / `SAFE_HOLD` happened before or during TF anchoring and did not persist after tracking started.
- Rosbag topic metadata reported `/initialpose` count `0`; the publish event is recorded in the monitor log, not in the bag.
- After shutdown, no Day5, Livox, FAST-LIO, Ranger, MPPI, safety, or rosbag process remained, and ROS topics returned to `/parameter_events` and `/rosout`.

## Not Verified

- This does not prove the `drop_dock` half route or full continuous route.
- This does not validate semantic task stops, traffic-light logic, obstacle avoidance, arm tasks, or docking closed loops.
- Video/operator observation is external to this repository record.
