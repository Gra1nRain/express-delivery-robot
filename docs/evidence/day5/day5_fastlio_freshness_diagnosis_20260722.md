# Day 5 FAST-LIO freshness diagnosis

Date: 2026-07-22

## Scope and safety

This diagnosis used existing rosbag artifacts and two hardware runs with
`start_base:=false`, `command_output_topic:=/cmd_vel_safe`, and no
`/initialpose`. `/cmd_vel_safe` had zero subscribers and `/cmd_vel` did not
exist. No chassis motion was enabled.

## Feedback loop

`scripts/analyze_day5_sensor_freshness.py` compares each bag record timestamp
with the embedded header stamp on `/cloud_registered_body` and `/Odometry`.
It also checks `/avoidance/proximity_status` for `stale_cloud`. The default
verdict allows at most 1% transient threshold violations after a 5 s warmup;
this avoids treating two isolated scheduling outliers in the known-fresh bag
as a sustained backlog.

Example command on the car:

```bash
python3 scripts/analyze_day5_sensor_freshness.py \
  /home/agilex/competition_ws/recordings/day5_control_validation_full_f06b794_20260722_220933
```

## Facts from recorded bags

Known-fresh short run:

- Bag: `day5_short_control_validation_b6f978e_20260722_215248`.
- `/cloud_registered_body`: p50 `0.051 s`, p95 `0.072 s`, maximum
  `0.301 s`, 0 frames over `0.50 s`.
- `/Odometry`: p50 `0.048 s`, p95 `0.070 s`, maximum `0.277 s`, 2 of
  2478 frames over `0.20 s` (`0.08%`).
- Proximity status: 2479 `clear`, 0 `stale_cloud`.
- Verdict: PASS.

First full-route startup bag:

- Bag: `day5_control_validation_full_f06b794_20260722_220500`.
- `/cloud_registered_body`: 549 of 2359 frames over `0.50 s` (`23.27%`),
  first violation at `187.370 s`, p95 `1.497 s`, maximum `2.020 s`.
- `/Odometry`: 551 of 2359 frames over `0.20 s` (`23.36%`), first
  violation at `157.505 s`, p95 `1.495 s`, maximum `2.013 s`.
- Proximity status: 548 of 2358 frames were `stale_cloud` (`23.24%`).
- Verdict: FAIL.

Second full-route startup bag:

- Bag: `day5_control_validation_full_f06b794_20260722_220933`.
- `/cloud_registered_body`: 748 of 1064 frames over `0.50 s` (`70.30%`),
  first violation at `38.184 s`, p50 `1.378 s`, maximum `2.069 s`.
- `/Odometry`: 749 of 1064 frames over `0.20 s` (`70.39%`), first
  violation at `36.905 s`, p50 `1.375 s`, maximum `2.068 s`.
- Proximity status: 748 of 1064 frames were `stale_cloud` (`70.30%`).
- Repeating the command produced the same metrics and FAIL verdict.

## Facts from no-motion reproduction

With the normal proximity node enabled, the core Day5 stack reproduced the
problem without the Ranger base or rosbag recording:

- At elapsed `1:57`, FAST-LIO used about `36.5%` CPU and proximity used about
  `20.8%`; proximity reported cloud age `0.065 s`.
- At elapsed `3:11`, proximity still reported cloud age `0.064 s`.
- Later in the same run, `/Odometry` delay was about `1.31-1.76 s` and
  proximity reported `stale_cloud` with cloud age `1.741 s`.

A single-variable A/B run set only `start_proximity_stop:=false`. It stayed
fresh for almost six minutes:

- At elapsed `5:53`, `/Odometry` delay remained about `0.038-0.074 s`.
- FAST-LIO used about `38.3%` CPU; total machine CPU was not saturated on the
  eight-core computer.

Both launches were stopped with SIGINT and left no Day5 processes running.
The Livox driver exited with code `-11` during shutdown in both trials; that
shutdown defect is separate from the runtime freshness result.

## Facts from the installed FAST-LIO source

Read-only inspection of the car's installed FAST-LIO source found:

- `rclcpp::spin` uses the default single-threaded executor.
- The Livox subscription uses queue depth 20.
- LiDAR preprocessing and the 100 Hz mapping timer therefore share one
  executor thread.
- With `scan_publish_en=true`, every processed scan publishes both
  `/cloud_registered` and `/cloud_registered_body`.
- `publish_frame_body` always builds its message from the full
  `feats_undistort` cloud; it does not honor `dense_publish_en=false`.
- The current Python proximity subscription uses the default reliable QoS
  with depth 10 and scans all received points.

The approximately 2 s maximum observed output age is consistent with a
10 Hz input accumulating close to the FAST-LIO queue depth of 20.

## Current inference

The evidence supports runtime backpressure/throughput loss in the FAST-LIO
body-cloud delivery path. The most likely immediate trigger is the reliable,
buffered delivery of a dense body cloud to the Python proximity subscriber,
not an incorrect age calculation and not a stopped sensor publisher.

This is still an inference. The next falsification test should change only the
proximity cloud subscription to best-effort, keep-last-1 QoS while preserving
the same point-cloud safety behavior. Acceptance requires a no-motion run of
at least six minutes with `/Odometry` and body-cloud freshness below their
thresholds, followed by a synthetic point-cloud stop test.

## Remaining P0 risk

Fixing freshness will only restore the current hard-stop gate. It will not add
online local replanning or prove that the vehicle can avoid a shelf. The
online local avoidance/costmap debt remains P0 OPEN.
