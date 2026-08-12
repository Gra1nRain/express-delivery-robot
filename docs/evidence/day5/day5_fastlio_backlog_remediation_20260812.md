# Day 5 FAST-LIO backlog remediation — 2026-08-12

## Scope

This work addressed the Livox-to-FAST-LIO point-cloud backlog only. The
`pickup_front` local-planner hold was intentionally left unchanged. All vehicle
tests were stationary and terminal 8 was not started.

## Reproduced failure

With Livox, FAST-LIO, the full navigation/safety stack, RViz, ToDesk, and the
passive trip diagnostics running:

- `/livox/lidar` remained approximately 10 Hz;
- `/livox/imu` remained approximately 200 Hz for an independent subscriber;
- `/cloud_registered_body` fell to approximately 0.775 Hz;
- `/Odometry` fell to approximately 0.79 Hz;
- cloud P95 stamp age was approximately 1.38–1.49 s;
- the machine still had approximately 54% idle CPU and no swap or I/O wait.

FAST-LIO's existing runtime timing showed that mapping computation itself took
only approximately 9–15 ms per processed scan. New boundary telemetry then
showed the actual failure:

- LiDAR callback: 9.9–10 Hz;
- LiDAR preprocessing: approximately 0.3–0.8 ms;
- mapping timer: approximately 100 Hz;
- FAST-LIO IMU callback: only approximately 5–42 Hz;
- `imu_minus_lidar_end_s`: repeatedly -0.4 to -1.3 s;
- selected LiDAR frame remained pinned while waiting for IMU.

At the same time, an independent `/livox/imu` subscriber received approximately
200 Hz. The failure therefore occurred at FAST-LIO's Reliable IMU subscription,
not at the MID360 hardware, Livox IMU publisher, point-cloud preprocessing,
mapping computation, RViz rendering, or total machine capacity.

## Remediation

The computer repository now contains the complete external FAST-LIO patch and
Release rebuild chain used on the vehicle. The new fixes:

1. subscribe to the high-rate IMU with `SensorDataQoS`, BEST_EFFORT, and a
   200-sample latest window so a cold Reliable connection cannot remain in a
   slow retransmission/catch-up state;
2. run LiDAR, IMU, and mapping callback groups on dedicated single-threaded
   executors so the 200 Hz IMU callback cannot be starved by the 100 Hz mapping
   timer or LiDAR deserialization;
3. report callback rates, preprocessing time, buffer depth, LiDAR/IMU time gap,
   and stale-sync drops through `FAST_LIO_HEALTH`;
4. discard a selected LiDAR frame when it is more than 0.35 s behind the latest
   LiDAR time and IMU still has not caught up, rather than publishing indefinitely
   stale localization output;
5. add `day5_sensor_sync_probe.py` for latest-only external timing diagnosis.

The freshness gate remains a safety precondition, not the mechanism that fixes
the backlog. The repaired FAST-LIO reached fresh output immediately after DDS
discovery in the cold-start tests.

## Verification facts

- The pre-fix cold-start sequence reproduced internal IMU delivery of only
  10–42 Hz and persistent selected-frame waits.
- The same cold-start sequence after the QoS/executor fix reached approximately
  181 Hz during the first partial window and then remained approximately
  199–201 Hz.
- LiDAR callbacks remained approximately 9.9–10 Hz.
- `stale_sync_drops` remained 0 in normal operation.
- Eight consecutive full-stack stationary cloud gates passed with P95 age
  approximately 0.034–0.042 s, versus the 0.35 s limit.
- A final direct frequency sample averaged approximately 10 Hz.
- Machine CPU retained approximately 57% idle capacity.
- Vehicle FAST-LIO built successfully in Release with `-O3 -DNDEBUG`.
- Repository project tests: `209 passed in 47.05s`.
- Targeted FAST-LIO patch tests: `12 passed`.

## Residual risks

- No moving-vehicle validation was performed in this change; motion requires a
  separate authorized test and a new initial-pose alignment.
- During a heavy on-vehicle C++ build, Livox point-cloud publication temporarily
  paused while IMU continued. It recovered after the build. Building software
  is outside competition runtime, but sensor freshness must remain a hard
  no-departure condition.
- A direct 10 Hz sample contained a rare approximately 0.33 s inter-arrival gap
  followed by catch-up frames. The eight P95 freshness windows stayed well below
  the safety limit; maximum-jitter behavior should remain observed during the
  next stationary and moving trial.
- The unrelated `pickup_front` planner `SAFETY_HOLD` remains open by design.
