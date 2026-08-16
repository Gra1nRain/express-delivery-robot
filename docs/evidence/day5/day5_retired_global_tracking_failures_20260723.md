# Day 5 retired global-tracking failure evidence (2026-07-23)

## Status and scope

This page preserves the field facts that were previously scattered across four
temporary continuation summaries. The corresponding hand-built trajectories,
old safety profiles, and raw copied run directories have been retired because
the current Day 5 stack uses online local replanning and the indoor one-lap
trajectory.

**Do not use the retired coordinates, continuations, or safety thresholds to
launch the vehicle.** They are historical diagnostic evidence only.

## Preserved field facts

1. A close-pass run stopped on a right-front proximity condition. Its last
   executed pose was approximately `(8.3947, 0.1173, yaw=-0.1960 rad)`. The
   estimated obstacle position was `(9.009, -0.331)` in the map frame.
2. A temporary left-bypass run reached approximately
   `(9.8011, 0.6883, yaw=1.0873 rad / 62.30 deg)`. It cleared the right-front
   obstacle, then stalled with heading error about `-20.18 deg` and zero safety
   output.
3. A later arc-rejoin run reached approximately
   `(9.5308, 1.6429, yaw=1.9929 rad / 114.19 deg)`. Proximity remained clear,
   but the run stalled when heading error reached about `-20.45 deg`, exceeding
   the then-active `20 deg` safety gate.
4. A subsequent run using the temporary heading-30 profile progressed to
   approximately `(1.5191, 3.1880, yaw=-2.8045 rad / -160.69 deg)`, then
   stopped when lateral error reached about `0.17 m`, exceeding the then-active
   `0.15 m` gate.

## Retired, unverified responses

The following responses were generated from those stop poses but were never
accepted as general solutions:

- a left-offset continuation that ramped to `0.18 m` and faded back to the
  source route;
- a synthetic arc with curvature `1.2 1/m` and length `1.15 m`;
- pose-specific straight/bridge rejoins;
- relaxed `heading30` and reduced-width proximity profiles.

These artifacts were explicitly marked **temporary and unverified**. Their
failure modes helped motivate the current online local-planning and unified
safety configuration; they are not current route or control inputs.

## Original artifact names

The deleted tracked files remain recoverable from Git history under these
prefixes:

- `debug_continuous_trajectory_global_tracking_left_bypass_from_idx89*`
- `debug_continuous_trajectory_global_tracking_arc_rejoin_from_pose_9801_0688*`
- `debug_continuous_trajectory_global_tracking_from_pose_9531_1643*`
- `debug_continuous_trajectory_global_tracking_from_pose_1519_3188*`

