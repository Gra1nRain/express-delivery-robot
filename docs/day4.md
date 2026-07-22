# Day 4 optimized trajectory record

## Facts

- Day4 remains offline-only: no ROS2 custom message, no tracker, no chassis command, and no real-car motion were added.
- `competition_planning.trajectory_parameterizer` converts Day3 `StepPlan` paths into trajectories with `x`, `y`, `yaw`, `s`, `curvature`, `v`, `yaw_rate`, and `t`.
- Stop points are derived from `semantic_map.yaml` `stop_lines[].point_ref` and `dock_poses[].point_ref`. Obstacle-zone entry and exit anchors are pass-through points unless they also appear in those semantic stop sources.
- Current debug speed limits are configured in `config/planning/optimizer_params.yaml`: normal `0.50m/s`, `RANDOM_OBSTACLE` and `CONE_LANE_CHANGE` zones `0.30m/s`, acceleration `0.30m/s^2`, deceleration `0.50m/s^2`, lateral acceleration `0.20m/s^2`.
- `offline_optimized_trajectory` generates a route-level YAML artifact, one CSV per trajectory step, a Markdown summary, and an SVG overview.
- Generated debug artifacts are under `docs/evidence/day4/`.

## Verification

- `python -m unittest tests.test_trajectory_parameterizer`: passed, 3 tests.
- Generated `debug_route` optimized trajectory: 6 trajectories, 0 failures.
- Artifact summary: `docs/evidence/day4/debug_optimized_trajectory_summary.md`.
- Artifact YAML: `docs/evidence/day4/debug_optimized_trajectory.yaml`.
- Artifact SVG: `docs/evidence/day4/debug_optimized_trajectory.svg`.
- Per-step CSV files: `docs/evidence/day4/csv/*.csv`.

## Current Offline Result

| step | points | length_m | duration_s | max_v_mps |
|---|---:|---:|---:|---:|
| `go_traffic_light_1` | 12 | 2.593 | 5.693 | 0.500 |
| `random_obstacle_1` | 36 | 6.673 | 18.900 | 0.500 |
| `cone_lane_change_1` | 53 | 9.049 | 25.225 | 0.500 |
| `return_to_pickup_area` | 59 | 10.143 | 27.891 | 0.500 |
| `cone_lane_change_2` | 53 | 9.049 | 25.225 | 0.500 |
| `finish_park` | 13 | 2.823 | 7.001 | 0.500 |

## Overall Avoidance Plan Adjustment

### Recommendation

- The avoidance group can continue algorithm research, but the main car project must own deployment adaptation and validation on our car.
- The avoidance handoff should include runnable baseline source, exact repo/version, model weights if any, license, input/output schema, dependencies, compute assumptions, launch command, offline sample data, and known failure cases.
- Capability should be staged: first support safety slow/stop on occupied or pedestrian detections with input timeout mapped to `SAFE_HOLD`; then add local detour for random static obstacles and rejoin the global path; keep cone lane change as a semantic planned corridor for now.

### Unverified

- No avoidance algorithm has been deployed or validated in this Day4 work.
- Runtime obstacle detour, pedestrian handling beyond stop/hold, and cone perception integration remain future work.

## Unverified

- The generated trajectories are offline artifacts only; ROS2 publication, tracker consumption, safety gating, and real-car motion are not validated here.
- Curve-speed limiting is implemented from curvature and lateral acceleration, but the debug Bezier geometry still has high local curvature in some segments. This needs tracker and real-car validation before raising speed.
- The initial route start is not forced to zero unless it is configured as a semantic stop. Actual execution should reconcile the trajectory with measured vehicle speed in the tracker/safety layer.
