import math
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.semantic_planner import PathPoint, StepPlan, load_yaml_file
from competition_planning.trajectory_parameterizer import (
    optimize_route_trajectory,
    parameterize_step_plan,
)


class TrajectoryParameterizerTest(unittest.TestCase):
    def test_semantic_stops_and_obstacle_zone_speed_caps_are_applied(self) -> None:
        plan = StepPlan(
            step_id="go_pickup",
            step_type="RUN_SEGMENT",
            corridor_ref="go_to_pickup",
            target_ref="pickup_dock",
            target_source="target_ref",
            planning_time_ms=1.0,
            path=(
                PathPoint(0.0, 0.0, 0.0, ref_id="start"),
                PathPoint(1.0, 0.0, 0.0, ref_id="traffic_light_stop_line"),
                PathPoint(2.0, 0.0, 0.0, ref_id="random_obstacle_entry"),
                PathPoint(3.0, 0.0, 0.0),
                PathPoint(4.0, 0.0, 0.0, ref_id="random_obstacle_exit"),
                PathPoint(5.0, 0.0, 0.0, ref_id="pickup_dock"),
            ),
        )
        semantic_map = {
            "stop_lines": [
                {"id": "traffic_light_stop_line", "point_ref": "traffic_light_stop_line"},
                {"id": "pickup_dock_stop_line", "point_ref": "pickup_dock"},
            ],
            "dock_poses": [
                {"id": "pickup_dock", "point_ref": "pickup_dock", "dock_type": "PICKUP"}
            ],
            "obstacle_zones": [
                {
                    "id": "random_obstacle_zone_1",
                    "semantic_type": "RANDOM_OBSTACLE",
                    "boundary": [[2.0, -1.0], [4.0, -1.0], [4.0, 1.0], [2.0, 1.0]],
                }
            ],
        }
        optimizer_config = {
            "trajectory_optimizer": {
                "max_speed_mps": 0.50,
                "max_acceleration_mps2": 0.30,
                "max_deceleration_mps2": 0.50,
                "max_lateral_acceleration_mps2": 0.20,
                "obstacle_zone_speed_limits_mps": {"RANDOM_OBSTACLE": 0.30},
            }
        }

        trajectory = parameterize_step_plan(plan, semantic_map, optimizer_config)

        by_ref = {point.ref_id: point for point in trajectory.points if point.ref_id}
        self.assertEqual(by_ref["traffic_light_stop_line"].v, 0.0)
        self.assertEqual(by_ref["pickup_dock"].v, 0.0)
        self.assertGreater(by_ref["random_obstacle_entry"].v, 0.0)
        self.assertGreater(by_ref["random_obstacle_exit"].v, 0.0)
        self.assertLessEqual(by_ref["random_obstacle_entry"].v, 0.30)
        self.assertLessEqual(by_ref["random_obstacle_exit"].v, 0.30)
        self.assertEqual([round(point.s, 3) for point in trajectory.points], [0, 1, 2, 3, 4, 5])
        self.assertTrue(all(point.curvature == 0.0 for point in trajectory.points))
        self.assertTrue(all(point.yaw_rate == 0.0 for point in trajectory.points))
        self.assertTrue(
            all(current.t > previous.t for previous, current in zip(trajectory.points, trajectory.points[1:]))
        )

        for previous, current in zip(trajectory.points, trajectory.points[1:]):
            ds = current.s - previous.s
            acceleration = (current.v * current.v - previous.v * previous.v) / (2.0 * ds)
            self.assertLessEqual(acceleration, 0.30 + 1e-9)
            self.assertGreaterEqual(acceleration, -0.50 - 1e-9)
            self.assertFalse(math.isnan(current.t))

    def test_curve_speed_cap_uses_lateral_acceleration_limit(self) -> None:
        plan = StepPlan(
            step_id="turn",
            step_type="RUN_SEGMENT",
            corridor_ref="turn_corridor",
            target_ref="goal",
            target_source="target_ref",
            planning_time_ms=1.0,
            path=(
                PathPoint(0.0, 0.0, 0.0, ref_id="start"),
                PathPoint(1.0, 0.0, 0.0),
                PathPoint(1.0, 1.0, math.pi / 2.0, ref_id="goal"),
            ),
        )
        optimizer_config = {
            "trajectory_optimizer": {
                "max_speed_mps": 1.00,
                "max_acceleration_mps2": 2.00,
                "max_deceleration_mps2": 2.00,
                "max_lateral_acceleration_mps2": 0.20,
            }
        }

        trajectory = parameterize_step_plan(plan, {}, optimizer_config)

        for point in trajectory.points:
            self.assertNotEqual(point.curvature, 0.0)
            expected_cap = math.sqrt(0.20 / abs(point.curvature))
            self.assertLessEqual(point.v, expected_cap + 1e-9)
            self.assertAlmostEqual(point.yaw_rate, point.v * point.curvature)

    def test_public_route_optimizer_generates_six_config_limited_trajectories(self) -> None:
        route = load_yaml_file(REPO_ROOT / "config" / "routes" / "debug_route.yaml")
        semantic_map = load_yaml_file(REPO_ROOT / "maps" / "debug" / "semantic_map.yaml")
        planning_params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "planning_params.yaml"
        )
        optimizer_params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
        )

        result = optimize_route_trajectory(route, semantic_map, planning_params, optimizer_params)

        self.assertTrue(result.ok, result.failures)
        self.assertEqual(
            [trajectory.step_id for trajectory in result.trajectories],
            [
                "go_traffic_light_1",
                "random_obstacle_1",
                "cone_lane_change_1",
                "return_to_pickup_area",
                "cone_lane_change_2",
                "finish_park",
            ],
        )
        self.assertEqual(result.frame_id, "map")
        self.assertEqual(result.route_name, "debug_route")

        traffic = _point_by_ref(result, "go_traffic_light_1", "traffic_light_stop_line")
        pickup = _point_by_ref(result, "random_obstacle_1", "pickup_dock")
        drop = _point_by_ref(result, "cone_lane_change_1", "drop_dock")
        finish = _point_by_ref(result, "finish_park", "finish_park")
        self.assertEqual(traffic.v, 0.0)
        self.assertEqual(pickup.v, 0.0)
        self.assertEqual(drop.v, 0.0)
        self.assertEqual(finish.v, 0.0)

        random_entry = _point_by_ref(result, "random_obstacle_1", "random_obstacle_entry")
        random_exit = _point_by_ref(result, "random_obstacle_1", "random_obstacle_exit")
        cone_entry = _point_by_ref(result, "cone_lane_change_1", "cone_lane_change_entry")
        cone_exit = _point_by_ref(result, "cone_lane_change_1", "cone_lane_change_exit")
        for point in (random_entry, random_exit, cone_entry, cone_exit):
            self.assertGreater(point.v, 0.0)
            self.assertLessEqual(point.v, 0.30 + 1e-9)

        for trajectory in result.trajectories:
            self.assertGreater(trajectory.duration_s, 0.0)
            self.assertGreater(trajectory.path_length_m, 0.0)
            self.assertLessEqual(max(point.v for point in trajectory.points), 0.50 + 1e-9)
            for previous, current in zip(trajectory.points, trajectory.points[1:]):
                self.assertGreaterEqual(current.s, previous.s)
                self.assertGreaterEqual(current.t, previous.t)
                ds = current.s - previous.s
                if ds > 1e-9:
                    acceleration = (current.v * current.v - previous.v * previous.v) / (2.0 * ds)
                    self.assertLessEqual(acceleration, 0.30 + 1e-9)
                    self.assertGreaterEqual(acceleration, -0.50 - 1e-9)
            sample = trajectory.points[0].to_dict()
            for key in ("x", "y", "yaw", "s", "curvature", "v", "yaw_rate", "t"):
                self.assertIn(key, sample)

    def test_offline_cli_writes_yaml_and_per_step_csv(self) -> None:
        script = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "offline_optimized_trajectory.py"
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = pathlib.Path(tmp_dir)
            output_yaml = tmp_path / "optimized_trajectory.yaml"
            csv_dir = tmp_path / "csv"
            report_path = tmp_path / "summary.md"
            svg_path = tmp_path / "overview.svg"
            env = os.environ.copy()
            python_path = str(REPO_ROOT / "src" / "competition_planning")
            env["PYTHONPATH"] = (
                python_path
                if not env.get("PYTHONPATH")
                else python_path + os.pathsep + env["PYTHONPATH"]
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--route",
                    str(REPO_ROOT / "config" / "routes" / "debug_route.yaml"),
                    "--semantic-map",
                    str(REPO_ROOT / "maps" / "debug" / "semantic_map.yaml"),
                    "--planning-params",
                    str(REPO_ROOT / "config" / "planning" / "planning_params.yaml"),
                    "--optimizer-params",
                    str(REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"),
                    "--output",
                    str(output_yaml),
                    "--csv-dir",
                    str(csv_dir),
                    "--report",
                    str(report_path),
                    "--svg",
                    str(svg_path),
                ],
                capture_output=True,
                env=env,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = load_yaml_file(output_yaml)
            self.assertEqual(len(output["trajectories"]), 6)
            self.assertIn(
                "route=debug_route ok=True trajectories=6 failures=0",
                completed.stdout,
            )
            csv_files = sorted(path.name for path in csv_dir.glob("*.csv"))
            self.assertEqual(
                csv_files,
                [
                    "cone_lane_change_1.csv",
                    "cone_lane_change_2.csv",
                    "finish_park.csv",
                    "go_traffic_light_1.csv",
                    "random_obstacle_1.csv",
                    "return_to_pickup_area.csv",
                ],
            )
            header = (csv_dir / "go_traffic_light_1.csv").read_text(
                encoding="utf-8"
            ).splitlines()[0]
            self.assertEqual(header, "x,y,yaw,s,curvature,v,yaw_rate,t,ref_id")
            self.assertIn(
                "Day 4 优化轨迹摘要",
                report_path.read_text(encoding="utf-8"),
            )
            self.assertIn("<svg", svg_path.read_text())


def _point_by_ref(result, step_id: str, ref_id: str):
    trajectory = next(item for item in result.trajectories if item.step_id == step_id)
    return next(point for point in trajectory.points if point.ref_id == ref_id)


if __name__ == "__main__":
    unittest.main()
