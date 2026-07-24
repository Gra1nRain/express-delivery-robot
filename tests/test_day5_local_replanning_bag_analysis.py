import importlib.util
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "analyze_day5_local_replanning_bag.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "analyze_day5_local_replanning_bag",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Day5LocalReplanningBagAnalysisTest(unittest.TestCase):
    def test_grid_projection_respects_rotated_origin(self) -> None:
        module = _load_module()
        grid = module.GridSnapshot(
            timestamp_ns=0,
            frame_id="body",
            resolution_m=1.0,
            width=3,
            height=2,
            origin=module.Transform2D(x=10.0, y=20.0, yaw=1.5707963267948966),
            data=(0, 100, 0, 0, 0, 0),
        )

        self.assertEqual(grid.cell_for_point(9.5, 21.5), (1, 0))
        self.assertTrue(grid.point_is_blocked(9.5, 21.5, inflation_radius_m=0.0))

    def test_inflation_matches_planner_cell_radius_semantics(self) -> None:
        module = _load_module()
        grid = module.GridSnapshot(
            timestamp_ns=0,
            frame_id="body",
            resolution_m=0.5,
            width=5,
            height=5,
            origin=module.Transform2D(),
            data=tuple(
                100 if (index % 5, index // 5) == (2, 2) else 0
                for index in range(25)
            ),
        )

        self.assertFalse(grid.point_is_blocked(1.75, 0.75, inflation_radius_m=0.5))
        self.assertTrue(grid.point_is_blocked(1.75, 1.25, inflation_radius_m=0.5))

    def test_transform_graph_composes_map_to_body(self) -> None:
        module = _load_module()
        transforms = {
            ("map", "camera_init"): module.Transform2D(x=2.0, y=3.0, yaw=0.0),
            ("camera_init", "body"): module.Transform2D(x=1.0, y=0.0, yaw=0.0),
        }

        map_from_body = module.lookup_transform(transforms, "map", "body")
        body_from_map = module.lookup_transform(transforms, "body", "map")

        self.assertEqual(map_from_body.apply(0.0, 0.0), (3.0, 3.0))
        self.assertEqual(body_from_map.apply(3.0, 3.0), (0.0, 0.0))

    def test_report_fails_on_large_local_to_global_deviation(self) -> None:
        module = _load_module()
        event = module.ReplanEventMetrics(
            timestamp_ns=2_000_000_000,
            elapsed_s=1.0,
            status="REPLANNED",
            reference_start_index=4,
            rejoin_index=8,
            dynamic_obstacle_count=0,
            local_path_point_count=3,
            max_local_to_global_deviation_m=1.2,
            p95_local_to_global_deviation_m=1.1,
            reference_point_count=5,
            dynamic_blocked_reference_points=0,
            static_blocked_reference_points=0,
            combined_blocked_reference_points=0,
        )

        report = module.build_report(
            [event],
            max_local_deviation_m=0.5,
            tf_edges=(("camera_init", "body"), ("map", "camera_init")),
        )

        self.assertFalse(report.passed)
        self.assertEqual(report.peak_event.max_local_to_global_deviation_m, 1.2)
        self.assertEqual(report.high_deviation_clear_reference_events, 1)
        self.assertIn("max_local_to_global_deviation_m", report.failed_checks[0])

    def test_report_fails_on_long_local_status_gap(self) -> None:
        module = _load_module()
        event = module.ReplanEventMetrics(
            timestamp_ns=2_000_000_000,
            elapsed_s=2.0,
            status="REPLANNED",
            reference_start_index=4,
            rejoin_index=8,
            dynamic_obstacle_count=0,
            local_path_point_count=3,
            max_local_to_global_deviation_m=0.1,
            p95_local_to_global_deviation_m=0.08,
            reference_point_count=5,
            dynamic_blocked_reference_points=0,
            static_blocked_reference_points=0,
            combined_blocked_reference_points=0,
        )

        report = module.build_report(
            [event],
            max_local_deviation_m=0.5,
            max_status_gap_s=2.5,
            status_timestamps_ns=(
                1_000_000_000,
                2_000_000_000,
                32_000_000_000,
            ),
            status_payloads=(
                {"status": "REFERENCE_CLEAR"},
                {"status": "REPLANNED"},
                {
                    "status": "PLAN_FAILED",
                    "detail": "hybrid_astar found no path",
                },
            ),
            bag_start_ns=1_000_000_000,
            planning_times_ms=(80.0, 29_500.0),
            tf_edges=(),
        )

        self.assertFalse(report.passed)
        self.assertEqual(report.status_message_count, 3)
        self.assertEqual(report.max_status_gap_s, 30.0)
        self.assertEqual(report.max_status_gap_start_elapsed_s, 1.0)
        self.assertEqual(report.max_status_gap_end_elapsed_s, 31.0)
        self.assertEqual(report.max_status_gap_before_status, "REPLANNED")
        self.assertEqual(report.max_status_gap_after_status, "PLAN_FAILED")
        self.assertEqual(
            report.max_status_gap_after_detail,
            "hybrid_astar found no path",
        )
        self.assertEqual(report.max_reported_planning_time_ms, 29_500.0)
        self.assertTrue(
            any("max_local_status_gap_s" in item for item in report.failed_checks)
        )


if __name__ == "__main__":
    unittest.main()
