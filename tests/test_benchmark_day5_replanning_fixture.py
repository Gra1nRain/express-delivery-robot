import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
PLANNING_SRC = REPO_ROOT / "src" / "competition_planning"
SCRIPTS_ROOT = REPO_ROOT / "scripts"
for path in (PLANNING_SRC, SCRIPTS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from benchmark_day5_replanning_fixture import config_from_mapping  # noqa: E402


class BenchmarkDay5ReplanningFixtureTest(unittest.TestCase):
    def test_config_mapping_applies_expansion_override(self) -> None:
        config = config_from_mapping(
            {
                "lookahead_distance_m": 3.0,
                "inflation_radius_m": 0.3,
                "search_padding_m": 1.5,
                "goal_position_tolerance_m": 0.25,
                "goal_heading_tolerance_deg": 15.0,
                "reference_deviation_weight": 2.0,
                "max_expansions": 250_000,
                "reference_search_window_points": 120,
            },
            max_expansions=20_000,
        )

        self.assertEqual(config.max_expansions, 20_000)
        self.assertEqual(config.sample_spacing_m, 0.10)
        self.assertEqual(config.min_turning_radius_m, 0.81)
        self.assertEqual(config.step_length_m, 0.20)
        self.assertEqual(config.curvature_bins, 9)
        self.assertEqual(config.heading_bins, 72)
        self.assertAlmostEqual(
            config.goal_heading_tolerance_rad,
            math.radians(15.0),
        )


if __name__ == "__main__":
    unittest.main()
