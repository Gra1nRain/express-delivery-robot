import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5OnlineReplanningTopologyTest(unittest.TestCase):
    def test_live_costmap_drives_reference_aware_local_trajectory(self) -> None:
        planning = yaml.safe_load(
            (REPO_ROOT / "config" / "planning" / "planning_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        control = yaml.safe_load(
            (REPO_ROOT / "config" / "control" / "control_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        safety = yaml.safe_load(
            (REPO_ROOT / "config" / "safety" / "safety_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")
        planning_setup = (
            REPO_ROOT / "src" / "competition_planning" / "setup.py"
        ).read_text(encoding="utf-8")
        replanner_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")
        control_node = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        replanning = planning["replanning"]
        self.assertTrue(replanning["enabled"])
        self.assertEqual(replanning["plugin"], "reference_aware_hybrid_astar")
        self.assertGreaterEqual(replanning["lookahead_distance_m"], 3.0)
        self.assertGreater(replanning["reference_deviation_weight"], 0.0)
        self.assertEqual(
            replanning["costmap_topic"],
            "/avoidance/local_costmap",
        )
        self.assertEqual(
            replanning["local_trajectory_topic"],
            control["visualization"]["local_trajectory_topic"],
        )
        self.assertGreaterEqual(
            safety["proximity_stop"]["grid_x_max_m"],
            replanning["lookahead_distance_m"],
        )
        self.assertGreaterEqual(
            safety["proximity_stop"]["grid_y_max_m"],
            2.0,
        )
        self.assertIn("local_replanner_node", planning_setup)
        self.assertIn("local_replanner_node", launch_text)
        self.assertIn("start_local_replanner", launch_text)
        self.assertIn("OccupancyGrid", replanner_node)
        self.assertIn("LocalTrajectoryPlanner", replanner_node)
        self.assertIn("local_trajectory_topic", control_node)
        self.assertIn("parameterize_local_path", control_node)
        self.assertIn("replace_trajectory", control_node)
        self.assertIn("LOCAL_PLAN_STALE", control_node)


if __name__ == "__main__":
    unittest.main()
