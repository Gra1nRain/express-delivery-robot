import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class MPPIRelocalizationTest(unittest.TestCase):
    def test_initial_pose_reanchors_controller_after_a_settle_window(self) -> None:
        node_text = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn("PoseWithCovarianceStamped", node_text)
        self.assertIn('"/initialpose"', node_text)
        self.assertIn("def _initialpose_callback", node_text)
        self.assertIn("_initial_pose_settle_until_s", node_text)
        self.assertIn('"initial_pose_settling"', node_text)
        self.assertIn("self._state_estimator.reset()", node_text)
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '"initial_pose_settle_s": estimator["initial_pose_settle_s"]',
            launch_text,
        )
        for filename in (
            "control_params.yaml",
            "control_params_day5_chassis_center_yneg020.yaml",
        ):
            config = yaml.safe_load(
                (
                    REPO_ROOT / "config" / "control" / filename
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(
                config["state_estimator"]["initial_pose_settle_s"],
                0.5,
            )


if __name__ == "__main__":
    unittest.main()
