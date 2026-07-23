import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5ChassisCenterTrackingProfileTest(unittest.TestCase):
    def test_launch_separates_localization_anchor_from_tracking_frame(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("localization_base_frame", launch_text)
        self.assertIn("tracking_base_frame", launch_text)
        self.assertIn("tracking_frame_transform", launch_text)
        self.assertIn("static_transform_publisher", launch_text)
        self.assertIn("--frame-id", launch_text)
        self.assertIn("--child-frame-id", launch_text)
        self.assertIn('"anchor_base_frame": localization_base_frame', launch_text)
        self.assertIn('"base_frame": tracking_base_frame', launch_text)

    def test_right_bias_profile_tracks_chassis_center_without_reanchoring_scan(self) -> None:
        control_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "control"
                / "control_params_day5_chassis_center_yneg020.yaml"
            ).read_text(encoding="utf-8")
        )

        estimator = control_config["state_estimator"]
        transform = estimator["tracking_frame_transform"]

        self.assertEqual(estimator["localization_base_frame"], "body")
        self.assertEqual(estimator["tracking_base_frame"], "chassis_center")
        self.assertEqual(transform["parent_frame"], "body")
        self.assertEqual(transform["child_frame"], "chassis_center")
        self.assertLess(transform["y_m"], 0.0)
        self.assertAlmostEqual(abs(transform["y_m"]), 0.20, places=3)

    def test_right_guard_profile_covers_reported_right_front_table_corner(self) -> None:
        safety_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "safety"
                / "safety_params_day5_right_guard_heading30.yaml"
            ).read_text(encoding="utf-8")
        )

        proximity = safety_config["proximity_stop"]
        self.assertEqual(proximity["stop_distance_m"], 0.70)
        self.assertGreaterEqual(proximity["lateral_half_width_m"], 0.60)
        self.assertLessEqual(
            proximity["grid_y_min_m"],
            -proximity["lateral_half_width_m"],
        )
        self.assertGreaterEqual(
            proximity["grid_y_max_m"],
            proximity["lateral_half_width_m"],
        )


if __name__ == "__main__":
    unittest.main()
