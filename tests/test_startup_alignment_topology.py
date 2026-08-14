import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class StartupAlignmentTopologyTest(unittest.TestCase):
    def test_localization_package_exposes_alignment_coordinator(self) -> None:
        setup_text = (
            REPO_ROOT / "src" / "competition_localization" / "setup.py"
        ).read_text(encoding="utf-8")

        self.assertIn("startup_alignment_node", setup_text)

    def test_day5_launch_starts_observer_and_coordinator(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("scan_map_residual_monitor_node", launch_text)
        self.assertIn("startup_alignment_node", launch_text)
        self.assertIn("startup_alignment_params_file", launch_text)

    def test_control_adapter_has_fail_closed_alignment_gate(self) -> None:
        control_text = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn("alignment_gate_decision", control_text)
        self.assertIn("/localization/checkpoint_alignment_request", control_text)
        self.assertIn(
            "completion_allowed=alignment_gate.completion_allowed", control_text
        )

    def test_only_anchor_node_owns_map_transform(self) -> None:
        coordinator_text = (
            REPO_ROOT
            / "src"
            / "competition_localization"
            / "competition_localization"
            / "startup_alignment_node.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn("TransformBroadcaster", coordinator_text)
        self.assertNotIn("Twist", coordinator_text)


if __name__ == "__main__":
    unittest.main()
