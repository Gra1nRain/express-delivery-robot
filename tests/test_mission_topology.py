import math
import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)


class MissionTopologyTest(unittest.TestCase):
    def test_arm_action_contains_agreed_task_phases_and_outcomes(self) -> None:
        action = (
            REPO_ROOT
            / "src"
            / "competition_interfaces"
            / "action"
            / "ArmTask.action"
        ).read_text(encoding="utf-8")

        for token in (
            "PICKUP=1",
            "DROP=2",
            "INSTRUCTION_NOT_FOUND=2",
            "TARGET_NOT_FOUND=3",
            "RECOGNIZING_INSTRUCTION=2",
            "TARGET_TYPE_LOCKED=3",
            "VERIFYING_OPERATION=6",
        ):
            self.assertIn(token, action)

    def test_mission_node_owns_explicit_release_and_vision_enable(self) -> None:
        node = (
            REPO_ROOT
            / "src"
            / "competition_mission"
            / "competition_mission"
            / "mission_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"/mission/checkpoint_release"', node)
        self.assertIn('"/perception/traffic_light_enable"', node)
        self.assertIn('"/perception/traffic_stop_enable"', node)
        self.assertIn("ActionClient", node)
        self.assertIn("ArmTask", node)

    def test_mission_shutdown_does_not_publish_after_ros_context_closes(self) -> None:
        node = (
            REPO_ROOT
            / "src"
            / "competition_mission"
            / "competition_mission"
            / "mission_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "finally:\n"
            "        if rclpy.ok():\n"
            "            node._route_enable_publisher.publish(Bool(data=False))\n"
            "            node._traffic_enable_publisher.publish(Bool(data=False))",
            node,
        )

    def test_arm_simulator_uses_humble_compatible_sync_callback(self) -> None:
        simulator = (
            REPO_ROOT
            / "src"
            / "competition_mission"
            / "competition_mission"
            / "arm_task_simulator_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn("def _execute(self, goal_handle):", simulator)
        self.assertIn("time.sleep(self._phase_delay_s)", simulator)
        self.assertNotIn("asyncio.sleep", simulator)

    def test_control_waits_for_release_and_reports_non_stop_marker(self) -> None:
        state_machine = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "segmented_route_state_machine.py"
        ).read_text(encoding="utf-8")
        control_node = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('WAIT_RELEASE = "WAIT_RELEASE"', state_machine)
        self.assertIn('"/mission/checkpoint_release"', control_node)
        self.assertIn('"/mission/marker_passed"', control_node)

    def test_indoor_route_has_traffic_stop_and_pretrigger_marker(self) -> None:
        route = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "routes"
                / "indoor_competition_mission_route.yaml"
            ).read_text(encoding="utf-8")
        )

        self.assertEqual(route["mission_checkpoints"][0], "traffic_light_stop_line")
        self.assertEqual(
            route["mission_markers"],
            [
                {
                    "id": "traffic_light_vision_on",
                    "before_checkpoint_ref": "traffic_light_stop_line",
                    "trigger_distance_m": 1.0,
                }
            ],
        )

    def test_integrated_launch_keeps_physical_motion_gates_off(self) -> None:
        launch = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "indoor_competition.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"start_base", default_value="false"', launch)
        self.assertIn('"start_chassis_adapter", default_value="false"', launch)
        self.assertIn('"start_wrist_camera", default_value="true"', launch)
        self.assertIn('"start_real_arm", default_value="false"', launch)
        self.assertIn('"start_arm_simulator", default_value="false"', launch)
        self.assertIn(
            '"arm_post_instruction_clear_delay_s",\n'
            '                default_value="0.0"',
            launch,
        )
        self.assertIn('"start_competition_mission": "true"', launch)
        self.assertIn('"start_wrist_traffic_perception": "true"', launch)

    def test_manual_field_runbook_matches_motion_gates_and_trajectory_start(self) -> None:
        runbook = (
            REPO_ROOT / "docs" / "competition_mission_manual_field_test.md"
        ).read_text(encoding="utf-8")
        trajectory = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "indoor_competition_mission_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        start = trajectory["points"][0]

        for gate in (
            "start_base:=true",
            "start_chassis_adapter:=true",
            "start_real_arm:=true",
            "start_arm_simulator:=false",
            "arm_post_instruction_clear_delay_s:=10.0",
        ):
            self.assertIn(gate, runbook)
        self.assertIn(f"x: {start['x']:.3f}", runbook)
        self.assertIn(f"y: {start['y']:.3f}", runbook)
        self.assertIn(f"z: {math.sin(start['yaw'] / 2.0):.7f}", runbook)
        self.assertIn(f"w: {math.cos(start['yaw'] / 2.0):.5f}", runbook)
        self.assertIn("export CAN_NAME=can2", runbook)
        self.assertIn("export PIPER_CAN_NAME=can2", runbook)
        self.assertIn(
            "/left_wrist_camera/camera/color/image_raw",
            runbook,
        )
        self.assertGreaterEqual(
            runbook.count("arm_post_instruction_clear_delay_s:=10.0"),
            2,
        )
        self.assertIn("Action servers: 1", runbook)

    def test_real_arm_adapter_is_persistent_and_reuses_shared_camera(self) -> None:
        launch = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "indoor_competition.launch.py"
        ).read_text(encoding="utf-8")
        setup = (
            REPO_ROOT / "src" / "competition_mission" / "setup.py"
        ).read_text(encoding="utf-8")
        backend = (
            REPO_ROOT
            / "src"
            / "competition_mission"
            / "competition_mission"
            / "piper_arm_backend.py"
        ).read_text(encoding="utf-8")

        self.assertIn('executable="piper_arm_task_node"', launch)
        self.assertNotIn('name="piper_arm_task"', launch)
        self.assertIn('"manage_camera": False', launch)
        self.assertIn("piper_arm_task_node =", setup)
        self.assertIn(
            "grasp_module.execute_place_after_grasp = lambda",
            backend,
        )
        self.assertIn(
            "self.place_module.execute_place_after_grasp(",
            backend,
        )

    def test_real_arm_initializes_transit_pose_before_accepting_tasks(self) -> None:
        node = (
            REPO_ROOT
            / "src"
            / "competition_mission"
            / "competition_mission"
            / "piper_arm_task_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn("self._startup_ready = False", node)
        self.assertIn("self._backend.initialize_transit_pose()", node)
        self.assertIn("if not self._startup_ready", node)

    def test_integrated_trajectory_matches_current_sources(self) -> None:
        trajectory_path = (
            REPO_ROOT
            / "docs"
            / "evidence"
            / "day5"
            / "indoor_competition_mission_trajectory.yaml"
        )
        artifact = yaml.safe_load(trajectory_path.read_text(encoding="utf-8"))
        source_paths = resolve_trajectory_source_paths(
            route_file=str(
                REPO_ROOT
                / "config"
                / "routes"
                / "indoor_competition_mission_route.yaml"
            ),
            semantic_map_file=str(
                REPO_ROOT / "maps" / "debug" / "semantic_map.yaml"
            ),
            planning_params_file=str(
                REPO_ROOT / "config" / "planning" / "planning_params.yaml"
            ),
            optimizer_params_file=str(
                REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
            ),
        )

        validate_source_manifest(artifact, source_paths)
        self.assertTrue(artifact["ok"])


if __name__ == "__main__":
    unittest.main()
