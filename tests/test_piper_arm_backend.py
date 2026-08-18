import os
import pathlib
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_mission"))

from competition_mission.arm_task_runner import ArmTaskPhase
from competition_mission.arm_task_runner import (
    ArmExecutionFailure,
    ArmTaskOutcome,
)
from competition_mission.piper_arm_backend import (
    COMPETITION_TRANSIT_JOINTS_RAD,
    PiperMigrationBackend,
    apply_migration_shell_defaults,
    deduplicate_yolo_candidates,
    deduplicate_yolo_detection,
    install_competition_yolo_postprocessing,
    prepare_competition_environment,
)


class FakeController:
    def __init__(self):
        self.state = SimpleNamespace(name="IDLE")
        self.arm_enabled = True
        self.arm_faulted = False
        self.color_image = object()
        self.depth_image = object()
        self.camera_matrix = object()
        self.instruction_detector = SimpleNamespace(is_loaded=True)
        self.vision_detector = SimpleNamespace(is_loaded=True)
        self.target_class_id = None
        self.target_model_class_name = None
        self.competition_grasp_failed = False
        self.grasp_running = False
        self.last_gripper_position_m = 0.020
        self.last_gripper_effort_nm = 0.4
        self.last_gripper_feedback_at = time.monotonic()
        self.observation_moves = 0
        self.drop_pose_moves = 0
        self.joint_pose_calls = []
        self.failed_move_labels = set()
        self.worker_calls = 0
        self.can_control_ready = True
        self.can_control_wait_calls = 0
        self.startup_events = []
        self.gripper_hold_active = False
        self.gripper_hold_m = None
        self.gripper_hold_target = ""
        self.worker_hold_m = None

    def wait_for_stable_can_control(self):
        self.can_control_wait_calls += 1
        self.startup_events.append("can_control_stable")
        return self.can_control_ready

    def activate_gripper_hold(self, gripper_m, target_type):
        self.gripper_hold_active = True
        self.gripper_hold_m = float(gripper_m)
        self.gripper_hold_target = str(target_type)

    def is_gripper_hold_active(self):
        return self.gripper_hold_active

    def get_gripper_hold_position(self):
        return self.gripper_hold_m if self.gripper_hold_active else None

    def authorize_gripper_release(self, _reason):
        if not self.gripper_hold_active:
            raise RuntimeError("no active hold")

    def complete_gripper_release(self):
        self.gripper_hold_active = False
        self.gripper_hold_m = None
        self.gripper_hold_target = ""

    def move_to_observation_joint_pose(self):
        self.observation_moves += 1
        return True

    def select_target_from_instruction_sheet(self):
        self.target_class_id = 0
        self.target_model_class_name = "green_bottle"
        return {"class_name": "green_bottle"}

    def set_target_class(
        self,
        class_id,
        color=None,
        model_class_name=None,
        source=None,
    ):
        self.target_class_id = class_id
        self.target_model_class_name = model_class_name

    def estimate_wrist_object_with_observation_scan(self):
        return {"object": "detected"}

    def _execute_wrist_grasp_worker(self, skip_instruction_confirmation):
        self.worker_calls += 1
        if self.worker_hold_m is not None:
            self.activate_gripper_hold(
                self.worker_hold_m,
                self.target_model_class_name,
            )
        self.last_gripper_position_m = 0.020
        self.last_gripper_feedback_at = time.monotonic()

    def get_grasp_config(self, _class_id):
        return {"gripper_closed": 0.0}

    def move_to_joint_pose(
        self,
        joints,
        *,
        label,
        gripper_m,
        timeout_s,
    ):
        self.startup_events.append("joint_target")
        self.drop_pose_moves += 1
        self.joint_pose_calls.append(
            {
                "joints": tuple(joints),
                "label": label,
                "gripper_m": gripper_m,
                "timeout_s": timeout_s,
            }
        )
        return label not in self.failed_move_labels


class FakePlaceModule:
    def __init__(self):
        self.PLACE_MOVE_OBSERVE_BEFORE_DETECT = True
        self.execute_calls = 0
        self.selection_calls = 0
        self.hold_seen_before_release = False

    def _select_target_sheet_candidate_with_scan(self, *_args, **_kwargs):
        self.selection_calls += 1
        return ("candidate", "bgr", "depth", "matrix", "result", "view", 0.0)

    def execute_place_after_grasp(self, *_args, **_kwargs):
        self.execute_calls += 1
        self._select_target_sheet_candidate_with_scan()
        controller = _args[0]
        self.hold_seen_before_release = controller.is_gripper_hold_active()
        controller.authorize_gripper_release("test placement release")
        controller.complete_gripper_release()
        controller.last_gripper_position_m = 0.070
        controller.last_gripper_feedback_at = time.monotonic()
        return True


class PiperArmBackendTest(unittest.TestCase):
    def setUp(self):
        self.controller = FakeController()
        self.place = FakePlaceModule()
        self.grasp = SimpleNamespace(
            parse_target_spec=lambda target: {
                "class_id": 0,
                "color": "green",
                "model_class_name": target,
            },
            PLACE_OBSERVATION_JOINTS_RAD=(1, 2, 3, 4, 5, 6),
            R_CAM_TO_GRIPPER="rotation",
            T_CAM_TO_GRIPPER="translation",
            solve_piper_ik_pose=lambda *_args, **_kwargs: "ik",
        )
        self.backend = PiperMigrationBackend(
            self.controller,
            self.grasp,
            self.place,
            readiness_timeout_s=0.1,
            feedback_timeout_s=0.1,
        )

    def test_pickup_does_not_call_legacy_place(self):
        phases = []
        target = self.backend.pickup_once(
            "",
            lambda phase, value: phases.append((phase, value)),
        )

        self.assertEqual(target, "green_bottle")
        self.assertEqual(self.controller.worker_calls, 1)
        self.assertEqual(self.place.execute_calls, 0)
        self.assertIn(
            (ArmTaskPhase.RECOGNIZING_INSTRUCTION, ""),
            phases,
        )

    def test_pickup_returns_to_transit_pose_without_opening_gripper(self):
        target = self.backend.pickup_once(
            "green_bottle",
            lambda *_args: None,
        )

        self.assertEqual(target, "green_bottle")
        self.assertEqual(len(self.controller.joint_pose_calls), 1)
        transit_move = self.controller.joint_pose_calls[0]
        self.assertEqual(
            transit_move["joints"],
            COMPETITION_TRANSIT_JOINTS_RAD,
        )
        self.assertEqual(transit_move["gripper_m"], 0.0)
        self.assertTrue(self.controller.gripper_hold_active)
        self.assertEqual(self.controller.gripper_hold_m, 0.0)
        self.assertEqual(self.controller.gripper_hold_target, "green_bottle")

    def test_pickup_preserves_contact_based_hold_for_transit(self):
        self.controller.worker_hold_m = 0.018

        self.backend.pickup_once(
            "green_bottle",
            lambda *_args: None,
        )

        transit_move = self.controller.joint_pose_calls[0]
        self.assertEqual(transit_move["gripper_m"], 0.018)
        self.assertEqual(self.controller.gripper_hold_m, 0.018)

    def test_second_pickup_is_rejected_until_held_object_is_dropped(self):
        self.backend.pickup_once("green_bottle", lambda *_args: None)
        prior_observation_moves = self.controller.observation_moves

        with self.assertRaises(ArmExecutionFailure) as context:
            self.backend.pickup_once("yellow_block", lambda *_args: None)

        self.assertIn("gripper_already_holding_object", context.exception.detail)
        self.assertEqual(
            self.controller.observation_moves,
            prior_observation_moves,
        )

    def test_pickup_does_not_return_to_transit_between_detection_and_grasp(self):
        events = []

        def detect_object():
            events.append("object_detected")
            return {"object": "detected"}

        def execute_grasp(*, skip_instruction_confirmation):
            self.assertTrue(skip_instruction_confirmation)
            events.append("grasp")
            self.controller.last_gripper_position_m = 0.020
            self.controller.last_gripper_feedback_at = time.monotonic()

        original_move = self.controller.move_to_joint_pose

        def record_joint_move(*args, **kwargs):
            events.append("transit")
            return original_move(*args, **kwargs)

        self.controller.estimate_wrist_object_with_observation_scan = detect_object
        self.controller._execute_wrist_grasp_worker = execute_grasp
        self.controller.move_to_joint_pose = record_joint_move

        self.backend.pickup_once("green_bottle", lambda *_args: None)

        self.assertEqual(events, ["object_detected", "grasp", "transit"])

    def test_pickup_preserves_detection_overlay_for_visualization(self):
        overlay = [["detected-object"]]
        self.controller.estimate_wrist_object_with_observation_scan = (
            lambda: {"object": "detected", "overlay": overlay}
        )

        self.backend.pickup_once("green_bottle", lambda *_args: None)

        self.assertEqual(self.controller.last_wrist_preview, overlay)
        self.assertIsNot(self.controller.last_wrist_preview, overlay)

    def test_startup_moves_to_transit_pose_with_current_gripper(self):
        self.backend.initialize_transit_pose()

        self.assertEqual(self.controller.can_control_wait_calls, 1)
        self.assertEqual(
            self.controller.startup_events,
            ["can_control_stable", "joint_target"],
        )
        self.assertEqual(len(self.controller.joint_pose_calls), 1)
        transit_move = self.controller.joint_pose_calls[0]
        self.assertEqual(
            transit_move["joints"],
            COMPETITION_TRANSIT_JOINTS_RAD,
        )
        self.assertEqual(transit_move["gripper_m"], 0.020)

    def test_startup_does_not_send_transit_target_without_stable_can_control(self):
        self.controller.can_control_ready = False

        with self.assertRaises(ArmExecutionFailure) as context:
            self.backend.initialize_transit_pose()

        self.assertEqual(self.controller.can_control_wait_calls, 1)
        self.assertEqual(self.controller.joint_pose_calls, [])
        self.assertEqual(
            context.exception.outcome,
            ArmTaskOutcome.OPERATION_FAILED,
        )
        self.assertIn(
            "failed_to_enter_stable_can_control",
            context.exception.detail,
        )

    def test_pickup_can_pause_after_instruction_for_manual_image_removal(self):
        delays = []
        backend = PiperMigrationBackend(
            self.controller,
            self.grasp,
            self.place,
            readiness_timeout_s=0.1,
            feedback_timeout_s=0.1,
            post_instruction_clear_delay_s=10.0,
            sleep=delays.append,
        )

        backend.pickup_once("", lambda *_args: None)
        self.assertEqual(delays, [10.0])

        self.controller.complete_gripper_release()
        delays.clear()
        backend.pickup_once("green_bottle", lambda *_args: None)
        self.assertEqual(delays, [])

    def test_pickup_retries_center_instruction_recognition_without_repositioning(
        self,
    ):
        selection_calls = 0

        def select_instruction():
            nonlocal selection_calls
            selection_calls += 1
            if selection_calls == 1:
                raise RuntimeError("instruction temporarily not visible")
            self.controller.target_class_id = 0
            self.controller.target_model_class_name = "green_bottle"
            return {"class_name": "green_bottle"}

        self.controller.select_target_from_instruction_sheet = select_instruction

        target = self.backend.pickup_once("", lambda *_args: None)

        self.assertEqual(target, "green_bottle")
        self.assertEqual(selection_calls, 2)
        self.assertEqual(self.controller.observation_moves, 1)
        self.assertEqual(len(self.controller.joint_pose_calls), 1)
        self.assertEqual(
            self.controller.joint_pose_calls[0]["joints"],
            COMPETITION_TRANSIT_JOINTS_RAD,
        )

    def test_drop_uses_locked_target_and_independent_place_function(self):
        phases = []
        self.backend.drop_once(
            "green_bottle",
            lambda phase, value: phases.append((phase, value)),
        )

        self.assertEqual(self.place.execute_calls, 1)
        self.assertTrue(self.place.hold_seen_before_release)
        self.assertFalse(self.controller.gripper_hold_active)
        self.assertEqual(self.controller.drop_pose_moves, 2)
        transit_move = self.controller.joint_pose_calls[-1]
        self.assertEqual(
            transit_move["joints"],
            COMPETITION_TRANSIT_JOINTS_RAD,
        )
        self.assertEqual(transit_move["gripper_m"], 0.070)
        self.assertIn(
            (ArmTaskPhase.OPERATING, "green_bottle"),
            phases,
        )

    def test_task_does_not_report_success_when_transit_move_fails(self):
        self.controller.failed_move_labels.add("Competition transit pose")

        with self.assertRaises(ArmExecutionFailure) as context:
            self.backend.pickup_once(
                "green_bottle",
                lambda *_args: None,
            )

        self.assertEqual(
            context.exception.outcome,
            ArmTaskOutcome.OPERATION_FAILED,
        )
        self.assertIn(
            "failed_to_reach_transit_pose",
            context.exception.detail,
        )

    def test_pickup_requires_nonzero_gripper_opening_confirmation(self):
        def empty_grasp(*, skip_instruction_confirmation):
            self.assertTrue(skip_instruction_confirmation)
            self.controller.last_gripper_position_m = 0.0
            self.controller.last_gripper_feedback_at = time.monotonic()

        self.controller._execute_wrist_grasp_worker = empty_grasp
        self.controller.failed_move_labels.add("Competition transit pose")
        with self.assertRaises(ArmExecutionFailure) as context:
            self.backend.pickup_once("green_bottle", lambda *_args: None)

        self.assertEqual(
            context.exception.outcome,
            ArmTaskOutcome.OPERATION_FAILED,
        )
        self.assertIn("pickup_not_verified", context.exception.detail)
        self.assertEqual(len(self.controller.joint_pose_calls), 1)
        self.assertEqual(
            self.controller.joint_pose_calls[0]["joints"],
            COMPETITION_TRANSIT_JOINTS_RAD,
        )

    def test_shell_defaults_do_not_override_existing_environment(self):
        variable = "CODEX_TEST_PIPER_DEFAULT"
        previous = os.environ.get(variable)
        try:
            os.environ[variable] = "existing"
            with tempfile.TemporaryDirectory() as directory:
                script = pathlib.Path(directory) / "run.sh"
                script.write_text(
                    'export CODEX_TEST_PIPER_DEFAULT="${CODEX_TEST_PIPER_DEFAULT:-new}"\n'
                    'export CODEX_TEST_PIPER_OTHER="${CODEX_TEST_PIPER_OTHER:-42}"\n',
                    encoding="utf-8",
                )
                os.environ.pop("CODEX_TEST_PIPER_OTHER", None)
                applied = apply_migration_shell_defaults(script)

            self.assertEqual(applied, 1)
            self.assertEqual(os.environ[variable], "existing")
            self.assertEqual(os.environ["CODEX_TEST_PIPER_OTHER"], "42")
        finally:
            os.environ.pop("CODEX_TEST_PIPER_OTHER", None)
            if previous is None:
                os.environ.pop(variable, None)
            else:
                os.environ[variable] = previous

    def test_migration_defaults_yolo_to_gpu(self):
        previous_environment = os.environ.copy()
        try:
            os.environ.pop("WRIST_YOLO_DEVICE", None)
            apply_migration_shell_defaults(
                REPO_ROOT
                / "Piper_Grasp_Humble_Migration_20260723"
                / "run_grasp_single.sh"
            )
            self.assertEqual(os.environ["WRIST_YOLO_DEVICE"], "0")
        finally:
            os.environ.clear()
            os.environ.update(previous_environment)

    def test_competition_environment_selects_new_weight_for_both_stages(self):
        object_variable = "WRIST_YOLO_MODEL_PATH"
        instruction_variable = "WRIST_INSTRUCTION_YOLO_MODEL_PATH"
        previous_object = os.environ.get(object_variable)
        previous_instruction = os.environ.get(instruction_variable)
        try:
            with tempfile.TemporaryDirectory() as directory:
                migration_root = pathlib.Path(directory)
                (migration_root / "run_grasp_single.sh").write_text(
                    "#!/usr/bin/env bash\n",
                    encoding="utf-8",
                )
                expected_model = migration_root / "best.pt"
                expected_model.write_bytes(b"new-object-weight")
                os.environ[object_variable] = "old-object-weight.pt"
                os.environ[instruction_variable] = "instruction-weight.pt"

                prepare_competition_environment(migration_root)

                self.assertEqual(
                    pathlib.Path(os.environ[object_variable]),
                    expected_model.resolve(),
                )
                self.assertEqual(
                    pathlib.Path(os.environ[instruction_variable]),
                    expected_model.resolve(),
                )
        finally:
            if previous_object is None:
                os.environ.pop(object_variable, None)
            else:
                os.environ[object_variable] = previous_object
            if previous_instruction is None:
                os.environ.pop(instruction_variable, None)
            else:
                os.environ[instruction_variable] = previous_instruction

    def test_competition_environment_extends_block_top_down_reach(self):
        variable = "WRIST_BLOCK_TOP_DOWN_MIN_FLANGE_Y_M"
        previous = os.environ.get(variable)
        try:
            os.environ[variable] = "-0.380"
            with patch(
                "competition_mission.piper_arm_backend."
                "apply_migration_shell_defaults",
                return_value=0,
            ):
                prepare_competition_environment(REPO_ROOT)

            self.assertEqual(os.environ[variable], "-0.545")
        finally:
            if previous is None:
                os.environ.pop(variable, None)
            else:
                os.environ[variable] = previous

    def test_competition_environment_restores_block_confidence_to_half(self):
        variable = "WRIST_BLOCK_COMPLETE_DETECTION_CONFIDENCE"
        previous = os.environ.get(variable)
        try:
            os.environ[variable] = "0.30"
            with patch(
                "competition_mission.piper_arm_backend."
                "apply_migration_shell_defaults",
                return_value=0,
            ):
                prepare_competition_environment(REPO_ROOT)

            self.assertEqual(os.environ[variable], "0.50")
        finally:
            if previous is None:
                os.environ.pop(variable, None)
            else:
                os.environ[variable] = previous

    def test_duplicate_yolo_candidates_keep_highest_confidence(self):
        candidates = [
            {
                "class_name": "green_bottle",
                "confidence": 0.62,
                "bbox": [10.0, 10.0, 110.0, 110.0],
            },
            {
                "class_name": "paper_green_bottle",
                "confidence": 0.91,
                "bbox": [12.0, 12.0, 108.0, 108.0],
            },
            {
                "class_name": "red_block",
                "confidence": 0.70,
                "bbox": [200.0, 20.0, 260.0, 80.0],
            },
        ]

        kept = deduplicate_yolo_candidates(candidates, iou_threshold=0.5)

        self.assertEqual(len(kept), 2)
        self.assertEqual(kept[0]["confidence"], 0.91)
        self.assertEqual(kept[1]["class_name"], "red_block")

    def test_duplicate_object_detection_keeps_fields_aligned(self):
        detection = {
            "boxes": [
                [10.0, 10.0, 110.0, 110.0],
                [12.0, 12.0, 108.0, 108.0],
                [200.0, 20.0, 260.0, 80.0],
            ],
            "labels": ["green_bottle", "paper_green_bottle", "red_block"],
            "confidences": [0.62, 0.91, 0.70],
            "class_ids": [0, 6, 3],
            "class_names": ["green_bottle", "paper_green_bottle", "red_block"],
            "text_prompt": "*",
        }

        kept = deduplicate_yolo_detection(detection, iou_threshold=0.5)

        self.assertEqual(kept["confidences"], [0.91, 0.70])
        self.assertEqual(kept["class_ids"], [6, 3])
        self.assertEqual(
            kept["labels"],
            ["paper_green_bottle", "red_block"],
        )

    def test_yolo_adapter_maps_paper_classes_and_configures_nms(self):
        class InstructionDetector:
            def __init__(self):
                self.model = SimpleNamespace(overrides={})

            def detect(self):
                return {"candidates": []}

        class ObjectDetector:
            def __init__(self):
                self.model = SimpleNamespace(overrides={})

            def detect_frame_simple(self):
                return {"boxes": [], "confidences": []}

        module = SimpleNamespace(
            CUSTOM_YOLO_MODEL_CLASS_ALIASES={},
            InstructionSheetDetector=InstructionDetector,
            SimpleVisionDetector=ObjectDetector,
        )

        install_competition_yolo_postprocessing(module)

        self.assertEqual(
            module.CUSTOM_YOLO_MODEL_CLASS_ALIASES["纸_绿瓶子"],
            "green_bottle",
        )
        self.assertEqual(
            module.CUSTOM_YOLO_MODEL_CLASS_ALIASES["paper_red_cube"],
            "red_block",
        )
        instruction_detector = InstructionDetector()
        instruction_detector.detect()
        self.assertTrue(instruction_detector.model.overrides["agnostic_nms"])
        self.assertEqual(instruction_detector.model.overrides["iou"], 0.6)

    def test_competition_environment_uses_single_detection_per_scan_view(self):
        expected = {
            "WRIST_INSTRUCTION_CONFIRM_FRAMES": "1",
            "WRIST_OBSERVATION_SCAN_DETECTION_ATTEMPTS": "1",
            "WRIST_PLACE_SCAN_ENABLED": "1",
            "WRIST_PLACE_SCAN_OFFSETS_DEG": "10,-10",
            "WRIST_PLACE_DETECT_REQUIRED_FRAMES": "1",
        }
        previous = {name: os.environ.get(name) for name in expected}
        try:
            with tempfile.TemporaryDirectory() as directory:
                migration_root = pathlib.Path(directory)
                (migration_root / "run_grasp_single.sh").write_text(
                    "#!/usr/bin/env bash\n",
                    encoding="utf-8",
                )

                prepare_competition_environment(migration_root)

            for name, value in expected.items():
                self.assertEqual(os.environ[name], value)
        finally:
            for name, value in previous.items():
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value

    def test_drop_search_failure_keeps_carried_object_clamped(self):
        self.controller.last_gripper_position_m = 0.0

        def fail_selection(*_args, **_kwargs):
            raise RuntimeError("placement marker not visible")

        self.place._select_target_sheet_candidate_with_scan = fail_selection

        with self.assertRaises(ArmExecutionFailure):
            self.backend.drop_once("green_bottle", lambda *_args: None)

        self.assertEqual(len(self.controller.joint_pose_calls), 2)
        self.assertTrue(
            all(
                move["gripper_m"] == 0.0
                for move in self.controller.joint_pose_calls
            )
        )


if __name__ == "__main__":
    unittest.main()
