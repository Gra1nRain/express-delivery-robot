import os
import pathlib
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_mission"))

from competition_mission.arm_task_runner import ArmTaskPhase
from competition_mission.arm_task_runner import (
    ArmExecutionFailure,
    ArmTaskOutcome,
)
from competition_mission.piper_arm_backend import (
    PiperMigrationBackend,
    apply_migration_shell_defaults,
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
        self.worker_calls = 0

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
        self.last_gripper_position_m = 0.020
        self.last_gripper_feedback_at = time.monotonic()

    def get_grasp_config(self, _class_id):
        return {"gripper_closed": 0.0}

    def move_to_joint_pose(self, *_args, **_kwargs):
        self.drop_pose_moves += 1
        return True


class FakePlaceModule:
    def __init__(self):
        self.PLACE_MOVE_OBSERVE_BEFORE_DETECT = True
        self.execute_calls = 0
        self.selection_calls = 0

    def _select_target_sheet_candidate_with_scan(self, *_args, **_kwargs):
        self.selection_calls += 1
        return ("candidate", "bgr", "depth", "matrix", "result", "view", 0.0)

    def execute_place_after_grasp(self, *_args, **_kwargs):
        self.execute_calls += 1
        self._select_target_sheet_candidate_with_scan()
        controller = _args[0]
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

    def test_drop_uses_locked_target_and_independent_place_function(self):
        phases = []
        self.backend.drop_once(
            "green_bottle",
            lambda phase, value: phases.append((phase, value)),
        )

        self.assertEqual(self.place.execute_calls, 1)
        self.assertEqual(self.controller.drop_pose_moves, 1)
        self.assertIn(
            (ArmTaskPhase.OPERATING, "green_bottle"),
            phases,
        )

    def test_pickup_requires_nonzero_gripper_opening_confirmation(self):
        def empty_grasp(*, skip_instruction_confirmation):
            self.assertTrue(skip_instruction_confirmation)
            self.controller.last_gripper_position_m = 0.0
            self.controller.last_gripper_feedback_at = time.monotonic()

        self.controller._execute_wrist_grasp_worker = empty_grasp
        with self.assertRaises(ArmExecutionFailure) as context:
            self.backend.pickup_once("green_bottle", lambda *_args: None)

        self.assertEqual(
            context.exception.outcome,
            ArmTaskOutcome.OPERATION_FAILED,
        )
        self.assertIn("pickup_not_verified", context.exception.detail)

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


if __name__ == "__main__":
    unittest.main()
