import pathlib
import sys
import unittest

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_mission"))

from competition_mission.arm_recognition_visualizer import (
    compose_arm_recognition_frame,
    describe_arm_recognition_stage,
    select_arm_recognition_source,
)


class ArmRecognitionVisualizerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = np.full((48, 64, 3), 10, dtype=np.uint8)
        self.instruction = np.full((48, 64, 3), 20, dtype=np.uint8)
        self.object = np.full((48, 64, 3), 30, dtype=np.uint8)

    def test_instruction_phase_prefers_instruction_overlay(self) -> None:
        selected, source = select_arm_recognition_source(
            self.raw,
            self.instruction,
            self.object,
            phase="RECOGNIZING_INSTRUCTION",
        )

        self.assertIs(selected, self.instruction)
        self.assertEqual(source, "INSTRUCTION_OVERLAY")

    def test_object_phase_prefers_object_overlay(self) -> None:
        selected, source = select_arm_recognition_source(
            self.raw,
            self.instruction,
            self.object,
            phase="SEARCHING_TARGET_OBJECT",
        )

        self.assertIs(selected, self.object)
        self.assertEqual(source, "OBJECT_OVERLAY")

    def test_object_search_without_result_keeps_live_camera_refreshing(self) -> None:
        selected, source = select_arm_recognition_source(
            self.raw,
            self.instruction,
            None,
            phase="SEARCHING_TARGET_OBJECT",
        )

        self.assertIs(selected, self.raw)
        self.assertEqual(source, "LIVE_CAMERA")

    def test_missing_overlay_falls_back_to_live_camera(self) -> None:
        selected, source = select_arm_recognition_source(
            self.raw,
            None,
            None,
            phase="RECOGNIZING_INSTRUCTION",
        )

        self.assertIs(selected, self.raw)
        self.assertEqual(source, "LIVE_CAMERA")

    def test_semantic_stage_distinguishes_image_and_object_recognition(self) -> None:
        self.assertEqual(
            describe_arm_recognition_stage(
                "RECOGNIZING_INSTRUCTION", "PICKUP"
            ),
            "INSTRUCTION IMAGE RECOGNITION",
        )
        self.assertEqual(
            describe_arm_recognition_stage(
                "SEARCHING_TARGET_OBJECT", "PICKUP"
            ),
            "TARGET OBJECT RECOGNITION",
        )

    def test_semantic_stage_names_pickup_and_drop_execution(self) -> None:
        self.assertEqual(
            describe_arm_recognition_stage("OPERATING", "PICKUP"),
            "PICKUP IN PROGRESS",
        )
        self.assertEqual(
            describe_arm_recognition_stage("OPERATING", "DROP"),
            "DROP IN PROGRESS",
        )

    def test_composition_adds_status_banner_without_mutating_source(self) -> None:
        before = self.object.copy()

        result = compose_arm_recognition_frame(
            self.object,
            task_type="PICKUP",
            phase="SEARCHING_TARGET_OBJECT",
            target_type="green_bottle",
            attempt=2,
            source="OBJECT_OVERLAY",
            status="RUNNING",
        )

        self.assertEqual(result.shape, (172, 64, 3))
        np.testing.assert_array_equal(self.object, before)
        self.assertGreater(int(result[:124].max()), 0)
        np.testing.assert_array_equal(result[124:], self.object)


if __name__ == "__main__":
    unittest.main()
