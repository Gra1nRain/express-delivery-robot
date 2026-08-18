import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
MIGRATION_ROOT = REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723"
sys.path.insert(0, str(MIGRATION_ROOT))

from target_detection_gate import (  # noqa: E402
    bbox_iou,
    evaluate_bbox_visibility,
    localization_detection_policy,
)


class TargetDetectionGateTest(unittest.TestCase):
    def test_rejects_real_partial_block_bbox_touching_left_edge(self):
        result = evaluate_bbox_visibility(
            [0.1, 162.4, 85.9, 304.1],
            image_width=640,
            image_height=480,
            edge_margin_px=12,
        )

        self.assertFalse(result["complete"])
        self.assertIn("left", result["clipped_edges"])

    def test_accepts_complete_bbox_clear_of_all_edges(self):
        result = evaluate_bbox_visibility(
            [180.0, 120.0, 300.0, 280.0],
            image_width=640,
            image_height=480,
            edge_margin_px=12,
        )

        self.assertTrue(result["complete"])
        self.assertEqual(result["clipped_edges"], [])

    def test_rejects_invalid_or_too_small_bbox(self):
        invalid = evaluate_bbox_visibility(
            [100.0, 100.0, 90.0, 120.0],
            image_width=640,
            image_height=480,
            edge_margin_px=12,
        )
        tiny = evaluate_bbox_visibility(
            [100.0, 100.0, 106.0, 107.0],
            image_width=640,
            image_height=480,
            edge_margin_px=12,
            min_size_px=12,
        )

        self.assertFalse(invalid["complete"])
        self.assertEqual(invalid["reason"], "invalid_bbox")
        self.assertFalse(tiny["complete"])
        self.assertEqual(tiny["reason"], "bbox_too_small")

    def test_iou_distinguishes_stable_and_shifted_confirmation_frames(self):
        first = [180.0, 120.0, 300.0, 280.0]
        stable = [184.0, 123.0, 304.0, 283.0]
        shifted = [360.0, 120.0, 480.0, 280.0]

        self.assertGreater(bbox_iou(first, stable), 0.80)
        self.assertEqual(bbox_iou(first, shifted), 0.0)

    def test_block_policy_lowers_threshold_only_with_stricter_gate(self):
        block = localization_detection_policy(
            is_block_target=True,
            yolo_confidence=0.10,
            regular_confidence=0.50,
            complete_block_confidence=0.30,
            block_confirm_frames=2,
        )
        bottle = localization_detection_policy(
            is_block_target=False,
            yolo_confidence=0.10,
            regular_confidence=0.50,
            complete_block_confidence=0.30,
            block_confirm_frames=2,
        )

        self.assertEqual(block["confidence"], 0.30)
        self.assertTrue(block["require_complete_bbox"])
        self.assertEqual(block["confirm_frames"], 2)
        self.assertEqual(bottle["confidence"], 0.50)
        self.assertFalse(bottle["require_complete_bbox"])
        self.assertEqual(bottle["confirm_frames"], 1)


if __name__ == "__main__":
    unittest.main()
