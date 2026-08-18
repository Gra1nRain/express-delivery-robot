import pathlib
import sys
import unittest

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
MIGRATION_ROOT = REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723"
sys.path.insert(0, str(MIGRATION_ROOT))

from yolo_runtime import warm_up_yolo_model


class _RecordingModel:
    def __init__(self):
        self.calls = []

    def predict(self, frame, **kwargs):
        self.calls.append((frame.copy(), kwargs))
        return []


class YoloRuntimeTest(unittest.TestCase):
    def test_warmup_runs_one_full_size_inference_on_selected_device(self):
        model = _RecordingModel()

        elapsed_s = warm_up_yolo_model(
            model,
            image_size=640,
            confidence=0.1,
            device="0",
        )

        self.assertGreaterEqual(elapsed_s, 0.0)
        self.assertEqual(len(model.calls), 1)
        frame, kwargs = model.calls[0]
        self.assertEqual(frame.shape, (640, 640, 3))
        self.assertEqual(frame.dtype, np.uint8)
        self.assertEqual(kwargs["imgsz"], 640)
        self.assertEqual(kwargs["conf"], 0.1)
        self.assertEqual(kwargs["device"], "0")
        self.assertFalse(kwargs["verbose"])

    def test_warmup_rejects_invalid_image_size_before_inference(self):
        model = _RecordingModel()

        with self.assertRaisesRegex(ValueError, "image_size"):
            warm_up_yolo_model(
                model,
                image_size=0,
                confidence=0.1,
                device="cpu",
            )

        self.assertEqual(model.calls, [])


if __name__ == "__main__":
    unittest.main()
