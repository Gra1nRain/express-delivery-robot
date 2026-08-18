#!/usr/bin/env python3
import time

import numpy as np


def warm_up_yolo_model(model, *, image_size, confidence, device):
    """Initialize the Ultralytics predictor and selected inference device."""
    size = int(image_size)
    if size <= 0:
        raise ValueError("image_size must be positive")

    dummy_frame = np.zeros((size, size, 3), dtype=np.uint8)
    started_at = time.perf_counter()
    model.predict(
        dummy_frame,
        imgsz=size,
        conf=float(confidence),
        device=device,
        verbose=False,
    )
    return time.perf_counter() - started_at
