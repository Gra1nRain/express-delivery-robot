"""Pure image composition for the real-arm recognition monitor."""

from __future__ import annotations

from typing import Any

import numpy as np

try:
    import cv2
except ImportError:  # Pure tests may run without the ROS/OpenCV runtime.
    cv2 = None


_BANNER_HEIGHT_PX = 96
_INSTRUCTION_PHASES = {
    "RECOGNIZING_INSTRUCTION",
    "TARGET_TYPE_LOCKED",
}
_OBJECT_PHASES = {
    "SEARCHING_TARGET_OBJECT",
    "OPERATING",
    "VERIFYING_OPERATION",
}


def select_arm_recognition_source(
    live_frame: Any,
    instruction_overlay: Any,
    object_overlay: Any,
    *,
    phase: str,
) -> tuple[Any, str]:
    """Select the most relevant already-computed frame for the task phase."""
    phase_name = str(phase).strip().upper()
    if phase_name in _INSTRUCTION_PHASES and instruction_overlay is not None:
        return instruction_overlay, "INSTRUCTION_OVERLAY"
    if phase_name in _OBJECT_PHASES:
        if object_overlay is not None:
            return object_overlay, "OBJECT_OVERLAY"
        if instruction_overlay is not None:
            return instruction_overlay, "INSTRUCTION_OVERLAY"
    return live_frame, "LIVE_CAMERA"


def compose_arm_recognition_frame(
    source_frame: Any,
    *,
    task_type: str,
    phase: str,
    target_type: str,
    attempt: int,
    source: str,
    status: str,
) -> np.ndarray:
    """Add a flag/light-style status banner without mutating the source."""
    frame = np.asarray(source_frame)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("arm recognition frame must be a BGR image")
    if frame.dtype != np.uint8:
        frame = np.clip(frame, 0, 255).astype(np.uint8)
    else:
        frame = frame.copy()

    banner = np.zeros((_BANNER_HEIGHT_PX, frame.shape[1], 3), dtype=np.uint8)
    lines = (
        f"ARM: {str(task_type).strip() or 'IDLE'}  STATUS: {str(status).strip() or 'IDLE'}",
        f"PHASE: {str(phase).strip() or 'IDLE'}",
        (
            f"TARGET: {str(target_type).strip() or '-'}  "
            f"ATTEMPT: {max(0, int(attempt))}  VIEW: {str(source).strip()}"
        ),
    )
    colors = ((255, 255, 255), (0, 255, 255), (0, 255, 0))
    if cv2 is None:
        for index, color in enumerate(colors):
            banner[10 + index * 29 : 14 + index * 29, 5:-5] = color
    else:
        for index, (line, color) in enumerate(zip(lines, colors)):
            cv2.putText(
                banner,
                line,
                (10, 25 + index * 29),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                color,
                2,
                cv2.LINE_AA,
            )
    return np.vstack((banner, frame))
