"""Pure image composition for the real-arm recognition monitor."""

from __future__ import annotations

from typing import Any

import numpy as np

try:
    import cv2
except ImportError:  # Pure tests may run without the ROS/OpenCV runtime.
    cv2 = None


_BANNER_HEIGHT_PX = 124
_INSTRUCTION_PHASES = {
    "RECOGNIZING_INSTRUCTION",
    "TARGET_TYPE_LOCKED",
}
_OBJECT_OVERLAY_PHASES = {
    "SEARCHING_TARGET_OBJECT",
}


def describe_arm_recognition_stage(phase: str, task_type: str) -> str:
    """Return an operator-facing stage instead of a raw state-machine name."""
    phase_name = str(phase).strip().upper()
    task_name = str(task_type).strip().upper()
    if phase_name == "STARTUP_TRANSIT":
        return "MOVING TO STANDBY POSE"
    if phase_name == "IDLE":
        return "READY"
    if phase_name == "MOVING_TO_INSTRUCTION_POSE":
        return "MOVING TO IMAGE POSE"
    if phase_name == "RECOGNIZING_INSTRUCTION":
        return "INSTRUCTION IMAGE RECOGNITION"
    if phase_name == "TARGET_TYPE_LOCKED":
        return "INSTRUCTION TARGET LOCKED"
    if phase_name == "SEARCHING_TARGET_OBJECT":
        return "TARGET OBJECT RECOGNITION"
    if phase_name == "OPERATING":
        return f"{task_name or 'ARM'} IN PROGRESS"
    if phase_name == "VERIFYING_OPERATION":
        return f"VERIFYING {task_name or 'ARM'} RESULT"
    return phase_name or "IDLE"


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
    if phase_name in _OBJECT_OVERLAY_PHASES:
        if object_overlay is not None:
            return object_overlay, "OBJECT_OVERLAY"
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
    stage = describe_arm_recognition_stage(phase, task_type)
    lines = (
        f"ARM: {str(task_type).strip() or 'IDLE'}  STATUS: {str(status).strip() or 'IDLE'}",
        f"STAGE: {stage}",
        f"PHASE: {str(phase).strip() or 'IDLE'}",
        (
            f"TARGET: {str(target_type).strip() or '-'}  "
            f"ATTEMPT: {max(0, int(attempt))}  VIEW: {str(source).strip()}"
        ),
    )
    stage_color = (
        (0, 255, 255)
        if "IMAGE" in stage
        else (0, 165, 255)
        if "OBJECT" in stage
        else (0, 255, 0)
    )
    colors = (
        (255, 255, 255),
        stage_color,
        (200, 200, 200),
        (0, 255, 0),
    )
    if cv2 is None:
        for index, color in enumerate(colors):
            banner[8 + index * 28 : 12 + index * 28, 5:-5] = color
    else:
        for index, (line, color) in enumerate(zip(lines, colors)):
            cv2.putText(
                banner,
                line,
                (10, 24 + index * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65 if index == 1 else 0.52,
                color,
                2,
                cv2.LINE_AA,
            )
    return np.vstack((banner, frame))
