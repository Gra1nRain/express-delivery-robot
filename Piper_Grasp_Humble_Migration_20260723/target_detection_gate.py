"""Pure helpers for deciding whether a detection is safe for localization."""

from __future__ import annotations

import math
from typing import Any, Sequence


def _normalized_bbox(bbox: Sequence[float]) -> tuple[float, float, float, float] | None:
    try:
        values = tuple(float(value) for value in bbox)
    except (TypeError, ValueError):
        return None
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        return None
    x0, y0, x1, y1 = values
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def evaluate_bbox_visibility(
    bbox: Sequence[float],
    *,
    image_width: int,
    image_height: int,
    edge_margin_px: int,
    min_size_px: int = 12,
) -> dict[str, Any]:
    """Reject boxes that may represent an object clipped by an image edge."""
    normalized = _normalized_bbox(bbox)
    if normalized is None or image_width <= 0 or image_height <= 0:
        return {
            "complete": False,
            "reason": "invalid_bbox",
            "clipped_edges": [],
        }

    x0, y0, x1, y1 = normalized
    if x1 - x0 < max(1, int(min_size_px)) or y1 - y0 < max(1, int(min_size_px)):
        return {
            "complete": False,
            "reason": "bbox_too_small",
            "clipped_edges": [],
        }

    margin = max(0.0, float(edge_margin_px))
    clipped_edges = []
    if x0 < margin:
        clipped_edges.append("left")
    if y0 < margin:
        clipped_edges.append("top")
    if x1 > float(image_width) - margin:
        clipped_edges.append("right")
    if y1 > float(image_height) - margin:
        clipped_edges.append("bottom")

    return {
        "complete": not clipped_edges,
        "reason": "complete" if not clipped_edges else "touches_image_edge",
        "clipped_edges": clipped_edges,
        "bbox": [x0, y0, x1, y1],
        "image_size": [int(image_width), int(image_height)],
        "edge_margin_px": int(max(0, edge_margin_px)),
    }


def bbox_iou(first: Sequence[float], second: Sequence[float]) -> float:
    """Return intersection-over-union for two valid xyxy boxes."""
    first_box = _normalized_bbox(first)
    second_box = _normalized_bbox(second)
    if first_box is None or second_box is None:
        return 0.0

    ax0, ay0, ax1, ay1 = first_box
    bx0, by0, bx1, by1 = second_box
    intersection_width = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    intersection_height = max(0.0, min(ay1, by1) - max(ay0, by0))
    intersection = intersection_width * intersection_height
    union = (
        (ax1 - ax0) * (ay1 - ay0)
        + (bx1 - bx0) * (by1 - by0)
        - intersection
    )
    return intersection / union if union > 0.0 else 0.0


def localization_detection_policy(
    *,
    is_block_target: bool,
    yolo_confidence: float,
    regular_confidence: float,
    complete_block_confidence: float,
    block_confirm_frames: int,
) -> dict[str, Any]:
    """Use a lower threshold only when block completeness checks are enabled."""
    if is_block_target:
        return {
            "confidence": max(
                float(yolo_confidence),
                float(complete_block_confidence),
            ),
            "require_complete_bbox": True,
            "confirm_frames": max(2, int(block_confirm_frames)),
        }
    return {
        "confidence": max(
            float(yolo_confidence),
            float(regular_confidence),
        ),
        "require_complete_bbox": False,
        "confirm_frames": 1,
    }
