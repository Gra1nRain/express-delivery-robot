"""Traditional HSV bright-spot detector for red, yellow, and green lights."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class LightSpotConfig:
    brightness_threshold: int = 100
    min_area_px: float = 30.0
    max_area_px: float = 8000.0
    min_circularity: float = 0.70
    morphology_kernel_size: int = 9

    def __post_init__(self) -> None:
        if not 0 <= self.brightness_threshold <= 255:
            raise ValueError("brightness_threshold must be in [0, 255]")
        if self.min_area_px < 0.0 or self.max_area_px <= self.min_area_px:
            raise ValueError("spot area limits are invalid")
        if not 0.0 <= self.min_circularity <= 1.0:
            raise ValueError("min_circularity must be in [0, 1]")
        if (
            self.morphology_kernel_size <= 0
            or self.morphology_kernel_size % 2 == 0
        ):
            raise ValueError("morphology_kernel_size must be a positive odd integer")


@dataclass(frozen=True)
class LightSpotDetection:
    color_name: str
    center_x: int
    center_y: int
    radius_px: int
    bbox: tuple[int, int, int, int]
    area_px: float
    circularity: float
    center_brightness: float
    confidence: float


_COLOR_RANGES = {
    "red": (
        ((0, 150, 100), (10, 255, 255)),
        ((160, 150, 100), (180, 255, 255)),
    ),
    "yellow": (((18, 160, 120), (35, 255, 255)),),
    "green": (((40, 150, 100), (85, 255, 255)),),
}


class LightSpotDetector:
    """Detect the strongest circular colored light spot in a BGR image."""

    def __init__(self, config: LightSpotConfig | None = None) -> None:
        self.config = config or LightSpotConfig()
        size = self.config.morphology_kernel_size
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def detect(self, frame) -> LightSpotDetection | None:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a BGR image")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        candidates: list[LightSpotDetection] = []

        for color_name, ranges in _COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                dynamic_lower = np.array(lower, dtype=np.uint8)
                dynamic_lower[2] = self.config.brightness_threshold
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(
                        hsv,
                        dynamic_lower,
                        np.array(upper, dtype=np.uint8),
                    ),
                )
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self._kernel, iterations=2
            )
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self._kernel, iterations=1
            )
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                candidate = self._candidate_from_contour(
                    contour,
                    color_name=color_name,
                    v_channel=v_channel,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                )
                if candidate is not None:
                    candidates.append(candidate)

        candidates = self._suppress_dimmer_spots(candidates)
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda item: (
                item.center_brightness,
                item.circularity,
                item.area_px,
            ),
        )

    def _candidate_from_contour(
        self,
        contour,
        *,
        color_name: str,
        v_channel,
        image_width: int,
        image_height: int,
    ) -> LightSpotDetection | None:
        area = float(cv2.contourArea(contour))
        if area < self.config.min_area_px or area > self.config.max_area_px:
            return None
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            return None
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < self.config.min_circularity:
            return None

        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        cx, cy, radius_px = int(center_x), int(center_y), int(radius)
        brightness = _center_brightness(v_channel, cx, cy, radius_px)
        confidence = min(1.0, (brightness / 255.0) * min(1.0, circularity))
        return LightSpotDetection(
            color_name=color_name,
            center_x=cx,
            center_y=cy,
            radius_px=radius_px,
            bbox=(
                max(0, cx - radius_px),
                max(0, cy - radius_px),
                min(image_width - 1, cx + radius_px),
                min(image_height - 1, cy + radius_px),
            ),
            area_px=area,
            circularity=circularity,
            center_brightness=brightness,
            confidence=confidence,
        )

    def _suppress_dimmer_spots(
        self, candidates: list[LightSpotDetection]
    ) -> list[LightSpotDetection]:
        if len(candidates) <= 1:
            return candidates
        ordered = sorted(
            candidates,
            key=lambda item: item.center_brightness,
            reverse=True,
        )
        return ordered[:1]


def _center_brightness(v_channel, cx: int, cy: int, radius: int) -> float:
    sample_radius = max(3, radius // 3)
    y1, y2 = max(0, cy - sample_radius), min(
        v_channel.shape[0], cy + sample_radius
    )
    x1, x2 = max(0, cx - sample_radius), min(
        v_channel.shape[1], cx + sample_radius
    )
    region = v_channel[y1:y2, x1:x2]
    return 0.0 if region.size == 0 else float(region.mean())
