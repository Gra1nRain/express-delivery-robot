"""HSV red-region extraction used by the flag-wave state machine."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class RedFlagDetection:
    centroid_x: int
    centroid_y: int
    x: int
    y: int
    width: int
    height: int
    area_px: float


class RedFlagColorDetector:
    def __init__(
        self,
        *,
        saturation_threshold: int = 100,
        value_threshold: int = 100,
        min_area_px: float = 800.0,
    ) -> None:
        self._saturation_threshold = int(saturation_threshold)
        self._value_threshold = int(value_threshold)
        self._min_area_px = float(min_area_px)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect(self, frame: np.ndarray) -> RedFlagDetection | None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_1 = np.array(
            [0, self._saturation_threshold, self._value_threshold], dtype=np.uint8
        )
        high_1 = np.array([10, 255, 255], dtype=np.uint8)
        low_2 = np.array(
            [160, self._saturation_threshold, self._value_threshold], dtype=np.uint8
        )
        high_2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, low_1, high_1),
            cv2.inRange(hsv, low_2, high_2),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self._kernel, iterations=2
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self._kernel, iterations=2
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        if area < self._min_area_px:
            return None
        x, y, width, height = cv2.boundingRect(largest)
        return RedFlagDetection(
            centroid_x=x + width // 2,
            centroid_y=y + height // 2,
            x=x,
            y=y,
            width=width,
            height=height,
            area_px=area,
        )
