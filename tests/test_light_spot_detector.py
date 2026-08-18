from pathlib import Path
import sys

import pytest


cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_perception"))

from competition_perception.light_spot_detector import (
    LightSpotConfig,
    LightSpotDetector,
)


@pytest.mark.parametrize(
    ("color_name", "bgr"),
    (
        ("red", (0, 0, 255)),
        ("yellow", (0, 255, 255)),
        ("green", (0, 255, 0)),
    ),
)
def test_detects_bright_circular_traffic_lights(color_name, bgr) -> None:
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(frame, (160, 120), 18, bgr, -1)

    detection = LightSpotDetector().detect(frame)

    assert detection is not None
    assert detection.color_name == color_name
    assert abs(detection.center_x - 160) <= 2
    assert abs(detection.center_y - 120) <= 2


def test_rejects_small_noise_and_non_circular_regions() -> None:
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(frame, (40, 40), 2, (0, 0, 255), -1)
    cv2.rectangle(frame, (100, 100), (190, 112), (0, 255, 0), -1)

    assert LightSpotDetector().detect(frame) is None


def test_dominant_bright_spot_wins() -> None:
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(frame, (90, 120), 18, (0, 0, 130), -1)
    cv2.circle(frame, (230, 120), 18, (0, 255, 0), -1)

    detection = LightSpotDetector().detect(frame)

    assert detection is not None
    assert detection.color_name == "green"
    assert detection.center_x > 200


@pytest.mark.parametrize(
    ("hue", "saturation"),
    (
        (0, 149),
        (25, 159),
        (60, 149),
    ),
)
def test_rejects_spots_below_new_color_saturation_limits(
    hue, saturation
) -> None:
    hsv_color = np.array([[[hue, saturation, 255]]], dtype=np.uint8)
    bgr = tuple(
        int(channel)
        for channel in cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
    )
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(frame, (160, 120), 18, bgr, -1)

    assert LightSpotDetector().detect(frame) is None


def test_uses_new_script_circularity_threshold_by_default() -> None:
    assert LightSpotConfig().min_circularity == pytest.approx(0.70)
