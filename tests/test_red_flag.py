import pathlib
import sys

import pytest


cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_perception"))

from competition_perception.red_flag import RedFlagColorDetector


def test_interior_flag_wins_over_larger_border_red_region() -> None:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (586, 332), (639, 406), (0, 0, 255), -1)
    cv2.rectangle(frame, (220, 120), (249, 159), (0, 0, 255), -1)

    detection = RedFlagColorDetector(min_area_px=800.0).detect(frame)

    assert detection is not None
    assert detection.x == 220
    assert detection.y == 120


def test_border_red_region_alone_is_not_a_flag() -> None:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (586, 332), (639, 406), (0, 0, 255), -1)

    detection = RedFlagColorDetector(min_area_px=800.0).detect(frame)

    assert detection is None
