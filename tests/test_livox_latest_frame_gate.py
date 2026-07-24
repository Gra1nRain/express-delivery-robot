import pathlib
import sys
import unittest


PACKAGE_ROOT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "src"
    / "competition_avoidance"
)
sys.path.insert(0, str(PACKAGE_ROOT))

from competition_avoidance.livox_latest_frame_gate import LatestFrameGate


class LatestFrameGateTest(unittest.TestCase):
    def test_only_a_new_sequence_can_be_published(self) -> None:
        gate = LatestFrameGate(maximum_age_s=0.4)

        self.assertTrue(gate.should_publish(sequence=1, age_s=0.1))
        gate.mark_published(sequence=1)
        self.assertFalse(gate.should_publish(sequence=1, age_s=0.1))
        self.assertTrue(gate.should_publish(sequence=2, age_s=0.1))

    def test_stale_or_invalid_frames_are_rejected(self) -> None:
        gate = LatestFrameGate(maximum_age_s=0.4)

        self.assertFalse(gate.should_publish(sequence=1, age_s=-0.01))
        self.assertFalse(gate.should_publish(sequence=1, age_s=0.401))
        self.assertTrue(gate.should_publish(sequence=1, age_s=0.4))

    def test_configuration_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            LatestFrameGate(maximum_age_s=0.0)


if __name__ == "__main__":
    unittest.main()
