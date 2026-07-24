import pathlib
import sys
import unittest


PACKAGE_ROOT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "src"
    / "competition_avoidance"
)
sys.path.insert(0, str(PACKAGE_ROOT))

from competition_avoidance.rate_gate import LatestSampleRateGate


class LatestSampleRateGateTest(unittest.TestCase):
    def test_limits_expensive_processing_to_configured_frequency(self) -> None:
        gate = LatestSampleRateGate(10.0)

        self.assertTrue(gate.allow(1.00))
        self.assertFalse(gate.allow(1.05))
        self.assertTrue(gate.allow(1.10))
        self.assertFalse(gate.allow(1.19))
        self.assertTrue(gate.allow(1.20))

    def test_rejects_invalid_frequency_and_timestamp(self) -> None:
        with self.assertRaises(ValueError):
            LatestSampleRateGate(0.0)

        gate = LatestSampleRateGate(10.0)
        with self.assertRaises(ValueError):
            gate.allow(float("nan"))


if __name__ == "__main__":
    unittest.main()
