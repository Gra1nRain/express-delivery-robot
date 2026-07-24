import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.livox_clock import ClockOffsetEstimator


class ClockOffsetEstimatorTests(unittest.TestCase):
    def test_uses_minimum_calibration_delay_and_freezes_it(self):
        estimator = ClockOffsetEstimator(calibration_samples=3)

        self.assertFalse(
            estimator.observe(
                source_stamp_ns=10_000_000_000,
                receipt_stamp_ns=11_320_000_000,
            )
        )
        self.assertFalse(
            estimator.observe(
                source_stamp_ns=10_010_000_000,
                receipt_stamp_ns=11_310_000_000,
            )
        )
        self.assertTrue(
            estimator.observe(
                source_stamp_ns=10_020_000_000,
                receipt_stamp_ns=11_330_000_000,
            )
        )

        self.assertEqual(estimator.offset_ns, 1_300_000_000)
        estimator.observe(
            source_stamp_ns=20_000_000_000,
            receipt_stamp_ns=22_000_000_000,
        )
        self.assertEqual(estimator.offset_ns, 1_300_000_000)

    def test_applies_one_offset_to_lidar_and_imu_clock_values(self):
        estimator = ClockOffsetEstimator(calibration_samples=1)
        estimator.observe(
            source_stamp_ns=100_000_000_000,
            receipt_stamp_ns=101_250_000_000,
        )

        lidar_ns = estimator.rebase(100_100_000_000)
        imu_ns = estimator.rebase(100_125_000_000)

        self.assertEqual(lidar_ns, 101_350_000_000)
        self.assertEqual(imu_ns, 101_375_000_000)
        self.assertEqual(imu_ns - lidar_ns, 25_000_000)

    def test_requires_calibration_before_rebasing(self):
        estimator = ClockOffsetEstimator(calibration_samples=2)

        with self.assertRaisesRegex(RuntimeError, "not calibrated"):
            estimator.rebase(1)

    def test_rejects_invalid_configuration_and_timestamps(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            ClockOffsetEstimator(calibration_samples=0)

        estimator = ClockOffsetEstimator(calibration_samples=1)
        with self.assertRaisesRegex(ValueError, "non-negative"):
            estimator.observe(source_stamp_ns=-1, receipt_stamp_ns=0)


if __name__ == "__main__":
    unittest.main()
