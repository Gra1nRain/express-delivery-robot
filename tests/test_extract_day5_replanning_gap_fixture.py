import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = REPO_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from extract_day5_replanning_gap_fixture import (  # noqa: E402
    StatusRecord,
    largest_status_gap,
)


class ExtractDay5ReplanningGapFixtureTest(unittest.TestCase):
    def test_largest_status_gap_preserves_surrounding_payloads(self) -> None:
        records = (
            StatusRecord(1_000_000_000, {"status": "REFERENCE_CLEAR"}),
            StatusRecord(2_000_000_000, {"status": "REPLANNED"}),
            StatusRecord(
                72_000_000_000,
                {"status": "PLAN_FAILED", "detail": "no path"},
            ),
            StatusRecord(73_000_000_000, {"status": "REFERENCE_CLEAR"}),
        )

        before, after = largest_status_gap(records)

        self.assertEqual(before.payload["status"], "REPLANNED")
        self.assertEqual(after.payload["status"], "PLAN_FAILED")
        self.assertEqual(after.timestamp_ns - before.timestamp_ns, 70_000_000_000)

    def test_largest_status_gap_requires_two_records(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least two"):
            largest_status_gap((StatusRecord(1, {"status": "only"}),))


if __name__ == "__main__":
    unittest.main()
