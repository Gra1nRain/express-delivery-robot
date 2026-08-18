import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
MIGRATION_ROOT = REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723"
sys.path.insert(0, str(MIGRATION_ROOT))

from gripper_hold_guard import (  # noqa: E402
    GripperHoldGuard,
    choose_bottle_hold_position,
    evaluate_contact_stability,
)


class GripperHoldGuardTest(unittest.TestCase):
    def test_contact_stability_accepts_a_settled_object_contact(self):
        result = evaluate_contact_stability(
            [0.0460, 0.0456, 0.0458, 0.0457],
            min_samples=4,
            max_span_m=0.0015,
            min_contact_opening_m=0.008,
        )

        self.assertTrue(result["stable"])
        self.assertAlmostEqual(result["median_opening_m"], 0.04575)
        self.assertAlmostEqual(result["span_m"], 0.0004)

    def test_contact_stability_rejects_rebound_during_close(self):
        result = evaluate_contact_stability(
            [0.046, 0.052, 0.045, 0.051],
            min_samples=4,
            max_span_m=0.0015,
            min_contact_opening_m=0.008,
        )

        self.assertFalse(result["stable"])
        self.assertGreater(result["span_m"], 0.0015)

    def test_contact_stability_rejects_an_empty_fully_closed_gripper(self):
        result = evaluate_contact_stability(
            [0.0004, 0.0002, 0.0003, 0.0002],
            min_samples=4,
            max_span_m=0.0015,
            min_contact_opening_m=0.008,
        )

        self.assertFalse(result["stable"])

    def test_bottle_hold_uses_contact_opening_with_small_preload(self):
        self.assertAlmostEqual(
            choose_bottle_hold_position(0.0, 0.032, 0.002),
            0.030,
        )

    def test_bottle_hold_never_opens_past_measured_contact(self):
        self.assertEqual(
            choose_bottle_hold_position(0.010, 0.009, 0.002),
            0.010,
        )

    def test_unlocked_guard_preserves_requested_opening(self):
        guard = GripperHoldGuard()

        effective, clamped = guard.apply(0.080)

        self.assertEqual(effective, 0.080)
        self.assertFalse(clamped)

    def test_active_hold_blocks_opening_but_allows_tighter_close(self):
        guard = GripperHoldGuard()
        guard.activate(0.012, "yellow_block")

        opening, opening_clamped = guard.apply(0.080)
        tighter, tighter_clamped = guard.apply(0.008)

        self.assertEqual(opening, 0.012)
        self.assertTrue(opening_clamped)
        self.assertEqual(tighter, 0.008)
        self.assertFalse(tighter_clamped)
        self.assertTrue(guard.is_active())

    def test_release_requires_authorization_and_completion(self):
        guard = GripperHoldGuard()
        guard.activate(0.012, "yellow_block")

        guard.authorize_release("place step 4")
        effective, clamped = guard.apply(0.080)

        self.assertEqual(effective, 0.080)
        self.assertFalse(clamped)
        self.assertTrue(guard.is_active())

        guard.complete_release()
        self.assertFalse(guard.is_active())

    def test_cancelled_release_restores_closed_hold(self):
        guard = GripperHoldGuard()
        guard.activate(0.012, "yellow_block")
        guard.authorize_release("place step 4")
        guard.cancel_release()

        effective, clamped = guard.apply(0.080)

        self.assertEqual(effective, 0.012)
        self.assertTrue(clamped)
        self.assertTrue(guard.is_active())
