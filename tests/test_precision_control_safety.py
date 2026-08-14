import json
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.control_safety import (
    alignment_gate_decision,
    segmented_safety_stop_requested,
    update_local_hard_stop_latch,
)


class PrecisionControlSafetyRegressionTest(unittest.TestCase):
    def test_alignment_gate_blocks_start_and_requests_stopped_checkpoint(self) -> None:
        startup = alignment_gate_decision(
            required=True,
            route_enabled=True,
            status_age_s=0.1,
            status_timeout_s=1.0,
            startup_ready=False,
            checkpoint_hold=False,
            checkpoint_ready_ref=None,
            active_checkpoint_ref="pickup_front",
            dock_hold_reached=False,
        )
        approaching = alignment_gate_decision(
            required=True,
            route_enabled=True,
            status_age_s=0.1,
            status_timeout_s=1.0,
            startup_ready=True,
            checkpoint_hold=False,
            checkpoint_ready_ref=None,
            active_checkpoint_ref="pickup_front",
            dock_hold_reached=False,
        )
        docked = alignment_gate_decision(
            required=True,
            route_enabled=True,
            status_age_s=0.1,
            status_timeout_s=1.0,
            startup_ready=True,
            checkpoint_hold=False,
            checkpoint_ready_ref=None,
            active_checkpoint_ref="pickup_front",
            dock_hold_reached=True,
        )

        self.assertTrue(startup.hold_requested)
        self.assertFalse(approaching.hold_requested)
        self.assertFalse(approaching.completion_allowed)
        self.assertFalse(approaching.request_checkpoint)
        self.assertTrue(docked.request_checkpoint)

    def test_stale_alignment_status_fails_safe_while_route_is_enabled(self) -> None:
        decision = alignment_gate_decision(
            required=True,
            route_enabled=True,
            status_age_s=1.5,
            status_timeout_s=1.0,
            startup_ready=True,
            checkpoint_hold=False,
            checkpoint_ready_ref=None,
            active_checkpoint_ref="pickup_front",
            dock_hold_reached=False,
        )

        self.assertTrue(decision.hold_requested)
        self.assertFalse(decision.completion_allowed)
        self.assertEqual(decision.reason, "alignment_status_stale")

    def test_start_pose_blocked_stays_latched_until_planner_recovers(self) -> None:
        fixture = (
            REPO_ROOT
            / "tests"
            / "fixtures"
            / "day5_precision_start_blocked_20260814.jsonl"
        )
        rows = [
            json.loads(line)
            for line in fixture.read_text(encoding="utf-8").splitlines()
        ]

        hard_stop_requested = False
        for row in rows:
            hard_stop_requested = update_local_hard_stop_latch(
                hard_stop_requested,
                row["local_status"],
            )
            self.assertEqual(
                hard_stop_requested,
                row["expected_hard_stop"],
                f"elapsed_s={row['elapsed_s']}",
            )
            self.assertEqual(
                segmented_safety_stop_requested(
                    replanning_enabled=True,
                    precision_active=row["precision_active"],
                    local_stop_requested=row["local_stop_requested"],
                    local_hard_stop_requested=hard_stop_requested,
                    local_plan_stale=False,
                    avoidance_stop_requested=False,
                ),
                row["expected_hard_stop"],
                f"elapsed_s={row['elapsed_s']}",
            )

        self.assertGreater(rows[0]["recorded_body_cmd_x"], 0.0)
        self.assertGreater(rows[-1]["recorded_body_cmd_x"], 0.0)

    def test_non_start_planning_failure_does_not_create_precision_hard_stop(
        self,
    ) -> None:
        status = {
            "status": "HYBRID_ASTAR_NO_FEASIBLE_PATH",
            "stop_requested": True,
            "detail": (
                "hybrid_astar found no forward pose-constrained path "
                "from (6.630, -0.001, -0.136) to (8.413, -0.081, 0.015)"
            ),
        }

        hard_stop_requested = update_local_hard_stop_latch(False, status)

        self.assertFalse(hard_stop_requested)
        self.assertFalse(
            segmented_safety_stop_requested(
                replanning_enabled=True,
                precision_active=True,
                local_stop_requested=True,
                local_hard_stop_requested=hard_stop_requested,
                local_plan_stale=False,
                avoidance_stop_requested=False,
            )
        )

    def test_proximity_stop_remains_effective_during_precision(self) -> None:
        self.assertTrue(
            segmented_safety_stop_requested(
                replanning_enabled=True,
                precision_active=True,
                local_stop_requested=False,
                local_hard_stop_requested=False,
                local_plan_stale=False,
                avoidance_stop_requested=True,
            )
        )


if __name__ == "__main__":
    unittest.main()
