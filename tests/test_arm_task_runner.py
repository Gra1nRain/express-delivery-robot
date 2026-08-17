import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_mission"))

from competition_mission.arm_task_runner import (
    ArmExecutionFailure,
    ArmTaskOutcome,
    ArmTaskPhase,
    ArmTaskRequest,
    ArmTaskRunner,
    ArmTaskType,
)


class FakeBackend:
    def __init__(self, pickup=None, drop=None):
        self.pickup_results = list(pickup or [])
        self.drop_results = list(drop or [])
        self.pickup_hints = []
        self.drop_targets = []

    def pickup_once(self, target_hint, publish_phase):
        self.pickup_hints.append(target_hint)
        publish_phase(ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE, target_hint)
        if target_hint:
            target_type = target_hint
        else:
            publish_phase(ArmTaskPhase.RECOGNIZING_INSTRUCTION, "")
            target_type = "green_bottle"
        publish_phase(ArmTaskPhase.TARGET_TYPE_LOCKED, target_type)
        publish_phase(ArmTaskPhase.SEARCHING_TARGET_OBJECT, target_type)
        publish_phase(ArmTaskPhase.OPERATING, target_type)
        publish_phase(ArmTaskPhase.VERIFYING_OPERATION, target_type)
        result = self.pickup_results.pop(0)
        if isinstance(result, Exception):
            raise result
        return target_type

    def drop_once(self, target_type, publish_phase):
        self.drop_targets.append(target_type)
        publish_phase(ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE, target_type)
        publish_phase(ArmTaskPhase.RECOGNIZING_INSTRUCTION, target_type)
        publish_phase(ArmTaskPhase.TARGET_TYPE_LOCKED, target_type)
        publish_phase(ArmTaskPhase.OPERATING, target_type)
        publish_phase(ArmTaskPhase.VERIFYING_OPERATION, target_type)
        result = self.drop_results.pop(0)
        if isinstance(result, Exception):
            raise result


class ArmTaskRunnerTest(unittest.TestCase):
    def _run(self, backend, request, cancel=lambda: False):
        feedback = []
        result = ArmTaskRunner(backend).run(
            request,
            publish_feedback=lambda phase, target, attempt: feedback.append(
                (phase, target, attempt)
            ),
            cancel_requested=cancel,
        )
        return result, feedback

    def test_pickup_recognizes_instruction_then_locks_target(self):
        backend = FakeBackend(pickup=[True])
        result, feedback = self._run(
            backend,
            ArmTaskRequest(ArmTaskType.PICKUP, "", 3, 120.0),
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.SUCCESS)
        self.assertEqual(result.target_type, "green_bottle")
        self.assertEqual(result.attempts, 1)
        self.assertEqual(
            [item[0] for item in feedback],
            [
                ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE,
                ArmTaskPhase.RECOGNIZING_INSTRUCTION,
                ArmTaskPhase.TARGET_TYPE_LOCKED,
                ArmTaskPhase.SEARCHING_TARGET_OBJECT,
                ArmTaskPhase.OPERATING,
                ArmTaskPhase.VERIFYING_OPERATION,
            ],
        )

    def test_rear_pickup_reuses_locked_target_hint(self):
        backend = FakeBackend(pickup=[True])
        result, feedback = self._run(
            backend,
            ArmTaskRequest(
                ArmTaskType.PICKUP,
                "purple_bottle",
                3,
                120.0,
            ),
        )

        self.assertEqual(result.target_type, "purple_bottle")
        self.assertNotIn(
            ArmTaskPhase.RECOGNIZING_INSTRUCTION,
            [item[0] for item in feedback],
        )
        self.assertEqual(backend.pickup_hints, ["purple_bottle"])

    def test_retry_uses_same_target_after_target_search_failure(self):
        backend = FakeBackend(
            pickup=[
                ArmExecutionFailure(
                    ArmTaskOutcome.TARGET_NOT_FOUND,
                    "first search failed",
                    target_type="orange_bottle",
                ),
                True,
            ]
        )
        result, feedback = self._run(
            backend,
            ArmTaskRequest(ArmTaskType.PICKUP, "", 3, 120.0),
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.SUCCESS)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(
            backend.pickup_hints,
            ["", "orange_bottle"],
        )
        self.assertIn(2, [item[2] for item in feedback])

    def test_final_failure_is_returned_after_max_attempts(self):
        failure = ArmExecutionFailure(
            ArmTaskOutcome.OPERATION_FAILED,
            "gripper verification failed",
            target_type="green_bottle",
        )
        backend = FakeBackend(pickup=[failure, failure, failure])
        result, _ = self._run(
            backend,
            ArmTaskRequest(ArmTaskType.PICKUP, "", 3, 120.0),
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.OPERATION_FAILED)
        self.assertEqual(result.attempts, 3)
        self.assertEqual(result.target_type, "green_bottle")

    def test_drop_retries_and_keeps_pickup_target_type(self):
        backend = FakeBackend(
            drop=[
                ArmExecutionFailure(
                    ArmTaskOutcome.TARGET_NOT_FOUND,
                    "sign not found",
                    target_type="green_bottle",
                ),
                True,
            ]
        )
        result, _ = self._run(
            backend,
            ArmTaskRequest(
                ArmTaskType.DROP,
                "green_bottle",
                3,
                90.0,
            ),
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.SUCCESS)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(
            backend.drop_targets,
            ["green_bottle", "green_bottle"],
        )

    def test_drop_without_cargo_target_is_rejected(self):
        backend = FakeBackend(drop=[True])
        result, feedback = self._run(
            backend,
            ArmTaskRequest(ArmTaskType.DROP, "", 3, 90.0),
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.OPERATION_FAILED)
        self.assertEqual(result.attempts, 0)
        self.assertEqual(feedback, [])
        self.assertEqual(backend.drop_targets, [])

    def test_cancel_before_attempt_returns_timeout_without_backend_call(self):
        backend = FakeBackend(pickup=[True])
        result, feedback = self._run(
            backend,
            ArmTaskRequest(ArmTaskType.PICKUP, "", 3, 120.0),
            cancel=lambda: True,
        )

        self.assertEqual(result.outcome, ArmTaskOutcome.TIMEOUT)
        self.assertEqual(result.attempts, 0)
        self.assertEqual(feedback, [])
        self.assertEqual(backend.pickup_hints, [])


if __name__ == "__main__":
    unittest.main()
