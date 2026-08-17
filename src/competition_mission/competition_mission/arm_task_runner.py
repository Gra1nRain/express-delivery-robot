"""Pure retry/timeout contract shared by real and simulated arm adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Callable, Protocol


class ArmTaskType(str, Enum):
    PICKUP = "PICKUP"
    DROP = "DROP"


class ArmTaskOutcome(str, Enum):
    SUCCESS = "SUCCESS"
    INSTRUCTION_NOT_FOUND = "INSTRUCTION_NOT_FOUND"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    OPERATION_FAILED = "OPERATION_FAILED"
    TIMEOUT = "TIMEOUT"


class ArmTaskPhase(str, Enum):
    MOVING_TO_INSTRUCTION_POSE = "MOVING_TO_INSTRUCTION_POSE"
    RECOGNIZING_INSTRUCTION = "RECOGNIZING_INSTRUCTION"
    TARGET_TYPE_LOCKED = "TARGET_TYPE_LOCKED"
    SEARCHING_TARGET_OBJECT = "SEARCHING_TARGET_OBJECT"
    OPERATING = "OPERATING"
    VERIFYING_OPERATION = "VERIFYING_OPERATION"


@dataclass(frozen=True)
class ArmTaskRequest:
    task_type: ArmTaskType
    target_type_hint: str
    max_attempts: int
    timeout_s: float


@dataclass(frozen=True)
class ArmTaskResult:
    outcome: ArmTaskOutcome
    target_type: str
    detail: str
    attempts: int


class ArmExecutionFailure(RuntimeError):
    def __init__(
        self,
        outcome: ArmTaskOutcome,
        detail: str,
        *,
        target_type: str = "",
    ) -> None:
        if outcome == ArmTaskOutcome.SUCCESS:
            raise ValueError("failure outcome cannot be SUCCESS")
        super().__init__(detail)
        self.outcome = outcome
        self.detail = str(detail)
        self.target_type = str(target_type).strip()


PhasePublisher = Callable[[ArmTaskPhase, str], None]


class ArmBackend(Protocol):
    def pickup_once(
        self,
        target_hint: str,
        publish_phase: PhasePublisher,
    ) -> str: ...

    def drop_once(
        self,
        target_type: str,
        publish_phase: PhasePublisher,
    ) -> None: ...


class ArmTaskRunner:
    def __init__(
        self,
        backend: ArmBackend,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._backend = backend
        self._monotonic = monotonic

    def run(
        self,
        request: ArmTaskRequest,
        *,
        publish_feedback: Callable[[ArmTaskPhase, str, int], None],
        cancel_requested: Callable[[], bool],
    ) -> ArmTaskResult:
        max_attempts = max(1, int(request.max_attempts))
        timeout_s = max(0.0, float(request.timeout_s))
        target_type = str(request.target_type_hint).strip()
        if request.task_type == ArmTaskType.DROP and not target_type:
            return ArmTaskResult(
                ArmTaskOutcome.OPERATION_FAILED,
                "",
                "drop_requires_locked_target_type",
                0,
            )

        started_at = self._monotonic()
        deadline = started_at + timeout_s
        last_failure: ArmExecutionFailure | None = None

        for attempt in range(1, max_attempts + 1):
            if cancel_requested() or self._monotonic() >= deadline:
                return self._timeout_result(target_type, attempt - 1)

            def publish_phase(phase: ArmTaskPhase, phase_target: str) -> None:
                if cancel_requested() or self._monotonic() >= deadline:
                    raise ArmExecutionFailure(
                        ArmTaskOutcome.TIMEOUT,
                        "arm_task_cancelled_or_timed_out",
                        target_type=phase_target or target_type,
                    )
                publish_feedback(phase, phase_target, attempt)

            try:
                if request.task_type == ArmTaskType.PICKUP:
                    target_type = self._backend.pickup_once(
                        target_type,
                        publish_phase,
                    ).strip()
                    if not target_type:
                        raise ArmExecutionFailure(
                            ArmTaskOutcome.INSTRUCTION_NOT_FOUND,
                            "pickup_completed_without_target_type",
                        )
                else:
                    self._backend.drop_once(target_type, publish_phase)
            except ArmExecutionFailure as exc:
                last_failure = exc
                if exc.target_type:
                    target_type = exc.target_type
                if exc.outcome == ArmTaskOutcome.TIMEOUT:
                    return self._timeout_result(target_type, attempt)
                continue
            except Exception as exc:
                last_failure = ArmExecutionFailure(
                    ArmTaskOutcome.OPERATION_FAILED,
                    f"unexpected_backend_error: {exc}",
                    target_type=target_type,
                )
                continue

            if cancel_requested() or self._monotonic() >= deadline:
                return self._timeout_result(target_type, attempt)
            return ArmTaskResult(
                ArmTaskOutcome.SUCCESS,
                target_type,
                "operation_verified",
                attempt,
            )

        assert last_failure is not None
        return ArmTaskResult(
            last_failure.outcome,
            target_type,
            last_failure.detail,
            max_attempts,
        )

    @staticmethod
    def _timeout_result(target_type: str, attempts: int) -> ArmTaskResult:
        return ArmTaskResult(
            ArmTaskOutcome.TIMEOUT,
            target_type,
            "arm_task_cancelled_or_timed_out",
            attempts,
        )
