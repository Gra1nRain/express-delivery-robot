"""Pure event-driven state machine for the indoor competition mission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import TypeAlias


class MissionState(str, Enum):
    WAIT_START_FLAG = "WAIT_START_FLAG"
    RUN_TO_TRAFFIC_STOP = "RUN_TO_TRAFFIC_STOP"
    WAIT_TRAFFIC_LIGHT = "WAIT_TRAFFIC_LIGHT"
    WAIT_PICKUP_VISION_READY = "WAIT_PICKUP_VISION_READY"
    RUN_TO_PICKUP_FRONT = "RUN_TO_PICKUP_FRONT"
    PICKUP_FRONT_TASK = "PICKUP_FRONT_TASK"
    RUN_TO_PICKUP_REAR = "RUN_TO_PICKUP_REAR"
    PICKUP_REAR_TASK = "PICKUP_REAR_TASK"
    RUN_TO_DROP_FRONT = "RUN_TO_DROP_FRONT"
    DROP_FRONT_TASK = "DROP_FRONT_TASK"
    RUN_TO_DROP_REAR = "RUN_TO_DROP_REAR"
    DROP_REAR_TASK = "DROP_REAR_TASK"
    PICKUP_FAILED = "PICKUP_FAILED"
    BYPASS_DROP_TASKS = "BYPASS_DROP_TASKS"
    RUN_TO_FINISH = "RUN_TO_FINISH"
    FINISHED = "FINISHED"


class LightObservation(str, Enum):
    UNKNOWN = "unknown"
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    OFF = "off"


class ArmTaskType(str, Enum):
    PICKUP = "PICKUP"
    DROP = "DROP"


class ArmStation(str, Enum):
    FRONT = "FRONT"
    REAR = "REAR"


class ArmOutcome(str, Enum):
    SUCCESS = "SUCCESS"
    INSTRUCTION_NOT_FOUND = "INSTRUCTION_NOT_FOUND"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    OPERATION_FAILED = "OPERATION_FAILED"
    TIMEOUT = "TIMEOUT"


class CommandType(str, Enum):
    SET_ROUTE_ENABLED = "SET_ROUTE_ENABLED"
    SET_TRAFFIC_LIGHT_ENABLED = "SET_TRAFFIC_LIGHT_ENABLED"
    SET_TRAFFIC_STOP_ENABLED = "SET_TRAFFIC_STOP_ENABLED"
    PRELOAD_ARM_VISION = "PRELOAD_ARM_VISION"
    RELEASE_TO_CHECKPOINT = "RELEASE_TO_CHECKPOINT"
    START_ARM_TASK = "START_ARM_TASK"
    CANCEL_ARM_TASK = "CANCEL_ARM_TASK"
    MISSION_FINISHED = "MISSION_FINISHED"


@dataclass(frozen=True)
class MissionConfig:
    traffic_marker_ref: str = "traffic_light_vision_on"
    traffic_checkpoint_ref: str = "traffic_light_stop_line"
    pickup_front_ref: str = "pickup_front"
    pickup_rear_ref: str = "pickup_rear"
    drop_front_ref: str = "drop_front"
    drop_rear_ref: str = "drop_rear"
    finish_ref: str = "finish_park"
    traffic_no_result_timeout_s: float = 15.0
    pickup_timeout_s: float = 120.0
    drop_timeout_s: float = 90.0
    pickup_vision_ready_timeout_s: float = 30.0
    pickup_max_attempts: int = 3
    drop_max_attempts: int = 3
    allow_rear_pickup_fallback: bool = False
    allow_skip_failed_pickup: bool = False

    def __post_init__(self) -> None:
        refs = (
            self.traffic_marker_ref,
            self.traffic_checkpoint_ref,
            self.pickup_front_ref,
            self.pickup_rear_ref,
            self.drop_front_ref,
            self.drop_rear_ref,
            self.finish_ref,
        )
        if any(not ref.strip() for ref in refs):
            raise ValueError("mission semantic refs must be non-empty")
        checkpoint_refs = refs[1:]
        if len(set(checkpoint_refs)) != len(checkpoint_refs):
            raise ValueError("mission checkpoint refs must be unique")
        timeouts = (
            self.traffic_no_result_timeout_s,
            self.pickup_timeout_s,
            self.drop_timeout_s,
            self.pickup_vision_ready_timeout_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in timeouts):
            raise ValueError("mission timeouts must be finite and positive")
        if self.pickup_max_attempts <= 0 or self.drop_max_attempts <= 0:
            raise ValueError("arm max attempts must be positive")


@dataclass(frozen=True)
class FlagDetected:
    now_s: float


@dataclass(frozen=True)
class MarkerPassed:
    now_s: float
    marker_ref: str


@dataclass(frozen=True)
class CheckpointReady:
    now_s: float
    checkpoint_ref: str


@dataclass(frozen=True)
class LightSample:
    now_s: float
    light: LightObservation


@dataclass(frozen=True)
class StableLight:
    now_s: float
    light: LightObservation


@dataclass(frozen=True)
class ArmVisionReady:
    now_s: float


@dataclass(frozen=True)
class ArmResult:
    now_s: float
    task_id: str
    outcome: ArmOutcome
    target_type: str = ""


@dataclass(frozen=True)
class Tick:
    now_s: float


MissionEvent: TypeAlias = (
    FlagDetected
    | MarkerPassed
    | CheckpointReady
    | LightSample
    | StableLight
    | ArmVisionReady
    | ArmResult
    | Tick
)


@dataclass(frozen=True)
class ArmTaskRequest:
    task_id: str
    task_type: ArmTaskType
    station: ArmStation
    target_type_hint: str
    max_attempts: int
    timeout_s: float


@dataclass(frozen=True)
class MissionCommand:
    command_type: CommandType
    enabled: bool | None = None
    checkpoint_ref: str | None = None
    arm_task: ArmTaskRequest | None = None
    task_id: str | None = None
    reason: str = ""


@dataclass(frozen=True)
class MissionDecision:
    state: MissionState
    commands: tuple[MissionCommand, ...]
    has_cargo: bool
    target_type: str
    active_arm_task_id: str | None
    finished: bool


_ARM_STATES = frozenset(
    {
        MissionState.PICKUP_FRONT_TASK,
        MissionState.PICKUP_REAR_TASK,
        MissionState.DROP_FRONT_TASK,
        MissionState.DROP_REAR_TASK,
    }
)


class CompetitionMissionStateMachine:
    """Own all competition-stage transitions behind one event interface."""

    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self._state = MissionState.WAIT_START_FLAG
        self._state_entered_s = float("-inf")
        self._has_cargo = False
        self._target_type = ""
        self._traffic_enabled = False
        self._traffic_no_result_deadline_s: float | None = None
        self._pickup_vision_ready = False
        self._pickup_vision_deadline_s: float | None = None
        self._active_arm_task: ArmTaskRequest | None = None
        self._arm_deadline_s: float | None = None
        self._arm_task_counter = 0

    @property
    def snapshot(self) -> MissionDecision:
        return self._decision()

    def handle(self, event: MissionEvent) -> MissionDecision:
        if not math.isfinite(event.now_s):
            raise ValueError("event now_s must be finite")
        if self._state == MissionState.FINISHED:
            return self._decision()
        if event.now_s < self._state_entered_s:
            return self._decision()

        if isinstance(event, Tick):
            return self._handle_tick(event)
        if isinstance(event, FlagDetected):
            if self._state != MissionState.WAIT_START_FLAG:
                return self._decision()
            self._transition(MissionState.RUN_TO_TRAFFIC_STOP, event.now_s)
            return self._decision(
                MissionCommand(
                    command_type=CommandType.SET_ROUTE_ENABLED,
                    enabled=True,
                    reason="start_flag_detected",
                )
            )
        if isinstance(event, MarkerPassed):
            return self._handle_marker(event)
        if isinstance(event, CheckpointReady):
            return self._handle_checkpoint(event)
        if isinstance(event, LightSample):
            return self._handle_light_sample(event)
        if isinstance(event, StableLight):
            if (
                self._state == MissionState.WAIT_TRAFFIC_LIGHT
                and event.light == LightObservation.GREEN
            ):
                return self._release_from_traffic(event.now_s, "stable_green")
            return self._decision()
        if isinstance(event, ArmVisionReady):
            self._pickup_vision_ready = True
            if self._state == MissionState.WAIT_PICKUP_VISION_READY:
                return self._release_to_pickup(
                    event.now_s,
                    "pickup_vision_ready",
                )
            return self._decision()
        if isinstance(event, ArmResult):
            return self._handle_arm_result(event)
        raise TypeError(f"unsupported mission event: {type(event).__name__}")

    def _handle_marker(self, event: MarkerPassed) -> MissionDecision:
        if (
            self._state != MissionState.RUN_TO_TRAFFIC_STOP
            or event.marker_ref != self.config.traffic_marker_ref
            or self._traffic_enabled
        ):
            return self._decision()
        self._traffic_enabled = True
        return self._decision(
            MissionCommand(
                command_type=CommandType.SET_TRAFFIC_LIGHT_ENABLED,
                enabled=True,
                reason="traffic_vision_marker_passed",
            )
        )

    def _handle_checkpoint(self, event: CheckpointReady) -> MissionDecision:
        ref = event.checkpoint_ref
        if (
            self._state == MissionState.RUN_TO_TRAFFIC_STOP
            and ref == self.config.traffic_checkpoint_ref
        ):
            self._transition(MissionState.WAIT_TRAFFIC_LIGHT, event.now_s)
            self._traffic_no_result_deadline_s = (
                event.now_s + self.config.traffic_no_result_timeout_s
            )
            if self._traffic_enabled:
                return self._decision(
                    MissionCommand(
                        command_type=CommandType.SET_TRAFFIC_STOP_ENABLED,
                        enabled=True,
                        reason="traffic_stop_reached",
                    )
                )
            self._traffic_enabled = True
            return self._decision(
                MissionCommand(
                    command_type=CommandType.SET_TRAFFIC_LIGHT_ENABLED,
                    enabled=True,
                    reason="traffic_stop_reached_before_marker",
                ),
                MissionCommand(
                    command_type=CommandType.SET_TRAFFIC_STOP_ENABLED,
                    enabled=True,
                    reason="traffic_stop_reached",
                ),
            )
        if (
            self._state == MissionState.RUN_TO_PICKUP_FRONT
            and ref == self.config.pickup_front_ref
        ):
            return self._start_arm(
                now_s=event.now_s,
                task_type=ArmTaskType.PICKUP,
                station=ArmStation.FRONT,
            )
        if (
            self._state == MissionState.RUN_TO_PICKUP_REAR
            and ref == self.config.pickup_rear_ref
        ):
            return self._start_arm(
                now_s=event.now_s,
                task_type=ArmTaskType.PICKUP,
                station=ArmStation.REAR,
            )
        if (
            self._state == MissionState.RUN_TO_DROP_FRONT
            and ref == self.config.drop_front_ref
            and self._has_cargo
        ):
            return self._start_arm(
                now_s=event.now_s,
                task_type=ArmTaskType.DROP,
                station=ArmStation.FRONT,
            )
        if (
            self._state == MissionState.RUN_TO_DROP_REAR
            and ref == self.config.drop_rear_ref
            and self._has_cargo
        ):
            return self._start_arm(
                now_s=event.now_s,
                task_type=ArmTaskType.DROP,
                station=ArmStation.REAR,
            )
        if (
            self._state
            in {MissionState.RUN_TO_FINISH, MissionState.BYPASS_DROP_TASKS}
            and ref == self.config.finish_ref
        ):
            self._transition(MissionState.FINISHED, event.now_s)
            return self._decision(
                MissionCommand(
                    command_type=CommandType.MISSION_FINISHED,
                    reason="finish_arrived_and_stopped",
                )
            )
        return self._decision()

    def _handle_light_sample(self, event: LightSample) -> MissionDecision:
        if self._state != MissionState.WAIT_TRAFFIC_LIGHT:
            return self._decision()
        if event.light in {LightObservation.RED, LightObservation.YELLOW}:
            self._traffic_no_result_deadline_s = (
                event.now_s + self.config.traffic_no_result_timeout_s
            )
        return self._decision()

    def _handle_tick(self, event: Tick) -> MissionDecision:
        if (
            self._state == MissionState.WAIT_PICKUP_VISION_READY
            and self._pickup_vision_deadline_s is not None
            and event.now_s >= self._pickup_vision_deadline_s
        ):
            return self._release_to_pickup(
                event.now_s,
                "pickup_vision_preload_timeout",
            )
        if (
            self._state == MissionState.WAIT_TRAFFIC_LIGHT
            and self._traffic_no_result_deadline_s is not None
            and event.now_s >= self._traffic_no_result_deadline_s
        ):
            return self._release_from_traffic(
                event.now_s,
                "traffic_no_result_timeout",
            )
        if (
            self._state in _ARM_STATES
            and self._active_arm_task is not None
            and self._arm_deadline_s is not None
            and event.now_s >= self._arm_deadline_s
        ):
            task_id = self._active_arm_task.task_id
            decision = self._complete_arm_task(
                now_s=event.now_s,
                outcome=ArmOutcome.TIMEOUT,
            )
            return self._decision(
                MissionCommand(
                    command_type=CommandType.CANCEL_ARM_TASK,
                    task_id=task_id,
                    reason="arm_total_timeout",
                ),
                *decision.commands,
            )
        return self._decision()

    def _handle_arm_result(self, event: ArmResult) -> MissionDecision:
        if (
            self._state not in _ARM_STATES
            or self._active_arm_task is None
            or event.task_id != self._active_arm_task.task_id
        ):
            return self._decision()
        outcome = event.outcome
        target_type = event.target_type.strip()
        if target_type:
            self._target_type = target_type
        if (
            self._active_arm_task.task_type == ArmTaskType.PICKUP
            and outcome == ArmOutcome.SUCCESS
            and not self._target_type
        ):
            outcome = ArmOutcome.OPERATION_FAILED
        return self._complete_arm_task(
            now_s=event.now_s,
            outcome=outcome,
        )

    def _start_arm(
        self,
        *,
        now_s: float,
        task_type: ArmTaskType,
        station: ArmStation,
    ) -> MissionDecision:
        self._arm_task_counter += 1
        timeout_s = (
            self.config.pickup_timeout_s
            if task_type == ArmTaskType.PICKUP
            else self.config.drop_timeout_s
        )
        max_attempts = (
            self.config.pickup_max_attempts
            if task_type == ArmTaskType.PICKUP
            else self.config.drop_max_attempts
        )
        task = ArmTaskRequest(
            task_id=f"{task_type.value.lower()}-{station.value.lower()}-"
            f"{self._arm_task_counter}",
            task_type=task_type,
            station=station,
            target_type_hint=self._target_type,
            max_attempts=max_attempts,
            timeout_s=timeout_s,
        )
        target_state = {
            (ArmTaskType.PICKUP, ArmStation.FRONT): MissionState.PICKUP_FRONT_TASK,
            (ArmTaskType.PICKUP, ArmStation.REAR): MissionState.PICKUP_REAR_TASK,
            (ArmTaskType.DROP, ArmStation.FRONT): MissionState.DROP_FRONT_TASK,
            (ArmTaskType.DROP, ArmStation.REAR): MissionState.DROP_REAR_TASK,
        }[(task_type, station)]
        self._transition(target_state, now_s)
        self._active_arm_task = task
        self._arm_deadline_s = now_s + timeout_s
        return self._decision(
            MissionCommand(
                command_type=CommandType.START_ARM_TASK,
                arm_task=task,
                task_id=task.task_id,
                reason="checkpoint_arrived_and_stopped",
            )
        )

    def _complete_arm_task(
        self,
        *,
        now_s: float,
        outcome: ArmOutcome,
    ) -> MissionDecision:
        state = self._state
        self._active_arm_task = None
        self._arm_deadline_s = None

        if state == MissionState.PICKUP_FRONT_TASK:
            if outcome == ArmOutcome.SUCCESS:
                self._has_cargo = True
                return self._release_to(
                    MissionState.RUN_TO_DROP_FRONT,
                    self.config.drop_front_ref,
                    now_s,
                    "pickup_front_success",
                )
            if self.config.allow_rear_pickup_fallback:
                return self._release_to(
                    MissionState.RUN_TO_PICKUP_REAR,
                    self.config.pickup_rear_ref,
                    now_s,
                    f"pickup_front_{outcome.value.lower()}",
                )
            self._transition(MissionState.PICKUP_FAILED, now_s)
            return self._decision()

        if state == MissionState.PICKUP_REAR_TASK:
            if outcome == ArmOutcome.SUCCESS:
                self._has_cargo = True
                return self._release_to(
                    MissionState.RUN_TO_DROP_FRONT,
                    self.config.drop_front_ref,
                    now_s,
                    "pickup_rear_success",
                )
            self._has_cargo = False
            if self.config.allow_skip_failed_pickup:
                return self._release_to(
                    MissionState.BYPASS_DROP_TASKS,
                    self.config.finish_ref,
                    now_s,
                    f"pickup_rear_{outcome.value.lower()}_bypass_drop",
                )
            self._transition(MissionState.PICKUP_FAILED, now_s)
            return self._decision()

        if state == MissionState.DROP_FRONT_TASK:
            if outcome == ArmOutcome.SUCCESS:
                self._has_cargo = False
                return self._release_to(
                    MissionState.RUN_TO_FINISH,
                    self.config.finish_ref,
                    now_s,
                    "drop_front_success",
                )
            return self._release_to(
                MissionState.RUN_TO_DROP_REAR,
                self.config.drop_rear_ref,
                now_s,
                f"drop_front_{outcome.value.lower()}",
            )

        if state == MissionState.DROP_REAR_TASK:
            if outcome == ArmOutcome.SUCCESS:
                self._has_cargo = False
            return self._release_to(
                MissionState.RUN_TO_FINISH,
                self.config.finish_ref,
                now_s,
                f"drop_rear_{outcome.value.lower()}",
            )
        raise RuntimeError(f"arm result received outside arm task state: {state}")

    def _release_from_traffic(self, now_s: float, reason: str) -> MissionDecision:
        self._traffic_no_result_deadline_s = None
        self._traffic_enabled = False
        traffic_commands = (
            MissionCommand(
                command_type=CommandType.SET_TRAFFIC_STOP_ENABLED,
                enabled=False,
                reason=reason,
            ),
            MissionCommand(
                command_type=CommandType.SET_TRAFFIC_LIGHT_ENABLED,
                enabled=False,
                reason=reason,
            ),
            MissionCommand(
                command_type=CommandType.PRELOAD_ARM_VISION,
                enabled=True,
                reason=f"{reason}_preload_pickup_vision",
            ),
        )
        if self._pickup_vision_ready:
            pickup_decision = self._release_to_pickup(now_s, reason)
            return self._decision(*traffic_commands, *pickup_decision.commands)

        self._transition(MissionState.WAIT_PICKUP_VISION_READY, now_s)
        self._pickup_vision_deadline_s = (
            now_s + self.config.pickup_vision_ready_timeout_s
        )
        return self._decision(*traffic_commands)

    def _release_to_pickup(self, now_s: float, reason: str) -> MissionDecision:
        self._pickup_vision_deadline_s = None
        return self._release_to(
            MissionState.RUN_TO_PICKUP_FRONT,
            self.config.pickup_front_ref,
            now_s,
            reason,
        )

    def _release_to(
        self,
        state: MissionState,
        checkpoint_ref: str,
        now_s: float,
        reason: str,
    ) -> MissionDecision:
        self._transition(state, now_s)
        return self._decision(
            MissionCommand(
                command_type=CommandType.RELEASE_TO_CHECKPOINT,
                checkpoint_ref=checkpoint_ref,
                reason=reason,
            )
        )

    def _transition(self, state: MissionState, now_s: float) -> None:
        self._state = state
        self._state_entered_s = now_s

    def _decision(self, *commands: MissionCommand) -> MissionDecision:
        return MissionDecision(
            state=self._state,
            commands=tuple(commands),
            has_cargo=self._has_cargo,
            target_type=self._target_type,
            active_arm_task_id=(
                self._active_arm_task.task_id
                if self._active_arm_task is not None
                else None
            ),
            finished=self._state == MissionState.FINISHED,
        )
