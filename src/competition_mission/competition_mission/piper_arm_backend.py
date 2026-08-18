"""Adapter around the tested Piper migration without modifying its sources."""

from __future__ import annotations

import importlib
import math
import os
from pathlib import Path
import re
import sys
import time
from types import ModuleType
from typing import Any

from competition_mission.arm_task_runner import (
    ArmExecutionFailure,
    ArmTaskOutcome,
    ArmTaskPhase,
    PhasePublisher,
)


_SHELL_DEFAULT_RE = re.compile(
    r'^export\s+([A-Z][A-Z0-9_]*)="\$\{\1:-(.*)\}"$'
)

COMPETITION_TRANSIT_JOINTS_RAD = (
    0.005760,
    0.289742,
    -0.565347,
    -0.081856,
    0.045605,
    0.092502,
)


def apply_migration_shell_defaults(script_path: Path) -> int:
    """Apply the simple ``${NAME:-default}`` calibration exports."""
    applied = 0
    for raw_line in script_path.read_text(encoding="utf-8").splitlines():
        match = _SHELL_DEFAULT_RE.match(raw_line.strip())
        if match is None:
            continue
        name, value = match.groups()
        if name not in os.environ:
            os.environ[name] = value
            applied += 1
    return applied


def prepare_competition_environment(migration_root: Path) -> int:
    migration_root = migration_root.expanduser().resolve()
    applied = apply_migration_shell_defaults(
        migration_root / "run_grasp_single.sh"
    )
    overrides = {
        "WRIST_AUTO_GRASP": "0",
        "WRIST_AUTO_PREVIEW": "0",
        "WRIST_AUTO_MOVE_OBSERVE": "0",
        "WRIST_STARTUP_MOVE_OBSERVE": "0",
        "WRIST_PRE_INSTRUCTION_ENTER_CONFIRM": "0",
        "WRIST_PRE_GRASP_ENTER_CONFIRM": "0",
        "WRIST_PLACE_PRE_DETECT_ENTER_ENABLED": "0",
        "WRIST_RETRY_GRASP_ON_TARGET_FAILURE": "0",
        "WRIST_PLACE_AFTER_GRASP_ENABLED": "1",
        "WRIST_YOLO_MODEL_PATH": str(migration_root / "best.pt"),
    }
    os.environ.update(overrides)
    return applied


def load_piper_modules(migration_root: Path) -> tuple[ModuleType, ModuleType]:
    migration_root = migration_root.expanduser().resolve()
    required = [
        migration_root / "grasp_single.py",
        migration_root / "place_after_grasp.py",
        migration_root / "run_grasp_single.sh",
        migration_root / "best.pt",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Piper migration is incomplete: " + ", ".join(missing)
        )

    prepare_competition_environment(migration_root)
    root_text = str(migration_root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)

    place_module = importlib.import_module("place_after_grasp")
    grasp_module = importlib.import_module("grasp_single")
    for module in (place_module, grasp_module):
        module_file = Path(str(module.__file__)).resolve()
        if migration_root not in module_file.parents:
            raise ImportError(
                f"loaded {module.__name__} from unexpected path: {module_file}"
            )

    # The legacy worker calls this symbol immediately after every grasp.  The
    # competition supervisor owns DROP as a separate task, so only this local
    # module binding is replaced; the original migration file is untouched.
    grasp_module.execute_place_after_grasp = lambda *_args, **_kwargs: True
    place_module.PLACE_AFTER_GRASP_ENABLED = True
    place_module.PLACE_PRE_DETECT_ENTER_ENABLED = False
    return grasp_module, place_module


def create_competition_piper_controller(
    grasp_module: ModuleType,
    *,
    manage_camera: bool = True,
):
    class CompetitionPiperController(grasp_module.PiperController):
        def __init__(self) -> None:
            self.competition_grasp_failed = False
            self.last_gripper_position_m = None
            self.last_gripper_effort_nm = None
            self.last_gripper_feedback_at = None
            super().__init__(
                {
                    "auto_grasp": False,
                    "auto_preview": False,
                    "auto_instruction_target": True,
                    "auto_move_observe": False,
                    "auto_zero_observe": False,
                }
            )

        def terminal_input_worker(self) -> None:
            return

        def initialize_ui(self) -> bool:
            self.ui_initialized = False
            return False

        def start_piper_node(self) -> bool:
            topics = ("/arm_status", "/joint_states_single", "/end_pose")
            if all(self.topic_has_publishers(topic) for topic in topics):
                self.get_logger().info(
                    "Detected an existing Piper control node; reusing it"
                )
                return True
            return super().start_piper_node()

        def start_camera_node(self) -> bool:
            if not manage_camera:
                self.get_logger().info(
                    "Wrist camera is managed by competition bringup; "
                    "subscribing to the shared RGBD topics"
                )
                return True
            return super().start_camera_node()

        def joint_feedback_callback(self, message) -> None:
            super().joint_feedback_callback(message)
            if len(message.position) < 7:
                return
            position = float(message.position[6])
            effort = (
                float(message.effort[6])
                if len(message.effort) >= 7
                else math.nan
            )
            if not math.isfinite(position):
                return
            self.last_gripper_position_m = position
            self.last_gripper_effort_nm = (
                effort if math.isfinite(effort) else None
            )
            self.last_gripper_feedback_at = time.monotonic()

        def return_to_observation_after_grasp_failure(self) -> bool:
            self.competition_grasp_failed = True
            try:
                super().return_to_observation_after_grasp_failure()
            except Exception as exc:
                self.get_logger().error(
                    f"Competition grasp recovery also failed: {exc}"
                )
            # Disable the legacy interactive auto-retry. ArmTaskRunner owns it.
            return False

    return CompetitionPiperController()


class PiperMigrationBackend:
    def __init__(
        self,
        controller: Any,
        grasp_module: ModuleType,
        place_module: ModuleType,
        *,
        readiness_timeout_s: float = 30.0,
        feedback_timeout_s: float = 2.0,
        min_pickup_opening_m: float = 0.002,
        min_drop_opening_m: float = 0.030,
        post_instruction_clear_delay_s: float = 0.0,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        self.controller = controller
        self.grasp_module = grasp_module
        self.place_module = place_module
        self.readiness_timeout_s = float(readiness_timeout_s)
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.min_pickup_opening_m = float(min_pickup_opening_m)
        self.min_drop_opening_m = float(min_drop_opening_m)
        self.post_instruction_clear_delay_s = float(
            post_instruction_clear_delay_s
        )
        if self.post_instruction_clear_delay_s < 0.0:
            raise ValueError(
                "post_instruction_clear_delay_s must be non-negative"
            )
        self._monotonic = monotonic
        self._sleep = sleep

    def initialize_transit_pose(self) -> None:
        self._wait_until_ready(require_object_model=False)
        gripper_m, _effort = self._fresh_gripper_feedback()
        self._return_to_transit_pose(
            gripper_m,
            "",
            required=True,
        )

    def pickup_once(
        self,
        target_hint: str,
        publish_phase: PhasePublisher,
    ) -> str:
        try:
            target_type, gripper_m = self._pickup_operation(
                target_hint,
                publish_phase,
            )
        except Exception:
            self._return_to_transit_pose(
                self._current_gripper_position(),
                self._current_target_type() or str(target_hint).strip(),
                required=False,
            )
            raise
        self._return_to_transit_pose(
            gripper_m,
            target_type,
            required=True,
        )
        return target_type

    def _pickup_operation(
        self,
        target_hint: str,
        publish_phase: PhasePublisher,
    ) -> tuple[str, float]:
        publish_phase(ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE, target_hint)
        self._wait_until_ready(
            require_object_model=True,
            heartbeat=lambda: publish_phase(
                ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE,
                target_hint,
            ),
        )
        if self.controller.move_to_observation_joint_pose() is False:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                "failed_to_reach_pickup_instruction_pose",
                target_hint,
            )

        target_type = str(target_hint).strip()
        recognized_instruction = not target_type
        if target_type:
            self._set_target_type(target_type)
        else:
            publish_phase(ArmTaskPhase.RECOGNIZING_INSTRUCTION, "")
            try:
                candidate = self.controller.select_target_from_instruction_sheet()
            except Exception as exc:
                self._fail(
                    ArmTaskOutcome.INSTRUCTION_NOT_FOUND,
                    f"instruction_recognition_failed: {exc}",
                )
            target_type = str(candidate.get("class_name", "")).strip()
            if not target_type:
                self._fail(
                    ArmTaskOutcome.INSTRUCTION_NOT_FOUND,
                    "instruction_recognition_returned_empty_target",
                )

        publish_phase(ArmTaskPhase.TARGET_TYPE_LOCKED, target_type)
        if (
            recognized_instruction
            and self.post_instruction_clear_delay_s > 0.0
        ):
            self._sleep(self.post_instruction_clear_delay_s)
        publish_phase(ArmTaskPhase.SEARCHING_TARGET_OBJECT, target_type)
        try:
            detection = self.controller.estimate_wrist_object_with_observation_scan()
        except Exception as exc:
            self._fail(
                ArmTaskOutcome.TARGET_NOT_FOUND,
                f"target_search_failed: {exc}",
                target_type,
            )
        self._remember_preview("last_wrist_preview", detection)

        publish_phase(ArmTaskPhase.OPERATING, target_type)
        self._perform_pickup(detection, target_type)
        publish_phase(ArmTaskPhase.VERIFYING_OPERATION, target_type)
        self._verify_pickup(target_type)
        closed_gripper = float(
            self.controller.get_grasp_config(
                self.controller.target_class_id
            )["gripper_closed"]
        )
        return target_type, closed_gripper

    def drop_once(
        self,
        target_type: str,
        publish_phase: PhasePublisher,
    ) -> None:
        try:
            gripper_m = self._drop_operation(target_type, publish_phase)
        except Exception:
            self._return_to_transit_pose(
                self._current_gripper_position(),
                str(target_type).strip(),
                required=False,
            )
            raise
        self._return_to_transit_pose(
            gripper_m,
            str(target_type).strip(),
            required=True,
        )

    def _drop_operation(
        self,
        target_type: str,
        publish_phase: PhasePublisher,
    ) -> float:
        target_type = str(target_type).strip()
        self._set_target_type(target_type)
        publish_phase(ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE, target_type)
        self._wait_until_ready(
            require_object_model=False,
            heartbeat=lambda: publish_phase(
                ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE,
                target_type,
            ),
        )

        closed_gripper = float(
            self.controller.get_grasp_config(
                self.controller.target_class_id
            )["gripper_closed"]
        )
        observation_joints = self.grasp_module.PLACE_OBSERVATION_JOINTS_RAD
        if self.controller.move_to_joint_pose(
            observation_joints,
            label="Competition DROP instruction pose",
            gripper_m=closed_gripper,
            timeout_s=20.0,
        ) is False:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                "failed_to_reach_drop_instruction_pose",
                target_type,
            )

        publish_phase(ArmTaskPhase.RECOGNIZING_INSTRUCTION, target_type)
        try:
            cached_selection = (
                self.place_module._select_target_sheet_candidate_with_scan(
                    self.controller,
                    target_type,
                    observation_joints,
                    closed_gripper,
                )
            )
        except Exception as exc:
            self._fail(
                ArmTaskOutcome.TARGET_NOT_FOUND,
                f"drop_instruction_not_found: {exc}",
                target_type,
            )
        if isinstance(cached_selection, tuple) and len(cached_selection) >= 5:
            self._remember_preview(
                "last_instruction_preview",
                cached_selection[4],
            )

        publish_phase(ArmTaskPhase.TARGET_TYPE_LOCKED, target_type)
        publish_phase(ArmTaskPhase.OPERATING, target_type)
        self._perform_drop(cached_selection, observation_joints, target_type)
        publish_phase(ArmTaskPhase.VERIFYING_OPERATION, target_type)
        return self._verify_drop(target_type)

    def _remember_preview(self, attribute: str, result: Any) -> None:
        if not isinstance(result, dict):
            return
        overlay = result.get("overlay")
        if overlay is None:
            return
        try:
            overlay = overlay.copy()
        except Exception:
            return
        setattr(self.controller, attribute, overlay)

    def _wait_until_ready(
        self,
        *,
        require_object_model: bool,
        heartbeat=None,
    ) -> None:
        deadline = self._monotonic() + max(0.0, self.readiness_timeout_s)
        while self._monotonic() < deadline:
            state_name = str(
                getattr(getattr(self.controller, "state", None), "name", "")
            )
            instruction = getattr(self.controller, "instruction_detector", None)
            object_detector = getattr(self.controller, "vision_detector", None)
            ready = (
                state_name == "IDLE"
                and bool(getattr(self.controller, "arm_enabled", False))
                and not bool(getattr(self.controller, "arm_faulted", False))
                and getattr(self.controller, "color_image", None) is not None
                and getattr(self.controller, "depth_image", None) is not None
                and getattr(self.controller, "camera_matrix", None) is not None
                and instruction is not None
                and bool(getattr(instruction, "is_loaded", False))
                and (
                    not require_object_model
                    or (
                        object_detector is not None
                        and bool(getattr(object_detector, "is_loaded", False))
                    )
                )
            )
            if ready:
                return
            if heartbeat is not None:
                heartbeat()
            self._sleep(0.10)
        self._fail(
            ArmTaskOutcome.OPERATION_FAILED,
            "piper_camera_or_models_not_ready",
            self._current_target_type(),
        )

    def _set_target_type(self, target_type: str) -> None:
        try:
            parsed = self.grasp_module.parse_target_spec(target_type)
            self.controller.set_target_class(
                parsed["class_id"],
                color=parsed.get("color"),
                model_class_name=parsed.get("model_class_name"),
                source="ArmTask target_type_hint",
            )
        except Exception as exc:
            self._fail(
                ArmTaskOutcome.INSTRUCTION_NOT_FOUND,
                f"unsupported_target_type_{target_type}: {exc}",
                target_type,
            )

    def _perform_pickup(self, detection: Any, target_type: str) -> None:
        original_estimator = (
            self.controller.estimate_wrist_object_with_observation_scan
        )
        self.controller.competition_grasp_failed = False
        self.controller.grasp_running = True
        self.controller.estimate_wrist_object_with_observation_scan = (
            lambda: detection
        )
        try:
            self.controller._execute_wrist_grasp_worker(
                skip_instruction_confirmation=True
            )
        finally:
            self.controller.estimate_wrist_object_with_observation_scan = (
                original_estimator
            )
            self.controller.grasp_running = False
        if bool(self.controller.competition_grasp_failed):
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                "legacy_grasp_worker_reported_failure",
                target_type,
            )

    def _perform_drop(
        self,
        cached_selection: Any,
        observation_joints: Any,
        target_type: str,
    ) -> None:
        original_selector = (
            self.place_module._select_target_sheet_candidate_with_scan
        )
        original_move_flag = self.place_module.PLACE_MOVE_OBSERVE_BEFORE_DETECT
        self.place_module._select_target_sheet_candidate_with_scan = (
            lambda *_args, **_kwargs: cached_selection
        )
        self.place_module.PLACE_MOVE_OBSERVE_BEFORE_DETECT = False
        try:
            success = self.place_module.execute_place_after_grasp(
                self.controller,
                self.grasp_module.R_CAM_TO_GRIPPER,
                self.grasp_module.T_CAM_TO_GRIPPER,
                observation_joints,
                self.grasp_module.solve_piper_ik_pose,
            )
        except Exception as exc:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                f"drop_operation_failed: {exc}",
                target_type,
            )
        finally:
            self.place_module._select_target_sheet_candidate_with_scan = (
                original_selector
            )
            self.place_module.PLACE_MOVE_OBSERVE_BEFORE_DETECT = (
                original_move_flag
            )
        if success is not True:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                "drop_operation_returned_false",
                target_type,
            )

    def _verify_pickup(self, target_type: str) -> float:
        position, effort = self._fresh_gripper_feedback()
        if position < self.min_pickup_opening_m:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                (
                    "pickup_not_verified: "
                    f"opening={position:.4f}m, effort={effort}"
                ),
                target_type,
            )
        return position

    def _verify_drop(self, target_type: str) -> float:
        position, effort = self._fresh_gripper_feedback()
        if position < self.min_drop_opening_m:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                (
                    "drop_not_verified: "
                    f"opening={position:.4f}m, effort={effort}"
                ),
                target_type,
            )
        return position

    def _return_to_transit_pose(
        self,
        gripper_m: float | None,
        target_type: str,
        *,
        required: bool,
    ) -> None:
        if gripper_m is None:
            detail = "cannot_reach_transit_pose_without_gripper_feedback"
            if required:
                self._fail(
                    ArmTaskOutcome.OPERATION_FAILED,
                    detail,
                    target_type,
                )
            return
        try:
            moved = self.controller.move_to_joint_pose(
                COMPETITION_TRANSIT_JOINTS_RAD,
                label="Competition transit pose",
                gripper_m=gripper_m,
                timeout_s=20.0,
            )
        except Exception as exc:
            if required:
                self._fail(
                    ArmTaskOutcome.OPERATION_FAILED,
                    f"failed_to_reach_transit_pose: {exc}",
                    target_type,
                )
            return
        if moved is False and required:
            self._fail(
                ArmTaskOutcome.OPERATION_FAILED,
                "failed_to_reach_transit_pose",
                target_type,
            )

    def _current_gripper_position(self) -> float | None:
        value = getattr(self.controller, "last_gripper_position_m", None)
        if value is None:
            return None
        try:
            position = float(value)
        except (TypeError, ValueError):
            return None
        return position if math.isfinite(position) else None

    def _fresh_gripper_feedback(self) -> tuple[float, float | None]:
        deadline = self._monotonic() + max(0.0, self.feedback_timeout_s)
        while self._monotonic() <= deadline:
            position = getattr(
                self.controller,
                "last_gripper_position_m",
                None,
            )
            received_at = getattr(
                self.controller,
                "last_gripper_feedback_at",
                None,
            )
            if (
                position is not None
                and received_at is not None
                and self._monotonic() - float(received_at) <= 1.0
            ):
                return (
                    float(position),
                    getattr(self.controller, "last_gripper_effort_nm", None),
                )
            self._sleep(0.05)
        self._fail(
            ArmTaskOutcome.OPERATION_FAILED,
            "missing_fresh_gripper_feedback",
            self._current_target_type(),
        )

    def _current_target_type(self) -> str:
        return str(
            getattr(self.controller, "target_model_class_name", "") or ""
        ).strip()

    @staticmethod
    def _fail(
        outcome: ArmTaskOutcome,
        detail: str,
        target_type: str = "",
    ) -> None:
        raise ArmExecutionFailure(
            outcome,
            detail,
            target_type=target_type,
        )
