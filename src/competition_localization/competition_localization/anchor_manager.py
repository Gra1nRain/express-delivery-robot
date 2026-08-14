"""Versioned ownership of the fixed map-to-FAST-LIO anchor."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from competition_localization.planar_transform import PlanarTransform


class AnchorCorrectionMode(str, Enum):
    STARTUP = "STARTUP"
    CHECKPOINT = "CHECKPOINT"


@dataclass(frozen=True)
class AnchorSafetyState:
    stationary: bool
    route_enabled: bool
    checkpoint_hold: bool


@dataclass(frozen=True)
class AnchorLimits:
    startup_translation_m: float = 0.50
    startup_yaw_rad: float = math.radians(10.0)
    checkpoint_translation_m: float = 0.20
    checkpoint_yaw_rad: float = math.radians(5.0)


@dataclass(frozen=True)
class AnchorUpdate:
    applied: bool
    reason: str
    revision: int
    transform: PlanarTransform | None


class AnchorManager:
    """Apply coarse anchors and compare-and-swap refinements at one seam."""

    def __init__(self, limits: AnchorLimits = AnchorLimits()) -> None:
        self._limits = limits
        self._revision = 0
        self._transform: PlanarTransform | None = None
        self._rollback_transform: PlanarTransform | None = None
        self._rollback_mode: AnchorCorrectionMode | None = None

    @property
    def revision(self) -> int:
        return self._revision

    @property
    def transform(self) -> PlanarTransform | None:
        return self._transform

    def set_coarse_anchor(
        self,
        *,
        target_map_base: PlanarTransform,
        odom_to_base: PlanarTransform,
        stationary: bool,
    ) -> AnchorUpdate:
        if not stationary:
            return self._result(False, "vehicle must be stationary for coarse anchor")
        if not target_map_base.is_finite() or not odom_to_base.is_finite():
            return self._result(False, "anchor inputs must be finite")
        self._transform = target_map_base.compose(odom_to_base.inverse())
        self._revision += 1
        self._rollback_transform = None
        self._rollback_mode = None
        return self._result(True, "coarse anchor applied")

    def apply_correction(
        self,
        *,
        correction: PlanarTransform,
        displacement_correction: PlanarTransform | None = None,
        expected_revision: int,
        mode: AnchorCorrectionMode,
        safety: AnchorSafetyState,
    ) -> AnchorUpdate:
        rejection = self._correction_rejection(
            correction=correction,
            displacement_correction=(displacement_correction or correction),
            expected_revision=expected_revision,
            mode=mode,
            safety=safety,
        )
        if rejection is not None:
            return self._result(False, rejection)
        assert self._transform is not None
        self._rollback_transform = self._transform
        self._rollback_mode = mode
        self._transform = correction.compose(self._transform)
        self._revision += 1
        return self._result(True, "anchor correction applied")

    def rollback(
        self,
        *,
        expected_revision: int,
        safety: AnchorSafetyState,
    ) -> AnchorUpdate:
        if expected_revision != self._revision:
            return self._result(False, "anchor revision mismatch")
        if self._rollback_transform is None or self._rollback_mode is None:
            return self._result(False, "no anchor correction is available to roll back")
        if not self._safety_allows(self._rollback_mode, safety):
            return self._result(False, "anchor rollback is not safe in current state")
        self._transform = self._rollback_transform
        self._revision += 1
        self._rollback_transform = None
        self._rollback_mode = None
        return self._result(True, "anchor correction rolled back")

    def _correction_rejection(
        self,
        *,
        correction: PlanarTransform,
        displacement_correction: PlanarTransform,
        expected_revision: int,
        mode: AnchorCorrectionMode,
        safety: AnchorSafetyState,
    ) -> str | None:
        if self._transform is None:
            return "coarse anchor is not available"
        if expected_revision != self._revision:
            return "anchor revision mismatch"
        if not correction.is_finite() or not displacement_correction.is_finite():
            return "anchor correction must be finite"
        if not self._safety_allows(mode, safety):
            return "anchor correction is not safe in current state"
        if mode == AnchorCorrectionMode.STARTUP:
            translation_limit = self._limits.startup_translation_m
            yaw_limit = self._limits.startup_yaw_rad
        else:
            translation_limit = self._limits.checkpoint_translation_m
            yaw_limit = self._limits.checkpoint_yaw_rad
        if (
            math.hypot(displacement_correction.x, displacement_correction.y)
            > translation_limit
        ):
            return "anchor correction translation exceeds hard limit"
        if abs(displacement_correction.yaw) > yaw_limit:
            return "anchor correction yaw exceeds hard limit"
        return None

    @staticmethod
    def _safety_allows(
        mode: AnchorCorrectionMode,
        safety: AnchorSafetyState,
    ) -> bool:
        if not safety.stationary:
            return False
        if mode == AnchorCorrectionMode.STARTUP:
            return not safety.route_enabled
        return safety.checkpoint_hold

    def _result(self, applied: bool, reason: str) -> AnchorUpdate:
        return AnchorUpdate(
            applied=applied,
            reason=reason,
            revision=self._revision,
            transform=self._transform,
        )
