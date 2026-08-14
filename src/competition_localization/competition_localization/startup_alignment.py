"""Pure startup and stopped-checkpoint scan-to-map alignment policy."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import statistics

from competition_localization.planar_transform import PlanarTransform, wrap_angle


class AlignmentMode(str, Enum):
    STARTUP = "STARTUP"
    CHECKPOINT = "CHECKPOINT"


class AlignmentPhase(str, Enum):
    WAITING = "WAITING"
    COLLECTING = "COLLECTING"
    APPLYING = "APPLYING"
    VERIFYING = "VERIFYING"
    READY = "READY"
    REJECTED = "REJECTED"
    LOCKED = "LOCKED"


@dataclass(frozen=True)
class AlignmentConfig:
    required_samples: int = 5
    verification_samples: int = 3
    max_sample_translation_spread_m: float = 0.05
    max_sample_yaw_spread_rad: float = math.radians(1.0)
    min_inlier_ratio: float = 0.60
    max_best_median_residual_m: float = 0.05
    max_acceptable_baseline_median_m: float = 0.08
    min_acceptable_baseline_inlier_ratio: float = 0.50
    max_startup_translation_m: float = 0.50
    max_startup_yaw_rad: float = math.radians(10.0)
    max_checkpoint_translation_m: float = 0.20
    max_checkpoint_yaw_rad: float = math.radians(5.0)
    verification_translation_m: float = 0.03
    verification_yaw_rad: float = math.radians(0.75)


@dataclass(frozen=True)
class AlignmentObservation:
    stamp_s: float
    stationary: bool
    confident: bool
    search_boundary_hit: bool
    correction: PlanarTransform
    best_median_residual_m: float
    inlier_ratio: float
    residual_correction: PlanarTransform | None = None
    baseline_median_residual_m: float = math.inf
    baseline_inlier_ratio: float = 0.0


@dataclass(frozen=True)
class AlignmentDecision:
    phase: AlignmentPhase
    mode: AlignmentMode | None
    reference: str | None
    reason: str
    expected_anchor_revision: int | None = None
    correction: PlanarTransform | None = None
    residual_correction: PlanarTransform | None = None
    rollback_required: bool = False


class StartupAlignment:
    """Turn trusted stationary residuals into one bounded anchor correction."""

    def __init__(self, config: AlignmentConfig = AlignmentConfig()) -> None:
        self._config = config
        self._phase = AlignmentPhase.WAITING
        self._mode: AlignmentMode | None = None
        self._reference: str | None = None
        self._anchor_revision: int | None = None
        self._samples: deque[AlignmentObservation] = deque(
            maxlen=max(config.required_samples, config.verification_samples)
        )

    def begin(
        self,
        *,
        mode: AlignmentMode,
        reference: str,
        anchor_revision: int,
    ) -> AlignmentDecision:
        self._mode = mode
        self._reference = reference
        self._anchor_revision = anchor_revision
        self._samples.clear()
        self._phase = AlignmentPhase.COLLECTING
        return self._decision("collecting stationary scan-map evidence")

    def observe(self, observation: AlignmentObservation) -> AlignmentDecision:
        if self._phase not in (
            AlignmentPhase.COLLECTING,
            AlignmentPhase.VERIFYING,
        ):
            return self._decision("observation ignored outside collection")
        if not self._usable(observation):
            self._samples.clear()
            return self._decision("unusable alignment observation")

        self._samples.append(observation)
        required = (
            self._config.required_samples
            if self._phase == AlignmentPhase.COLLECTING
            else self._config.verification_samples
        )
        if len(self._samples) < required:
            return self._decision("collecting stationary scan-map evidence")

        correction = self._median_correction(use_residual=False)
        residual_correction = self._median_correction(use_residual=True)
        if not self._stable(residual_correction):
            return self._decision("alignment correction is not stable")
        if self._phase == AlignmentPhase.VERIFYING:
            if (
                math.hypot(residual_correction.x, residual_correction.y)
                <= self._config.verification_translation_m
                and abs(residual_correction.yaw) <= self._config.verification_yaw_rad
            ):
                self._phase = AlignmentPhase.READY
                return self._decision("anchor correction verified")
            self._phase = AlignmentPhase.REJECTED
            return self._decision(
                "anchor correction failed verification",
                rollback_required=True,
            )
        if not self._within_mode_limit(residual_correction):
            self._phase = AlignmentPhase.REJECTED
            return self._decision("alignment correction exceeds safe limit")

        self._phase = AlignmentPhase.APPLYING
        return self._decision(
            "stable alignment correction ready",
            correction=correction,
            residual_correction=residual_correction,
        )

    def applied(self, *, new_anchor_revision: int) -> AlignmentDecision:
        if self._phase != AlignmentPhase.APPLYING:
            return self._decision("anchor application ignored outside applying phase")
        if (
            self._anchor_revision is None
            or new_anchor_revision <= self._anchor_revision
        ):
            self._phase = AlignmentPhase.REJECTED
            return self._decision("anchor revision did not advance")
        self._anchor_revision = new_anchor_revision
        self._samples.clear()
        self._phase = AlignmentPhase.VERIFYING
        return self._decision("verifying applied anchor correction")

    def lock(self, reason: str) -> AlignmentDecision:
        self._samples.clear()
        self._phase = AlignmentPhase.LOCKED
        return self._decision(reason)

    def reject(self, reason: str) -> AlignmentDecision:
        self._samples.clear()
        self._phase = AlignmentPhase.REJECTED
        return self._decision(reason)

    def _usable(self, observation: AlignmentObservation) -> bool:
        common = observation.stationary and observation.correction.is_finite()
        if not common:
            return False
        if self._coarse_anchor_acceptable(observation):
            return True
        return (
            observation.confident
            and not observation.search_boundary_hit
            and observation.best_median_residual_m
            <= self._config.max_best_median_residual_m
            and observation.inlier_ratio >= self._config.min_inlier_ratio
        )

    def _median_correction(self, *, use_residual: bool) -> PlanarTransform:
        corrections = [
            self._sample_correction(sample, use_residual=use_residual)
            for sample in self._samples
        ]
        first_yaw = corrections[0].yaw
        yaw_offsets = [
            wrap_angle(correction.yaw - first_yaw) for correction in corrections
        ]
        return PlanarTransform(
            x=statistics.median(correction.x for correction in corrections),
            y=statistics.median(correction.y for correction in corrections),
            yaw=wrap_angle(first_yaw + statistics.median(yaw_offsets)),
        )

    def _stable(self, correction: PlanarTransform) -> bool:
        return all(
            math.hypot(
                self._sample_correction(sample, use_residual=True).x - correction.x,
                self._sample_correction(sample, use_residual=True).y - correction.y,
            )
            <= self._config.max_sample_translation_spread_m
            and abs(
                wrap_angle(
                    self._sample_correction(sample, use_residual=True).yaw
                    - correction.yaw
                )
            )
            <= self._config.max_sample_yaw_spread_rad
            for sample in self._samples
        )

    def _sample_correction(
        self,
        sample: AlignmentObservation,
        *,
        use_residual: bool,
    ) -> PlanarTransform:
        if self._coarse_anchor_acceptable(sample):
            return PlanarTransform(0.0, 0.0, 0.0)
        if use_residual and sample.residual_correction is not None:
            return sample.residual_correction
        return sample.correction

    def _coarse_anchor_acceptable(
        self,
        observation: AlignmentObservation,
    ) -> bool:
        return (
            observation.baseline_median_residual_m
            <= self._config.max_acceptable_baseline_median_m
            and observation.baseline_inlier_ratio
            >= self._config.min_acceptable_baseline_inlier_ratio
        )

    def _within_mode_limit(self, correction: PlanarTransform) -> bool:
        assert self._mode is not None
        if self._mode == AlignmentMode.STARTUP:
            translation_limit = self._config.max_startup_translation_m
            yaw_limit = self._config.max_startup_yaw_rad
        else:
            translation_limit = self._config.max_checkpoint_translation_m
            yaw_limit = self._config.max_checkpoint_yaw_rad
        return (
            math.hypot(correction.x, correction.y) <= translation_limit
            and abs(correction.yaw) <= yaw_limit
        )

    def _decision(
        self,
        reason: str,
        *,
        correction: PlanarTransform | None = None,
        residual_correction: PlanarTransform | None = None,
        rollback_required: bool = False,
    ) -> AlignmentDecision:
        return AlignmentDecision(
            phase=self._phase,
            mode=self._mode,
            reference=self._reference,
            reason=reason,
            expected_anchor_revision=self._anchor_revision,
            correction=correction,
            residual_correction=residual_correction,
            rollback_required=rollback_required,
        )
