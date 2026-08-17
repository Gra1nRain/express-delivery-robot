"""Pure state machines for the start flag and traffic-light rules."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from statistics import mean


class WaveState(str, Enum):
    IDLE = "idle"
    TRACKING = "tracking"
    READY = "ready"
    TRIGGERED = "triggered"


@dataclass(frozen=True)
class WaveConfig:
    trajectory_window: int = 20
    min_downward_displacement_px: float = 40.0
    direct_min_speed_pxps: float = 80.0
    ready_min_speed_pxps: float = 50.0
    prepare_min_displacement_px: float = -15.0
    min_total_travel_px: float = 100.0
    cooldown_s: float = 2.0
    max_lost_frames: int = 3


class FlagWaveDetector:
    """Detect one downward flag wave from a stream of vertical centroids."""

    def __init__(self, config: WaveConfig | None = None) -> None:
        self.config = config or WaveConfig()
        if self.config.trajectory_window < 5:
            raise ValueError("trajectory_window must be at least 5")
        self.state = WaveState.IDLE
        self.trajectory: deque[tuple[float, float]] = deque(
            maxlen=self.config.trajectory_window
        )
        self.last_trigger_s = float("-inf")
        self.lost_frames = 0
        self.peak_high_y: float | None = None
        self.peak_low_y: float | None = None

    def update(
        self,
        *,
        centroid_y: float | None,
        timestamp_s: float,
    ) -> bool:
        if centroid_y is None:
            self.lost_frames += 1
            if self.lost_frames > self.config.max_lost_frames:
                self._clear_tracking(WaveState.IDLE)
            return False

        self.lost_frames = 0
        self.trajectory.append((timestamp_s, float(centroid_y)))
        self.peak_high_y = (
            float(centroid_y)
            if self.peak_high_y is None
            else min(self.peak_high_y, float(centroid_y))
        )
        self.peak_low_y = (
            float(centroid_y)
            if self.peak_low_y is None
            else max(self.peak_low_y, float(centroid_y))
        )

        if len(self.trajectory) < 5:
            if self.state == WaveState.IDLE:
                self.state = WaveState.TRACKING
            return False
        if timestamp_s - self.last_trigger_s < self.config.cooldown_s:
            return False

        samples = list(self.trajectory)
        sample_count = len(samples)
        third = max(3, sample_count // 3)
        midpoint = sample_count // 2
        early_y = mean(point[1] for point in samples[:midpoint])
        late_y = mean(point[1] for point in samples[midpoint:])
        recent_y = mean(point[1] for point in samples[-third:])
        before_y = mean(point[1] for point in samples[:third])
        full_dt = samples[-1][0] - samples[0][0]
        short_dt = samples[-1][0] - samples[third][0]
        if full_dt < 0.03 or short_dt <= 0.03:
            return False

        full_displacement = late_y - early_y
        short_displacement = recent_y - before_y
        full_speed = full_displacement / full_dt
        short_speed = short_displacement / short_dt
        total_travel = (
            self.peak_low_y - self.peak_high_y
            if self.peak_low_y is not None and self.peak_high_y is not None
            else 0.0
        )

        if self.state in (WaveState.IDLE, WaveState.TRACKING):
            if self._is_downward_wave(
                short_displacement,
                short_speed,
                total_travel,
                self.config.direct_min_speed_pxps,
            ):
                return self._trigger(timestamp_s)
            if (
                full_displacement < self.config.prepare_min_displacement_px
                and abs(full_speed) > self.config.direct_min_speed_pxps * 0.4
            ):
                self.state = WaveState.READY
        elif self.state == WaveState.READY and self._is_downward_wave(
            short_displacement,
            short_speed,
            total_travel,
            self.config.ready_min_speed_pxps,
        ):
            return self._trigger(timestamp_s)
        elif self.state == WaveState.TRIGGERED:
            self.state = WaveState.TRACKING
        return False

    def _is_downward_wave(
        self,
        displacement_px: float,
        speed_pxps: float,
        total_travel_px: float,
        min_speed_pxps: float,
    ) -> bool:
        return bool(
            displacement_px > self.config.min_downward_displacement_px
            and speed_pxps > min_speed_pxps
            and total_travel_px >= self.config.min_total_travel_px
        )

    def _trigger(self, timestamp_s: float) -> bool:
        self.state = WaveState.TRIGGERED
        self.last_trigger_s = timestamp_s
        self.trajectory.clear()
        self.peak_high_y = None
        self.peak_low_y = None
        return True

    def _clear_tracking(self, state: WaveState) -> None:
        self.state = state
        self.trajectory.clear()
        self.peak_high_y = None
        self.peak_low_y = None


class LightState(str, Enum):
    UNKNOWN = "unknown"
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    OFF = "off"


@dataclass(frozen=True)
class TrafficDecision:
    started: bool
    light: LightState
    stop_required: bool
    reason: str


class TrafficRuleController:
    """Latch the flag start, then enforce stable traffic-light observations."""

    def __init__(self, confirm_frames: int = 3) -> None:
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be positive")
        self._confirm_frames = confirm_frames
        self._started = False
        self._light = LightState.UNKNOWN
        self._candidate = LightState.UNKNOWN
        self._candidate_count = 0
        self._stop_required = True
        self._reason = "waiting_for_flag"

    @property
    def decision(self) -> TrafficDecision:
        return TrafficDecision(
            started=self._started,
            light=self._light,
            stop_required=self._stop_required,
            reason=self._reason,
        )

    def observe_flag_wave(self) -> TrafficDecision:
        self._started = True
        if self._light in (LightState.RED, LightState.YELLOW, LightState.OFF):
            self._stop_required = True
            self._reason = f"traffic_{self._light.value}"
        else:
            self._stop_required = False
            self._reason = "flag_start"
        return self.decision

    def observe_light(self, class_name: str | None) -> TrafficDecision:
        observation = _light_state(class_name)
        if observation == LightState.UNKNOWN:
            self._candidate = LightState.UNKNOWN
            self._candidate_count = 0
            return self.decision
        if observation == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = observation
            self._candidate_count = 1
        if self._candidate_count < self._confirm_frames:
            return self.decision

        self._light = observation
        if not self._started:
            self._stop_required = True
            self._reason = "waiting_for_flag"
        elif observation == LightState.GREEN:
            self._stop_required = False
            self._reason = "traffic_green"
        else:
            self._stop_required = True
            self._reason = f"traffic_{observation.value}"
        return self.decision


def _light_state(class_name: str | None) -> LightState:
    normalized = str(class_name or "").strip().lower()
    try:
        return LightState(normalized)
    except ValueError:
        return LightState.UNKNOWN
