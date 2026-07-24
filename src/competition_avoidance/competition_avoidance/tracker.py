"""Timestamp-aware nearest-neighbour obstacle tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math

from competition_avoidance.perception import ObstacleDetection


@dataclass(frozen=True)
class TrackerConfig:
    association_gate_m: float = 0.80
    track_timeout_s: float = 0.80
    minimum_confirmed_hits: int = 2
    moving_speed_mps: float = 0.20
    static_speed_mps: float = 0.08
    moving_confirmation_count: int = 2
    static_confirmation_count: int = 3
    maximum_unknown_dynamic_radius_m: float = 0.80
    alpha: float = 0.85
    beta: float = 0.45

    def __post_init__(self) -> None:
        if self.association_gate_m <= 0.0 or self.track_timeout_s <= 0.0:
            raise ValueError("association gate and timeout must be positive")
        if self.minimum_confirmed_hits < 1:
            raise ValueError("minimum_confirmed_hits must be positive")
        if self.static_speed_mps < 0.0:
            raise ValueError("static_speed_mps must be non-negative")
        if self.moving_speed_mps <= self.static_speed_mps:
            raise ValueError("moving threshold must exceed static threshold")
        if self.maximum_unknown_dynamic_radius_m <= 0.0:
            raise ValueError("maximum unknown dynamic radius must be positive")
        if not (0.0 < self.alpha <= 1.0 and 0.0 < self.beta <= 1.0):
            raise ValueError("alpha and beta must be in (0, 1]")


@dataclass(frozen=True)
class TrackedObstacle:
    track_id: int
    x: float
    y: float
    vx_mps: float
    vy_mps: float
    radius_m: float
    classification: str
    confidence: float
    motion_state: str
    confirmed: bool
    age_s: float
    last_seen_s: float

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.vx_mps, self.vy_mps)


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    vx_mps: float
    vy_mps: float
    radius_m: float
    classification: str
    confidence: float
    created_s: float
    timestamp_s: float
    last_seen_s: float
    hits: int = 1
    moving_streak: int = 0
    static_streak: int = 0
    motion_state: str = "UNKNOWN"


class ObstacleTracker:
    """Track detections in one fixed world frame."""

    def __init__(self, config: TrackerConfig = TrackerConfig()) -> None:
        self._config = config
        self._tracks: dict[int, _Track] = {}
        self._next_track_id = 1
        self._last_timestamp_s: float | None = None

    def update(
        self,
        detections: tuple[ObstacleDetection, ...],
        *,
        timestamp_s: float,
    ) -> tuple[TrackedObstacle, ...]:
        timestamp = float(timestamp_s)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp_s must be finite")
        if self._last_timestamp_s is not None and timestamp <= self._last_timestamp_s:
            raise ValueError("tracker timestamps must increase")
        self._last_timestamp_s = timestamp

        predictions = {
            track_id: self._predict(track, timestamp)
            for track_id, track in self._tracks.items()
        }
        candidates: list[tuple[float, int, int]] = []
        for track_id, (predicted_x, predicted_y) in predictions.items():
            for detection_index, detection in enumerate(detections):
                distance = math.hypot(
                    detection.x - predicted_x,
                    detection.y - predicted_y,
                )
                if distance <= self._config.association_gate_m:
                    candidates.append((distance, track_id, detection_index))
        candidates.sort()

        assigned_tracks: set[int] = set()
        assigned_detections: set[int] = set()
        for _, track_id, detection_index in candidates:
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            self._update_track(
                self._tracks[track_id],
                detections[detection_index],
                timestamp,
            )
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)

        for track_id, track in tuple(self._tracks.items()):
            if track_id not in assigned_tracks:
                track.x, track.y = predictions[track_id]
                track.timestamp_s = timestamp
            if timestamp - track.last_seen_s > self._config.track_timeout_s:
                del self._tracks[track_id]

        for detection_index, detection in enumerate(detections):
            if detection_index in assigned_detections:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = _Track(
                track_id=track_id,
                x=detection.x,
                y=detection.y,
                vx_mps=0.0,
                vy_mps=0.0,
                radius_m=max(0.05, detection.radius_m),
                classification=detection.classification,
                confidence=detection.confidence,
                created_s=timestamp,
                timestamp_s=timestamp,
                last_seen_s=timestamp,
                static_streak=1,
            )

        return tuple(
            self._snapshot(track, timestamp)
            for track in sorted(self._tracks.values(), key=lambda item: item.track_id)
        )

    @staticmethod
    def _predict(track: _Track, timestamp_s: float) -> tuple[float, float]:
        dt = timestamp_s - track.timestamp_s
        return (
            track.x + track.vx_mps * dt,
            track.y + track.vy_mps * dt,
        )

    def _update_track(
        self,
        track: _Track,
        detection: ObstacleDetection,
        timestamp_s: float,
    ) -> None:
        dt = timestamp_s - track.timestamp_s
        predicted_x, predicted_y = self._predict(track, timestamp_s)
        residual_x = detection.x - predicted_x
        residual_y = detection.y - predicted_y
        track.x = predicted_x + self._config.alpha * residual_x
        track.y = predicted_y + self._config.alpha * residual_y
        track.vx_mps += self._config.beta * residual_x / dt
        track.vy_mps += self._config.beta * residual_y / dt
        track.radius_m = max(0.05, detection.radius_m)
        track.classification = detection.classification
        track.confidence = detection.confidence
        track.timestamp_s = timestamp_s
        track.last_seen_s = timestamp_s
        track.hits += 1
        self._update_motion_state(track)

    def _update_motion_state(self, track: _Track) -> None:
        if (
            track.classification == "CONE_CANDIDATE"
            or (
                track.classification == "UNKNOWN"
                and track.radius_m
                > self._config.maximum_unknown_dynamic_radius_m
            )
        ):
            track.moving_streak = 0
            track.static_streak += 1
            if track.static_streak >= self._config.static_confirmation_count:
                track.motion_state = "STATIC"
            return

        speed = math.hypot(track.vx_mps, track.vy_mps)
        if speed >= self._config.moving_speed_mps:
            track.moving_streak += 1
            track.static_streak = 0
            if track.moving_streak >= self._config.moving_confirmation_count:
                track.motion_state = "DYNAMIC"
        elif speed <= self._config.static_speed_mps:
            track.static_streak += 1
            track.moving_streak = 0
            if track.static_streak >= self._config.static_confirmation_count:
                track.motion_state = "STATIC"
        else:
            track.moving_streak = 0
            track.static_streak = 0

    def _snapshot(self, track: _Track, timestamp_s: float) -> TrackedObstacle:
        return TrackedObstacle(
            track_id=track.track_id,
            x=track.x,
            y=track.y,
            vx_mps=track.vx_mps,
            vy_mps=track.vy_mps,
            radius_m=track.radius_m,
            classification=track.classification,
            confidence=track.confidence,
            motion_state=track.motion_state,
            confirmed=track.hits >= self._config.minimum_confirmed_hits,
            age_s=timestamp_s - track.created_s,
            last_seen_s=track.last_seen_s,
        )
