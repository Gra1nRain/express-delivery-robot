#!/usr/bin/env python3
"""Coordinate startup and stopped-checkpoint anchor refinement."""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from competition_localization.planar_transform import PlanarTransform
from competition_localization.startup_alignment import (
    AlignmentConfig,
    AlignmentDecision,
    AlignmentMode,
    AlignmentObservation,
    AlignmentPhase,
    StartupAlignment,
)


class StartupAlignmentNode(Node):
    """Own the calibration handshake while the anchor node remains sole TF writer."""

    def __init__(self) -> None:
        super().__init__("startup_alignment")
        self._alignment = StartupAlignment(
            AlignmentConfig(
                required_samples=int(
                    self.declare_parameter("required_samples", 5).value
                ),
                verification_samples=int(
                    self.declare_parameter("verification_samples", 3).value
                ),
                max_sample_translation_spread_m=float(
                    self.declare_parameter(
                        "max_sample_translation_spread_m", 0.05
                    ).value
                ),
                max_sample_yaw_spread_rad=math.radians(
                    float(
                        self.declare_parameter("max_sample_yaw_spread_deg", 1.0).value
                    )
                ),
                min_inlier_ratio=float(
                    self.declare_parameter("min_inlier_ratio", 0.60).value
                ),
                max_best_median_residual_m=float(
                    self.declare_parameter("max_best_median_residual_m", 0.05).value
                ),
                max_startup_translation_m=float(
                    self.declare_parameter("max_startup_translation_m", 0.50).value
                ),
                max_startup_yaw_rad=math.radians(
                    float(self.declare_parameter("max_startup_yaw_deg", 10.0).value)
                ),
                max_checkpoint_translation_m=float(
                    self.declare_parameter("max_checkpoint_translation_m", 0.20).value
                ),
                max_checkpoint_yaw_rad=math.radians(
                    float(self.declare_parameter("max_checkpoint_yaw_deg", 5.0).value)
                ),
                verification_translation_m=float(
                    self.declare_parameter("verification_translation_m", 0.03).value
                ),
                verification_yaw_rad=math.radians(
                    float(self.declare_parameter("verification_yaw_deg", 0.75).value)
                ),
            )
        )
        self._decision = AlignmentDecision(
            phase=AlignmentPhase.WAITING,
            mode=None,
            reference=None,
            reason="waiting for coarse anchor",
        )
        self._anchor_revision: int | None = None
        self._route_enabled = False
        self._startup_ready = False
        self._checkpoint_ready_ref: str | None = None
        self._checkpoint_hold = False
        self._pending_request_id: str | None = None
        self._pending_operation: str | None = None
        self._request_sequence = 0

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic", "/localization/alignment_status"
                ).value
            ),
            status_qos,
        )
        self._anchor_request_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "anchor_update_request_topic",
                    "/localization/anchor_update_request",
                ).value
            ),
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "anchor_status_topic", "/localization/anchor_status"
                ).value
            ),
            self._anchor_status_callback,
            status_qos,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "residual_topic", "/localization/scan_map_residual"
                ).value
            ),
            self._residual_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "route_enable_topic", "/mission/route_enable"
                ).value
            ),
            self._route_enable_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "checkpoint_request_topic",
                    "/localization/checkpoint_alignment_request",
                ).value
            ),
            self._checkpoint_request_callback,
            10,
        )
        self._timer = self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            "Startup alignment ready (stationary one-shot correction only)"
        )

    def _anchor_status_callback(self, message: String) -> None:
        status = self._decode(message.data)
        if status is None:
            return
        try:
            revision = int(status["revision"])
        except (KeyError, TypeError, ValueError):
            return
        self._anchor_revision = revision
        if status.get("source") == "coarse" and status.get("applied"):
            self._startup_ready = False
            self._checkpoint_ready_ref = None
            self._checkpoint_hold = False
            self._pending_request_id = None
            self._pending_operation = None
            self._decision = self._alignment.begin(
                mode=AlignmentMode.STARTUP,
                reference="startup",
                anchor_revision=revision,
            )
            self._publish_status()
            return
        request_id = status.get("request_id")
        if request_id != self._pending_request_id:
            return
        operation = self._pending_operation
        self._pending_request_id = None
        self._pending_operation = None
        if not status.get("applied"):
            self._decision = self._alignment.reject(
                f"anchor writer rejected {operation}: {status.get('reason')}"
            )
        elif operation == "apply":
            self._decision = self._alignment.applied(new_anchor_revision=revision)
        elif operation == "rollback":
            self._decision = self._alignment.reject(
                "anchor correction failed verification and was rolled back"
            )
        self._publish_status()

    def _residual_callback(self, message: String) -> None:
        status = self._decode(message.data)
        if status is None:
            return
        try:
            observation = AlignmentObservation(
                stamp_s=float(status["stamp_s"]),
                stationary=bool(status["stationary"]),
                confident=bool(status["confident"]),
                search_boundary_hit=bool(status["search_boundary_hit"]),
                correction=PlanarTransform(
                    x=float(status["anchor_correction_x_m"]),
                    y=float(status["anchor_correction_y_m"]),
                    yaw=float(status["anchor_correction_yaw_rad"]),
                ),
                best_median_residual_m=float(status["best_median_residual_m"]),
                inlier_ratio=float(status["inlier_ratio"]),
                residual_correction=PlanarTransform(
                    x=float(status["correction_x_m"]),
                    y=float(status["correction_y_m"]),
                    yaw=float(status["correction_yaw_rad"]),
                ),
            )
        except (KeyError, TypeError, ValueError):
            return
        self._decision = self._alignment.observe(observation)
        self._handle_decision()

    def _checkpoint_request_callback(self, message: String) -> None:
        reference = message.data.strip()
        if (
            not reference
            or not self._startup_ready
            or not self._route_enabled
            or self._anchor_revision is None
            or reference == self._checkpoint_ready_ref
        ):
            return
        if (
            self._decision.mode == AlignmentMode.CHECKPOINT
            and self._decision.reference == reference
            and self._decision.phase
            not in (AlignmentPhase.READY, AlignmentPhase.REJECTED)
        ):
            return
        self._checkpoint_ready_ref = None
        self._checkpoint_hold = True
        self._decision = self._alignment.begin(
            mode=AlignmentMode.CHECKPOINT,
            reference=reference,
            anchor_revision=self._anchor_revision,
        )
        self._publish_status()

    def _route_enable_callback(self, message: Bool) -> None:
        self._route_enabled = bool(message.data)
        if (
            self._route_enabled
            and self._startup_ready
            and self._decision.mode == AlignmentMode.STARTUP
            and self._decision.phase == AlignmentPhase.READY
        ):
            self._decision = self._alignment.lock("startup anchor locked for route")
        if not self._route_enabled:
            self._checkpoint_hold = False
            self._checkpoint_ready_ref = None
        self._publish_status()

    def _handle_decision(self) -> None:
        if (
            self._decision.phase == AlignmentPhase.APPLYING
            and self._decision.correction is not None
            and self._pending_request_id is None
        ):
            self._publish_anchor_request("apply")
        elif (
            self._decision.phase == AlignmentPhase.REJECTED
            and self._decision.rollback_required
            and self._pending_request_id is None
        ):
            self._publish_anchor_request("rollback")
        elif self._decision.phase == AlignmentPhase.READY:
            if self._decision.mode == AlignmentMode.STARTUP:
                self._startup_ready = True
            elif self._decision.reference is not None:
                self._checkpoint_ready_ref = self._decision.reference
                self._checkpoint_hold = False
        self._publish_status()

    def _publish_anchor_request(self, operation: str) -> None:
        if self._anchor_revision is None:
            self._decision = self._alignment.reject("anchor revision unavailable")
            return
        self._request_sequence += 1
        request_id = f"alignment-{self._request_sequence}"
        request: dict[str, object] = {
            "request_id": request_id,
            "operation": operation,
            "expected_revision": self._anchor_revision,
            "mode": (
                self._decision.mode.value if self._decision.mode is not None else None
            ),
            "reference": self._decision.reference,
        }
        if operation == "apply":
            assert self._decision.correction is not None
            assert self._decision.residual_correction is not None
            request.update(
                {
                    "correction_x_m": self._decision.correction.x,
                    "correction_y_m": self._decision.correction.y,
                    "correction_yaw_rad": self._decision.correction.yaw,
                    "displacement_x_m": self._decision.residual_correction.x,
                    "displacement_y_m": self._decision.residual_correction.y,
                    "displacement_yaw_rad": (self._decision.residual_correction.yaw),
                }
            )
        self._pending_request_id = request_id
        self._pending_operation = operation
        self._anchor_request_publisher.publish(
            String(data=json.dumps(request, separators=(",", ":"), allow_nan=False))
        )

    def _publish_status(self) -> None:
        status = {
            "phase": self._decision.phase.value,
            "mode": (
                self._decision.mode.value if self._decision.mode is not None else None
            ),
            "reference": self._decision.reference,
            "reason": self._decision.reason,
            "anchor_revision": self._anchor_revision,
            "startup_ready": self._startup_ready,
            "checkpoint_ready_ref": self._checkpoint_ready_ref,
            "checkpoint_hold": self._checkpoint_hold,
            "route_enabled": self._route_enabled,
            "pending_request_id": self._pending_request_id,
        }
        self._status_publisher.publish(
            String(data=json.dumps(status, separators=(",", ":"), allow_nan=False))
        )

    @staticmethod
    def _decode(encoded: str) -> dict[str, object] | None:
        try:
            value = json.loads(encoded)
        except (json.JSONDecodeError, TypeError):
            return None
        return value if isinstance(value, dict) else None


def main() -> None:
    rclpy.init()
    node = StartupAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
