#!/usr/bin/env python3
"""Run on-demand bright-spot traffic-light recognition on the wrist RGB topic."""

from __future__ import annotations

import json
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from competition_perception.light_spot_detector import (
    LightSpotConfig,
    LightSpotDetector,
)


def _event_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TrafficLightRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_recognition")
        self._inference_period_s = float(
            self.declare_parameter("inference_period_s", 0.10).value
        )
        if self._inference_period_s < 0.0:
            raise ValueError("inference_period_s must be non-negative")

        self._detector = LightSpotDetector(
            LightSpotConfig(
                brightness_threshold=int(
                    self.declare_parameter("brightness_threshold", 100).value
                ),
                min_area_px=float(
                    self.declare_parameter("min_spot_area_px", 30.0).value
                ),
                max_area_px=float(
                    self.declare_parameter("max_spot_area_px", 8000.0).value
                ),
                min_circularity=float(
                    self.declare_parameter("min_circularity", 0.70).value
                ),
                morphology_kernel_size=int(
                    self.declare_parameter("morphology_kernel_size", 9).value
                ),
            )
        )

        self._enabled = False
        self._last_inference_started_s = float("-inf")
        self._image_subscription = None
        self._bridge = CvBridge()

        detection_topic = str(
            self.declare_parameter(
                "detection_topic", "/perception/traffic_light_detection"
            ).value
        )
        active_topic = str(
            self.declare_parameter(
                "active_topic", "/perception/traffic_light_active"
            ).value
        )
        enable_topic = str(
            self.declare_parameter(
                "enable_topic", "/perception/traffic_light_enable"
            ).value
        )
        self._image_topic = str(
            self.declare_parameter(
                "image_topic", "/left_wrist_camera/camera/color/image_raw"
            ).value
        )
        self._detection_publisher = self.create_publisher(
            String, detection_topic, 10
        )
        self._active_publisher = self.create_publisher(
            Bool, active_topic, _event_qos()
        )
        self.create_subscription(Bool, enable_topic, self._enable_callback, 10)
        self._active_publisher.publish(Bool(data=False))
        self.get_logger().info(
            f"Traffic-light HSV spot detector ready but disabled; "
            f"image={self._image_topic}; enable={enable_topic}"
        )

    def _enable_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._last_inference_started_s = float("-inf")
        if enabled:
            self._image_subscription = self.create_subscription(
                Image,
                self._image_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
        elif self._image_subscription is not None:
            self.destroy_subscription(self._image_subscription)
            self._image_subscription = None
        self._active_publisher.publish(Bool(data=enabled))
        if not enabled:
            self._publish_detection(None, 0.0, None, 0.0)
        self.get_logger().info(
            "Traffic-light HSV spot detector enabled"
            if enabled
            else "Traffic-light HSV spot detector disabled"
        )

    def _image_callback(self, message: Image) -> None:
        if not self._enabled:
            return
        now_s = time.monotonic()
        if now_s - self._last_inference_started_s < self._inference_period_s:
            return
        self._last_inference_started_s = now_s
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        started_s = time.monotonic()
        observation, confidence, bbox = self._detect_light(frame)
        inference_s = time.monotonic() - started_s
        self._publish_detection(observation, confidence, bbox, inference_s)

    def _detect_light(
        self, frame
    ) -> tuple[str | None, float, tuple[int, int, int, int] | None]:
        detection = self._detector.detect(frame)
        if detection is None:
            return None, 0.0, None
        return detection.color_name, detection.confidence, detection.bbox

    def _publish_detection(
        self,
        class_name: str | None,
        confidence: float,
        bbox: tuple[int, int, int, int] | None,
        inference_s: float,
    ) -> None:
        self._detection_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "active": self._enabled,
                        "class_name": class_name,
                        "confidence": round(confidence, 4),
                        "bbox": list(bbox) if bbox is not None else None,
                        "inference_ms": round(inference_s * 1000.0, 1),
                    },
                    separators=(",", ":"),
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = TrafficLightRecognitionNode()
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
