#!/usr/bin/env python3
"""Run on-demand YOLO traffic-light recognition on the wrist RGB topic."""

from __future__ import annotations

import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
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
from ultralytics import YOLO


def _event_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TrafficLightRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_recognition")
        default_model = Path(get_package_share_directory("competition_perception")) / (
            "models/best_traffic_nano_yolo.pt"
        )
        model_path = Path(
            str(self.declare_parameter("model_path", str(default_model)).value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"traffic-light model not found: {model_path}")

        self._confidence = float(self.declare_parameter("confidence", 0.30).value)
        self._device = str(self.declare_parameter("device", "cpu").value)
        self._image_size = int(
            self.declare_parameter("inference_image_size", 320).value
        )
        self._inference_period_s = float(
            self.declare_parameter("inference_period_s", 0.50).value
        )
        if self._image_size <= 0:
            raise ValueError("inference_image_size must be positive")
        if self._inference_period_s < 0.0:
            raise ValueError("inference_period_s must be non-negative")

        self._enabled = False
        self._last_inference_started_s = float("-inf")
        self._image_subscription = None
        self._bridge = CvBridge()
        self._model = YOLO(str(model_path))
        self._class_names = {
            int(class_id): str(name).strip().lower()
            for class_id, name in dict(self._model.names).items()
        }
        missing = {"red", "green"} - set(self._class_names.values())
        if missing:
            raise ValueError(
                "traffic-light model is missing required embedded classes: "
                + ", ".join(sorted(missing))
            )

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
            f"Traffic-light YOLO ready but disabled; image={self._image_topic}; "
            f"enable={enable_topic}; imgsz={self._image_size}; model={model_path}"
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
            "Traffic-light YOLO enabled" if enabled else "Traffic-light YOLO disabled"
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
        results = self._model.predict(
            frame,
            imgsz=self._image_size,
            conf=self._confidence,
            device=self._device,
            verbose=False,
        )
        best_name: str | None = None
        best_confidence = 0.0
        best_bbox: tuple[int, int, int, int] | None = None
        if not results or results[0].boxes is None:
            return best_name, best_confidence, best_bbox
        for box in results[0].boxes:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            name = self._class_names.get(class_id)
            if (
                name in {"red", "green", "yellow", "off"}
                and confidence > best_confidence
            ):
                best_name = name
                best_confidence = confidence
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                best_bbox = (int(x1), int(y1), int(x2), int(y2))
        return best_name, best_confidence, best_bbox

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
