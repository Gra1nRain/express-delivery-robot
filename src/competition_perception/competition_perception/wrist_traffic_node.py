#!/usr/bin/env python3
"""Run flag-wave and traffic-light recognition on the existing wrist RGB topic."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from competition_perception.red_flag import RedFlagColorDetector
from competition_perception.traffic_rules import (
    FlagWaveDetector,
    TrafficRuleController,
    WaveConfig,
)


class WristTrafficPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("wrist_traffic_perception")
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
        self._camera_timeout_s = float(
            self.declare_parameter("camera_timeout_s", 1.0).value
        )
        self._frame_index = 0
        self._inference_stride = max(
            1, int(self.declare_parameter("inference_stride", 1).value)
        )
        self._last_frame_s: float | None = None
        self._last_light_confidence = 0.0
        self._last_light_observation: str | None = None
        self._last_light_bbox: tuple[int, int, int, int] | None = None
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

        self._flag_color = RedFlagColorDetector(
            saturation_threshold=int(
                self.declare_parameter("flag_saturation_threshold", 100).value
            ),
            value_threshold=int(
                self.declare_parameter("flag_value_threshold", 100).value
            ),
            min_area_px=float(self.declare_parameter("flag_min_area_px", 800.0).value),
            border_margin_px=int(
                self.declare_parameter("flag_border_margin_px", 8).value
            ),
        )
        self._flag_wave = FlagWaveDetector(
            WaveConfig(
                trajectory_window=int(
                    self.declare_parameter("flag_trajectory_window", 20).value
                ),
                min_displacement_px=float(
                    self.declare_parameter("flag_min_motion_px", 30.0).value
                ),
                cooldown_s=float(
                    self.declare_parameter("flag_cooldown_s", 2.0).value
                ),
                max_lost_frames=int(
                    self.declare_parameter("flag_max_lost_frames", 3).value
                ),
            )
        )
        self._rules = TrafficRuleController(
            confirm_frames=int(
                self.declare_parameter("light_confirm_frames", 3).value
            )
        )

        stop_topic = str(
            self.declare_parameter(
                "stop_request_topic", "/perception/traffic_stop_request"
            ).value
        )
        self._stop_publisher = self.create_publisher(Bool, stop_topic, 10)
        self._flag_publisher = self.create_publisher(
            Bool, "/perception/flag_wave_detected", 10
        )
        self._light_publisher = self.create_publisher(
            String, "/perception/traffic_light_state", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/perception/traffic_rules_status", 10
        )
        debug_image_topic = str(
            self.declare_parameter(
                "debug_image_topic", "/perception/wrist_traffic_annotated"
            ).value
        )
        self._debug_publisher = self.create_publisher(
            Image, debug_image_topic, qos_profile_sensor_data
        )
        image_topic = str(
            self.declare_parameter(
                "image_topic", "/left_wrist_camera/camera/color/image_raw"
            ).value
        )
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._publish_state)
        self.get_logger().info(
            f"Wrist traffic perception ready; topic={image_topic}; "
            f"model={model_path}; classes={self._class_names}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _image_callback(self, message: Image) -> None:
        now_s = self._now_s()
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        waiting_for_flag = not self._rules.decision.started
        flag = self._flag_color.detect(frame)
        triggered = self._flag_wave.update(
            centroid_x=None if flag is None else float(flag.centroid_x),
            centroid_y=None if flag is None else float(flag.centroid_y),
            timestamp_s=now_s,
        )
        if triggered:
            self._rules.observe_flag_wave()
            self._flag_publisher.publish(Bool(data=True))
            self.get_logger().info("Start flag wave confirmed")

        self._frame_index += 1
        if waiting_for_flag:
            self._publish_debug_image(frame, message, flag)
            self._last_frame_s = self._now_s()
            return
        if self._frame_index % self._inference_stride != 0:
            self._publish_debug_image(frame, message, flag)
            self._last_frame_s = self._now_s()
            return
        observation, confidence, bbox = self._detect_light(frame)
        self._last_light_observation = observation
        self._last_light_confidence = confidence
        self._last_light_bbox = bbox
        self._rules.observe_light(observation)
        self._publish_debug_image(frame, message, flag)
        self._last_frame_s = self._now_s()

    def _detect_light(
        self, frame
    ) -> tuple[str | None, float, tuple[int, int, int, int] | None]:
        results = self._model.predict(
            frame,
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

    def _publish_debug_image(self, frame, source_message: Image, flag) -> None:
        if self._debug_publisher.get_subscription_count() == 0:
            return

        annotated = frame.copy()
        if flag is not None:
            cv2.rectangle(
                annotated,
                (flag.x, flag.y),
                (flag.x + flag.width, flag.y + flag.height),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                annotated,
                (flag.centroid_x, flag.centroid_y),
                4,
                (0, 255, 0),
                -1,
            )
            cv2.putText(
                annotated,
                "RED FLAG COLOR",
                (flag.x, max(20, flag.y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        if self._last_light_bbox is not None:
            x1, y1, x2, y2 = self._last_light_bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 180, 0), 2)
            label = (
                f"LIGHT {str(self._last_light_observation).upper()} "
                f"{self._last_light_confidence:.2f}"
            )
            cv2.putText(
                annotated,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 180, 0),
                2,
            )

        decision = self._rules.decision
        started_text = "CONFIRMED" if decision.started else "WAITING"
        action_text = "STOP" if decision.stop_required else "GO"
        action_color = (0, 0, 255) if decision.stop_required else (0, 255, 0)
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 104), (20, 20, 20), -1)
        lines = (
            f"FLAG: {self._flag_wave.state.value.upper()}",
            f"START: {started_text}",
            f"LIGHT: {decision.light.value.upper()}  "
            f"CONF: {self._last_light_confidence:.2f}",
        )
        for index, text in enumerate(lines):
            cv2.putText(
                annotated,
                text,
                (12, 24 + index * 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                2,
            )
        cv2.putText(
            annotated,
            action_text,
            (annotated.shape[1] - 100, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            action_color,
            3,
        )

        debug_message = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_message.header = source_message.header
        self._debug_publisher.publish(debug_message)

    def _publish_state(self) -> None:
        now_s = self._now_s()
        camera_stale = bool(
            self._last_frame_s is None
            or now_s - self._last_frame_s > self._camera_timeout_s
        )
        decision = self._rules.decision
        stop_required = bool(camera_stale or decision.stop_required)
        reason = "wrist_camera_stale" if camera_stale else decision.reason
        self._stop_publisher.publish(Bool(data=stop_required))
        self._flag_publisher.publish(Bool(data=False))
        self._light_publisher.publish(String(data=decision.light.value))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "started": decision.started,
                        "light": decision.light.value,
                        "light_confidence": round(self._last_light_confidence, 4),
                        "flag_state": self._flag_wave.state.value,
                        "camera_stale": camera_stale,
                        "stop_required": stop_required,
                        "reason": reason,
                    },
                    separators=(",", ":"),
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = WristTrafficPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop_publisher.publish(Bool(data=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
