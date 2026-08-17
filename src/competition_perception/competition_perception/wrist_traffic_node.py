#!/usr/bin/env python3
"""Run flag-wave and traffic-light recognition on the existing wrist RGB topic."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
        )
        self._flag_wave = FlagWaveDetector(
            WaveConfig(
                trajectory_window=int(
                    self.declare_parameter("flag_trajectory_window", 20).value
                ),
                min_downward_displacement_px=float(
                    self.declare_parameter("flag_min_downward_px", 40.0).value
                ),
                direct_min_speed_pxps=float(
                    self.declare_parameter("flag_direct_min_speed_pxps", 80.0).value
                ),
                ready_min_speed_pxps=float(
                    self.declare_parameter("flag_ready_min_speed_pxps", 50.0).value
                ),
                prepare_min_displacement_px=float(
                    self.declare_parameter("flag_prepare_min_displacement_px", -15.0).value
                ),
                min_total_travel_px=float(
                    self.declare_parameter("flag_min_total_travel_px", 100.0).value
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
        self._last_frame_s = now_s
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        flag = self._flag_color.detect(frame)
        triggered = self._flag_wave.update(
            centroid_y=None if flag is None else float(flag.centroid_y),
            timestamp_s=now_s,
        )
        if triggered:
            self._rules.observe_flag_wave()
            self._flag_publisher.publish(Bool(data=True))
            self.get_logger().info("Start flag wave confirmed")

        self._frame_index += 1
        if self._frame_index % self._inference_stride != 0:
            return
        observation, confidence = self._detect_light(frame)
        self._last_light_confidence = confidence
        self._rules.observe_light(observation)

    def _detect_light(self, frame) -> tuple[str | None, float]:
        results = self._model.predict(
            frame,
            conf=self._confidence,
            device=self._device,
            verbose=False,
        )
        best_name: str | None = None
        best_confidence = 0.0
        if not results or results[0].boxes is None:
            return best_name, best_confidence
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
        return best_name, best_confidence

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
