from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


def test_light_spot_detector_reuses_flag_and_view_camera() -> None:
    wrist_node = _read(
        "src/competition_perception/competition_perception/wrist_traffic_node.py"
    )
    light_node = _read(
        "src/competition_perception/competition_perception/traffic_light_node.py"
    )

    assert "from ultralytics import YOLO" not in wrist_node
    assert "from ultralytics import YOLO" not in light_node
    assert "LightSpotDetector" in light_node
    assert '"/perception/traffic_light_enable"' in light_node
    assert '"/perception/traffic_stop_enable"' in wrist_node
    assert "set_traffic_stop_enabled" in wrist_node
    assert "if not self._rules.decision.started:" in wrist_node
    assert '"flag_active": not decision.started' in wrist_node


def test_launch_starts_one_camera_and_two_perception_nodes() -> None:
    launch = _read(
        "src/competition_perception/launch/wrist_traffic.launch.py"
    )

    assert launch.count("IncludeLaunchDescription(") == 1
    assert 'executable="wrist_traffic_node"' in launch
    assert 'executable="traffic_light_node"' in launch

    day5_launch = _read(
        "src/competition_bringup/launch/day5_motion_control.launch.py"
    )
    assert 'executable="wrist_traffic_node"' in day5_launch
    assert 'executable="traffic_light_node"' in day5_launch


def test_manual_test_script_can_toggle_light_detection() -> None:
    script = _read("scripts/start_wrist_vision_test.sh")

    assert '"--enable-light"' in script
    assert '"--disable-light"' in script
    assert '"--reset-flag"' in script
    assert 'TRAFFIC_LIGHT_ENABLE_TOPIC="/perception/traffic_light_enable"' in script


def test_light_spot_detector_uses_reference_thresholds_and_starts_disabled() -> None:
    config = _read("config/perception/wrist_traffic_rules.yaml")
    parsed_config = yaml.safe_load(config)
    light_node = _read(
        "src/competition_perception/competition_perception/traffic_light_node.py"
    )

    params = parsed_config["traffic_light_recognition"]["ros__parameters"]
    assert params["brightness_threshold"] == 100
    assert params["min_spot_area_px"] == 30.0
    assert params["max_spot_area_px"] == 8000.0
    assert params["min_circularity"] == 0.55
    assert params["dominance_ratio"] == 1.5
    assert (
        params["inference_period_s"] <= 0.1
    )
    assert "ultralytics" not in light_node
    assert "self._enabled = False" in light_node
    assert "if not self._enabled:" in light_node
    assert "self.destroy_subscription(self._image_subscription)" in light_node
