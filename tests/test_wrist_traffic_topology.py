from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


def test_yolo_isolated_from_flag_and_view_node() -> None:
    wrist_node = _read(
        "src/competition_perception/competition_perception/wrist_traffic_node.py"
    )
    light_node = _read(
        "src/competition_perception/competition_perception/traffic_light_node.py"
    )

    assert "from ultralytics import YOLO" not in wrist_node
    assert "from ultralytics import YOLO" in light_node
    assert '"/perception/traffic_light_enable"' in light_node


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


def test_manual_test_script_can_toggle_yolo() -> None:
    script = _read("scripts/start_wrist_vision_test.sh")

    assert '"--enable-light"' in script
    assert '"--disable-light"' in script
    assert 'TRAFFIC_LIGHT_ENABLE_TOPIC="/perception/traffic_light_enable"' in script


def test_cpu_yolo_uses_benchmarked_input_size_and_starts_disabled() -> None:
    config = _read("config/perception/wrist_traffic_rules.yaml")
    light_node = _read(
        "src/competition_perception/competition_perception/traffic_light_node.py"
    )

    assert "inference_image_size: 320" in config
    assert "self._enabled = False" in light_node
    assert "if not self._enabled:" in light_node
    assert "self.destroy_subscription(self._image_subscription)" in light_node
