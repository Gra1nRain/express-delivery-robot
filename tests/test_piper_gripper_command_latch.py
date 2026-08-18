import ast
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER_SOURCE = (
    REPO_ROOT
    / "Piper_Grasp_Humble_Migration_20260723"
    / "drivers"
    / "piper_ros"
    / "src"
    / "piper"
    / "piper"
    / "piper_ctrl_single_node.py"
)


def source_tree():
    source = DRIVER_SOURCE.read_text(encoding="utf-8")
    return source, ast.parse(source)


def function_source(function_name: str) -> str:
    source, tree = source_tree()
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            segment = ast.get_source_segment(source, node)
            if segment is not None:
                return segment
    raise AssertionError(f"function not found: {function_name}")


def method_source(method_name: str) -> str:
    source, tree = source_tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == method_name:
            segment = ast.get_source_segment(source, node)
            if segment is not None:
                return segment
    raise AssertionError(f"method not found: {method_name}")


class PiperGripperCommandLatchTests(unittest.TestCase):
    def test_identical_gripper_commands_are_suppressed(self):
        namespace = {}
        exec(function_source("update_gripper_command_latch"), namespace)
        update_latch = namespace["update_gripper_command_latch"]

        should_send, latched = update_latch(None, 0, 1000)
        self.assertTrue(should_send)

        should_send, repeated = update_latch(latched, 0, 1000)
        self.assertFalse(should_send)
        self.assertEqual(repeated, latched)

        should_send, opened = update_latch(latched, 100000, 1000)
        self.assertTrue(should_send)
        self.assertNotEqual(opened, latched)

    def test_pose_and_joint_callbacks_use_the_latched_sender(self):
        for callback_name in ("pos_callback", "joint_callback"):
            source = method_source(callback_name)
            self.assertIn("self.send_gripper_command_if_changed", source)
            self.assertNotIn("self.piper.GripperCtrl", source)


if __name__ == "__main__":
    unittest.main()
