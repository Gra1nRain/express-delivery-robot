import ast
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
GRASP_SOURCE_PATH = (
    REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723" / "grasp_single.py"
)


def method_source(method_name):
    source = GRASP_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "PiperController":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return ast.get_source_segment(source, item)
    raise AssertionError(f"PiperController.{method_name} not found")


class BlockGripperHoldTopologyTest(unittest.TestCase):
    def test_block_path_keeps_the_commanded_closed_target(self):
        source = method_source("execute_adaptive_block_grasp_path")

        self.assertNotIn("resolve_block_post_close_hold_gripper", source)
        self.assertIn(
            "activate_gripper_hold_for_current_target(closed_gripper)",
            source,
        )
        self.assertIn('retreat["gripper"] = closed_gripper', source)

    def test_pose_hold_commands_cannot_bypass_the_gripper_guard(self):
        source = method_source("publish_pose_for")

        self.assertIn("self.apply_gripper_hold", source)


if __name__ == "__main__":
    unittest.main()
