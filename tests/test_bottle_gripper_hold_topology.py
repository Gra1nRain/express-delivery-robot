import ast
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
GRASP_SINGLE = (
    REPO_ROOT
    / "Piper_Grasp_Humble_Migration_20260723"
    / "grasp_single.py"
)


def method_source(method_name):
    source = GRASP_SINGLE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "PiperController":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return ast.get_source_segment(source, item)
    raise AssertionError(f"PiperController.{method_name} not found")


class BottleGripperHoldTopologyTest(unittest.TestCase):
    def test_pickup_close_targets_cannot_be_overridden_to_nonzero_width(self):
        source = GRASP_SINGLE.read_text(encoding="utf-8")

        self.assertNotIn("WRIST_BOTTLE_GRIPPER_CLOSED_M", source)
        self.assertNotIn("WRIST_BLOCK_GRIPPER_CLOSED_M", source)

    def test_bottle_contact_feedback_cannot_replace_zero_close_target(self):
        source = method_source("resolve_post_close_hold_gripper")

        self.assertNotIn("get_fresh_gripper_feedback", source)
        self.assertNotIn("choose_bottle_hold_position", source)
        self.assertIn("return float(CLOSE_GRIPPER_M)", source)

    def test_bottle_path_locks_close_target_through_retreat(self):
        source = method_source("execute_upright_bottle_grasp_path")

        self.assertIn(
            "activate_gripper_hold_for_current_target(closed_gripper)",
            source,
        )
        self.assertIn('retreat["gripper"] = closed_gripper', source)


if __name__ == "__main__":
    unittest.main()
