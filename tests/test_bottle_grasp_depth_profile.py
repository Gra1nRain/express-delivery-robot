import pathlib
import re
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
GRASP_SOURCE = (
    REPO_ROOT
    / "Piper_Grasp_Humble_Migration_20260723"
    / "grasp_single.py"
)
RUN_SCRIPT = (
    REPO_ROOT
    / "Piper_Grasp_Humble_Migration_20260723"
    / "run_grasp_single.sh"
)


def source_default(variable_name: str) -> float:
    source = GRASP_SOURCE.read_text(encoding="utf-8")
    match = re.search(
        rf'{variable_name}\s*=\s*float\(\s*'
        rf'os\.getenv\("[^"]+",\s*"([0-9.]+)"\)',
        source,
    )
    if match is None:
        raise AssertionError(f"default not found: {variable_name}")
    return float(match.group(1))


class BottleGraspDepthProfileTests(unittest.TestCase):
    def test_bottle_grasp_is_15mm_deeper_without_changing_blocks(self):
        self.assertAlmostEqual(source_default("BOTTLE_FORWARD_EXTRA_M"), 0.055)
        self.assertAlmostEqual(source_default("BLOCK_FORWARD_EXTRA_M"), 0.040)

    def test_standalone_runner_uses_the_same_bottle_depth(self):
        script = RUN_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'WRIST_BOTTLE_FORWARD_EXTRA_M="${WRIST_BOTTLE_FORWARD_EXTRA_M:-0.055}"',
            script,
        )


if __name__ == "__main__":
    unittest.main()
