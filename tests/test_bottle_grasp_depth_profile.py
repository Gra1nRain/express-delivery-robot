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
    def test_bottle_grasp_height_targets_the_lower_body(self):
        bottom_z = 0.04403605297762281
        top_z = 0.16734633421797923
        height_m = top_z - bottom_z
        fraction = source_default("BOTTLE_GRASP_HEIGHT_FRACTION")
        min_above_bottom_m = source_default(
            "BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M"
        )
        max_below_top_m = source_default(
            "BOTTLE_GRASP_MAX_BELOW_TOP_M"
        )

        lower_z = bottom_z + min(min_above_bottom_m, height_m * 0.45)
        upper_z = top_z - min(max_below_top_m, height_m * 0.45)
        grasp_z = min(max(bottom_z + height_m * fraction, lower_z), upper_z)

        self.assertAlmostEqual(fraction, 0.25)
        self.assertAlmostEqual(min_above_bottom_m, 0.020)
        self.assertLessEqual(grasp_z, bottom_z + height_m * 0.30)
        self.assertGreaterEqual(grasp_z, bottom_z + 0.020)

    def test_final_bottle_target_is_clamped_after_pose_compensation(self):
        source = GRASP_SOURCE.read_text(encoding="utf-8")

        self.assertIn("minimum_bottle_target_z", source)
        self.assertIn(
            'float(object_result["bottom_center"][2])\n'
            "                + BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M",
            source,
        )
        self.assertIn(
            'target_center[2] = max(\n'
            '                float(target_center[2]),\n'
            '                minimum_bottle_target_z,',
            source,
        )

    def test_bottle_grasp_is_15mm_deeper_without_changing_blocks(self):
        self.assertAlmostEqual(source_default("BOTTLE_FORWARD_EXTRA_M"), 0.055)
        self.assertAlmostEqual(source_default("BLOCK_FORWARD_EXTRA_M"), 0.040)

    def test_block_descends_20mm_without_changing_bottle_height_policy(self):
        source = GRASP_SOURCE.read_text(encoding="utf-8")

        self.assertAlmostEqual(source_default("BLOCK_EXTRA_DESCENT_M"), 0.020)
        self.assertIn(
            "elif is_block_target:\n"
            "            target_center[2] -= BLOCK_EXTRA_DESCENT_M",
            source,
        )
        self.assertAlmostEqual(source_default("BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M"), 0.020)

    def test_standalone_runner_uses_the_same_bottle_depth(self):
        script = RUN_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'WRIST_BOTTLE_FORWARD_EXTRA_M="${WRIST_BOTTLE_FORWARD_EXTRA_M:-0.055}"',
            script,
        )

    def test_standalone_runner_targets_the_lower_bottle_body(self):
        script = RUN_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'WRIST_BOTTLE_GRASP_HEIGHT_FRACTION="${WRIST_BOTTLE_GRASP_HEIGHT_FRACTION:-0.25}"',
            script,
        )
        self.assertIn(
            'WRIST_BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M="${WRIST_BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M:-0.020}"',
            script,
        )


if __name__ == "__main__":
    unittest.main()
