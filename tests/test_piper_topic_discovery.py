import ast
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
GRASP_SOURCE = (
    REPO_ROOT
    / "Piper_Grasp_Humble_Migration_20260723"
    / "grasp_single.py"
)


def method_source(method_name: str) -> str:
    source = GRASP_SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.name == method_name:
                segment = ast.get_source_segment(source, node)
                if segment is not None:
                    return segment
    raise AssertionError(f"method not found: {method_name}")


class PiperTopicDiscoveryTests(unittest.TestCase):
    def test_publisher_detection_uses_the_live_node_graph(self):
        source = method_source("topic_has_publishers")

        self.assertIn("self.count_publishers(topic_name)", source)
        self.assertNotIn("--no-daemon", source)
        self.assertNotIn("subprocess.run", source)


if __name__ == "__main__":
    unittest.main()
