import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_safety.shutdown_guard import publish_shutdown_zero_if_ready


class ShutdownGuardTest(unittest.TestCase):
    def test_does_not_publish_after_ros_context_is_invalid(self) -> None:
        published = []

        result = publish_shutdown_zero_if_ready(
            context_is_valid=lambda: False,
            publish_zero=lambda: published.append(True),
        )

        self.assertFalse(result)
        self.assertEqual(published, [])

    def test_publishes_zero_while_ros_context_is_valid(self) -> None:
        published = []

        result = publish_shutdown_zero_if_ready(
            context_is_valid=lambda: True,
            publish_zero=lambda: published.append(True),
        )

        self.assertTrue(result)
        self.assertEqual(published, [True])


if __name__ == "__main__":
    unittest.main()
