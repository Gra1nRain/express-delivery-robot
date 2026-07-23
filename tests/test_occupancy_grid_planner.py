import math
import pathlib
import sys
import time
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.occupancy_grid_planner import OccupancyGridMap


def _reference_inflation(
    grid_map: OccupancyGridMap,
    inflation_radius_m: float,
) -> tuple[bool, ...]:
    radius_cells = math.ceil(inflation_radius_m / grid_map.resolution)
    blocked = [False] * len(grid_map.occupied)
    for row in range(grid_map.height):
        for col in range(grid_map.width):
            if not grid_map.occupied[row * grid_map.width + col]:
                continue
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    target_col = col + dx
                    target_row = row + dy
                    if (
                        dx * dx + dy * dy <= radius_cells * radius_cells
                        and 0 <= target_col < grid_map.width
                        and 0 <= target_row < grid_map.height
                    ):
                        blocked[target_row * grid_map.width + target_col] = True
    return tuple(blocked)


class OccupancyGridInflationTest(unittest.TestCase):
    def test_inflation_matches_reference_at_map_edges(self) -> None:
        width = 7
        height = 6
        occupied = [False] * (width * height)
        for col, row in ((0, 0), (3, 2), (6, 5)):
            occupied[row * width + col] = True
        grid_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution=0.10,
            origin_x=0.0,
            origin_y=0.0,
            occupied=tuple(occupied),
        )

        self.assertEqual(
            grid_map.inflated_blocked(0.20),
            _reference_inflation(grid_map, 0.20),
        )

    def test_dense_grid_inflation_meets_local_replanning_budget(self) -> None:
        size = 160
        grid_map = OccupancyGridMap(
            width=size,
            height=size,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            occupied=(True,) * (size * size),
        )

        started_at = time.perf_counter()
        blocked = grid_map.inflated_blocked(0.50)
        elapsed_s = time.perf_counter() - started_at

        self.assertTrue(all(blocked))
        self.assertLess(elapsed_s, 0.50)


if __name__ == "__main__":
    unittest.main()
