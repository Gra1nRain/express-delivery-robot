import pathlib
import sys
import tempfile
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.artifact_provenance import (
    build_source_manifest,
    resolve_trajectory_source_paths,
    validate_source_manifest,
)


class ArtifactProvenanceTest(unittest.TestCase):
    def test_current_segmented_artifact_matches_all_current_sources(self) -> None:
        artifact_path = (
            REPO_ROOT / "docs" / "evidence" / "day4" / "debug_optimized_trajectory.yaml"
        )
        artifact = yaml.safe_load(artifact_path.read_text(encoding="utf-8"))
        sources = resolve_trajectory_source_paths(
            route_file=REPO_ROOT / "config" / "routes" / "debug_route.yaml",
            semantic_map_file=REPO_ROOT / "maps" / "debug" / "semantic_map.yaml",
            planning_params_file=(
                REPO_ROOT / "config" / "planning" / "planning_params.yaml"
            ),
            optimizer_params_file=(
                REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
            ),
        )

        validate_source_manifest(artifact, sources)
        self.assertEqual(
            set(artifact["source_manifest"]),
            {
                "route",
                "semantic_map",
                "planning_params",
                "optimizer_params",
                "occupancy_map",
                "occupancy_image",
            },
        )

    def test_rejects_frozen_trajectory_after_any_source_changes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = pathlib.Path(tmp_dir)
            route = root / "route.yaml"
            planning = root / "planning.yaml"
            route.write_text("route_name: debug\n", encoding="utf-8")
            planning.write_text("global_planner: {}\n", encoding="utf-8")
            sources = {"route": route, "planning_params": planning}
            artifact = {"source_manifest": build_source_manifest(sources)}

            validate_source_manifest(artifact, sources)
            planning.write_text("global_planner:\n  min_turning_radius_m: 0.81\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "planning_params"):
                validate_source_manifest(artifact, sources)

    def test_indoor_one_lap_artifact_matches_its_route_sources(self) -> None:
        artifact_path = (
            REPO_ROOT
            / "docs"
            / "evidence"
            / "day4"
            / "debug_indoor_one_lap_trajectory.yaml"
        )
        artifact = yaml.safe_load(artifact_path.read_text(encoding="utf-8"))
        sources = resolve_trajectory_source_paths(
            route_file=(
                REPO_ROOT
                / "config"
                / "routes"
                / "debug_indoor_one_lap_route.yaml"
            ),
            semantic_map_file=REPO_ROOT / "maps" / "debug" / "semantic_map.yaml",
            planning_params_file=(
                REPO_ROOT / "config" / "planning" / "planning_params.yaml"
            ),
            optimizer_params_file=(
                REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
            ),
        )

        validate_source_manifest(artifact, sources)


if __name__ == "__main__":
    unittest.main()
