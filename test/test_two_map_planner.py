from __future__ import annotations

import math
from pathlib import Path
import re
import unittest

import cv2
import numpy as np

from computer.task4 import ContinuousPlanner, PlannerConfig
from mazemappingtask2 import (
    DemoConfig,
    Portal,
    build_normal_maze_map,
    build_obstacle_map,
    draw_normal_wall_diagnostics,
    encode_cardinal_turn,
    encode_cell_path,
    make_grid_calibration,
    plan_two_map_route,
    prepare_mapping_preview,
    run_two_map_demo,
    transform_points,
)


COMMAND_PATTERN = re.compile(
    r'^"[lrf]*,\['
    r'(?:\(-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?\)'
    r'(?:,\(-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?\))*)?'
    r'\],[lrf]*";$'
)
MOTION_PAIR_PATTERN = re.compile(
    r'\((-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)\)'
)


class TwoMapPlannerTests(unittest.TestCase):
    @staticmethod
    def _blank_task1_masks(size: int) -> dict[str, np.ndarray]:
        empty = np.zeros((size, size), dtype=np.uint8)
        return {
            "03a_main_dark_mask.png": empty.copy(),
            "02b_horizontal_line_mask.png": empty.copy(),
            "02c_vertical_line_mask.png": empty.copy(),
        }

    @classmethod
    def _synthetic_maps(cls):
        """Return matching normal/crop maps with one in-crop cylinder."""
        size = 180
        rectified = np.full((size, size, 3), 255, dtype=np.uint8)
        masks = cls._blank_task1_masks(size)

        # This cylinder lies inside rows/columns 2..6. The second dark object
        # is deliberately outside that crop and must not leak into its map.
        cv2.circle(masks["03a_main_dark_mask.png"], (90, 90), 6, 255, -1)
        cv2.circle(masks["03a_main_dark_mask.png"], (10, 10), 6, 255, -1)

        calibration = make_grid_calibration(0, 0, size, size)
        entrance = Portal(outside_cell=(7, 4), inside_cell=(6, 4))
        exit_portal = Portal(outside_cell=(1, 4), inside_cell=(2, 4))
        normal_map = build_normal_maze_map(
            masks,
            calibration,
            obstacle_top_left=(2, 2),
            obstacle_size=5,
            entrance=entrance,
            exit=exit_portal,
        )
        obstacle_map = build_obstacle_map(
            rectified,
            masks,
            calibration,
            obstacle_top_left=(2, 2),
            obstacle_size=5,
            entrance=entrance,
            exit=exit_portal,
            resolution_mm=10.0,
        )
        return rectified, normal_map, obstacle_map

    def assert_cardinal_cell_path(self, cells) -> None:
        for first, second in zip(cells, cells[1:]):
            self.assertEqual(
                abs(second[0] - first[0]) + abs(second[1] - first[1]),
                1,
                f"non-cardinal or skipped normal-maze edge: {first} -> {second}",
            )

    def assert_valid_hybrid_command(self, route) -> None:
        self.assertRegex(route.command, COMMAND_PATTERN)
        before, remainder = route.command[1:-2].split(",[", maxsplit=1)
        obstacle, after = remainder.rsplit("],", maxsplit=1)

        self.assertTrue(set(before) <= set("lrf"))
        self.assertTrue(set(after) <= set("lrf"))
        self.assertEqual(before.count("f"), len(route.normal_before_cells) - 1)
        self.assertEqual(after.count("f"), len(route.normal_after_cells) - 1)

        pairs = [
            (float(turn), float(distance))
            for turn, distance in MOTION_PAIR_PATTERN.findall(obstacle)
        ]
        self.assertTrue(pairs, "continuous obstacle command list is empty")
        for index, (turn, distance) in enumerate(pairs):
            self.assertTrue(math.isfinite(turn))
            self.assertTrue(math.isfinite(distance))
            self.assertLessEqual(abs(turn), 180.0)
            self.assertGreaterEqual(distance, 0.0)
            if distance == 0.0:
                self.assertEqual(
                    index,
                    len(pairs) - 1,
                    "only the final exit-alignment command may have zero distance",
                )

    def assert_route_joins(self, normal_map, route, tolerance: float = 1.5) -> None:
        entrance_full = normal_map.calibration.centre(
            route.normal_before_cells[-1]
        )
        exit_full = normal_map.calibration.centre(route.normal_after_cells[0])
        self.assertLessEqual(
            math.dist(entrance_full, route.obstacle_waypoints_full[0]),
            tolerance,
        )
        self.assertLessEqual(
            math.dist(exit_full, route.obstacle_waypoints_full[-1]),
            tolerance,
        )

    def test_normal_cell_commands_are_exact_lfr_per_cell(self) -> None:
        cells = [(8, 0), (7, 0), (7, 1), (7, 2), (8, 2)]

        command, final_heading = encode_cell_path(cells, 90.0)

        self.assertEqual(command, "frffrf")
        self.assertEqual(command.count("f"), len(cells) - 1)
        self.assertEqual(final_heading, -90.0)
        self.assertEqual(encode_cell_path([(0, 0), (1, 0)], 90.0)[0], "rrf")
        self.assertEqual(encode_cardinal_turn(90.0, 0.0), "r")
        self.assertEqual(encode_cardinal_turn(0.0, 180.0), "rr")
        self.assertEqual(encode_cardinal_turn(-90.0, -90.0), "")
        with self.assertRaises(ValueError):
            encode_cardinal_turn(0.0, 45.0)

    def test_normal_graph_excludes_obstacle_interior_except_portals(self) -> None:
        masks = self._blank_task1_masks(90)
        # Block the shared boundary between (7, 0) and (7, 1).
        cv2.line(
            masks["02c_vertical_line_mask.png"],
            (10, 70),
            (10, 80),
            255,
            3,
        )
        calibration = make_grid_calibration(0, 0, 90, 90)
        entrance = Portal((7, 4), (6, 4))
        exit_portal = Portal((1, 4), (2, 4))

        normal_map = build_normal_maze_map(
            masks,
            calibration,
            obstacle_top_left=(2, 2),
            obstacle_size=5,
            entrance=entrance,
            exit=exit_portal,
        )

        region = {(row, column) for row in range(2, 7) for column in range(2, 7)}
        self.assertEqual(
            normal_map.excluded_cells,
            region - {entrance.inside_cell, exit_portal.inside_cell},
        )
        self.assertFalse(normal_map.excluded_cells & set(normal_map.graph))
        self.assertEqual(normal_map.graph[entrance.inside_cell], [entrance.outside_cell])
        self.assertEqual(normal_map.graph[exit_portal.inside_cell], [exit_portal.outside_cell])
        self.assertNotIn((7, 1), normal_map.graph[(7, 0)])
        self.assertTrue(
            normal_map.walls[frozenset(((7, 0), (7, 1)))].blocked
        )
        for cell, neighbours in normal_map.graph.items():
            for neighbour in neighbours:
                self.assertEqual(
                    abs(neighbour[0] - cell[0]) + abs(neighbour[1] - cell[1]),
                    1,
                )

    def test_normal_wall_diagnostic_draws_graph_decisions(self) -> None:
        masks = self._blank_task1_masks(90)
        cv2.line(
            masks["02c_vertical_line_mask.png"],
            (10, 70),
            (10, 80),
            255,
            3,
        )
        calibration = make_grid_calibration(0, 0, 90, 90)
        normal_map = build_normal_maze_map(
            masks,
            calibration,
            obstacle_top_left=(2, 2),
            obstacle_size=5,
            entrance=Portal((7, 4), (6, 4)),
            exit=Portal((1, 4), (2, 4)),
        )

        diagnostic = draw_normal_wall_diagnostics(
            np.full((90, 90, 3), 240, dtype=np.uint8),
            normal_map,
        )

        np.testing.assert_array_equal(diagnostic[75, 10], (0, 0, 255))
        np.testing.assert_array_equal(diagnostic[85, 10], (240, 240, 240))
        np.testing.assert_array_equal(diagnostic[70, 45], (240, 240, 240))

    def test_borderline_wall_evidence_remains_traversable(self) -> None:
        masks = self._blank_task1_masks(90)
        # Two pixels across this sampled boundary produce a borderline score:
        # retain the metadata, but do not create a graph wall.
        cv2.line(
            masks["02c_vertical_line_mask.png"],
            (10, 72),
            (10, 73),
            255,
            1,
        )
        normal_map = build_normal_maze_map(
            masks,
            make_grid_calibration(0, 0, 90, 90),
            obstacle_top_left=(2, 2),
            obstacle_size=5,
            entrance=Portal((7, 4), (6, 4)),
            exit=Portal((1, 4), (2, 4)),
        )
        edge = frozenset(((7, 0), (7, 1)))

        self.assertTrue(normal_map.walls[edge].uncertain)
        self.assertFalse(normal_map.walls[edge].blocked)
        self.assertIn((7, 1), normal_map.graph[(7, 0)])

    def test_obstacle_crop_is_independent_and_detects_only_inside_cylinder(self) -> None:
        _, _, obstacle_map = self._synthetic_maps()

        self.assertEqual(obstacle_map.occupancy_mask.shape, (90, 90))
        self.assertEqual((obstacle_map.grid.width, obstacle_map.grid.height), (90, 90))
        self.assertEqual(len(obstacle_map.detected_centres), 1)
        centre = obstacle_map.detected_centres[0]
        self.assertLessEqual(math.dist(centre, (44, 44)), 2.0)
        self.assertNotEqual(int(obstacle_map.occupancy_mask[centre[1], centre[0]]), 0)
        self.assertEqual(
            int(obstacle_map.occupancy_mask[10, 10]),
            0,
            "the dark object outside the crop leaked into the obstacle map",
        )

    def test_obstacle_astar_uses_angles_and_avoids_inflated_occupancy(self) -> None:
        _, _, obstacle_map = self._synthetic_maps()
        planner = ContinuousPlanner(
            PlannerConfig(
                robot_radius_mm=0.0,
                safety_margin_mm=0.0,
                simplify_path=True,
            )
        )

        result = planner.plan(
            obstacle_map.grid,
            obstacle_map.entrance_local,
            obstacle_map.exit_local,
        )

        self.assertTrue(result.succeeded, result.status.value)
        self.assertEqual(
            (result.grid_path[0].x, result.grid_path[0].y),
            obstacle_map.entrance_local,
        )
        self.assertEqual(
            (result.grid_path[-1].x, result.grid_path[-1].y),
            obstacle_map.exit_local,
        )
        self.assertTrue(
            all(not result.inflated_grid.is_occupied(point) for point in result.grid_path)
        )
        self.assertTrue(
            any(
                first.x != second.x and first.y != second.y
                for first, second in zip(
                    result.grid_waypoints,
                    result.grid_waypoints[1:],
                )
            ),
            "the forced obstacle detour did not retain an angled segment",
        )

    def test_crop_transform_round_trip_and_route_joins_are_continuous(self) -> None:
        _, normal_map, obstacle_map = self._synthetic_maps()
        local_points = [
            obstacle_map.entrance_local,
            (17, 33),
            obstacle_map.exit_local,
        ]

        full_points = transform_points(local_points, obstacle_map.local_to_full)
        round_trip = transform_points(full_points, obstacle_map.full_to_local)

        for expected, actual in zip(local_points, round_trip):
            self.assertLessEqual(math.dist(expected, actual), 1e-3)

        route = plan_two_map_route(
            normal_map,
            obstacle_map,
            start=(8, 4),
            goal=(0, 4),
            initial_heading_deg=90.0,
            robot_radius_mm=0.0,
            safety_margin_mm=0.0,
        )
        self.assertTrue(route.succeeded)
        self.assert_route_joins(normal_map, route)

    def test_hybrid_command_contains_angles_only_in_obstacle_section(self) -> None:
        _, normal_map, obstacle_map = self._synthetic_maps()
        route = plan_two_map_route(
            normal_map,
            obstacle_map,
            start=(8, 4),
            goal=(0, 4),
            initial_heading_deg=90.0,
            robot_radius_mm=0.0,
            safety_margin_mm=0.0,
        )

        self.assert_valid_hybrid_command(route)
        obstacle_text = route.command.split(",[", 1)[1].rsplit("],", 1)[0]
        moving_pairs = [
            (float(turn), float(distance))
            for turn, distance in MOTION_PAIR_PATTERN.findall(obstacle_text)
            if float(distance) > 0.0
        ]
        self.assertTrue(
            any(abs(turn / 90.0 - round(turn / 90.0)) > 1e-3 for turn, _ in moving_pairs),
            "the synthetic obstacle detour should produce a non-cardinal turn",
        )

    def test_obstacle_distance_calibration_scales_only_positive_distances(self) -> None:
        _, normal_map, obstacle_map = self._synthetic_maps()
        unscaled = plan_two_map_route(
            normal_map,
            obstacle_map,
            start=(8, 4),
            goal=(0, 4),
            initial_heading_deg=90.0,
            robot_radius_mm=0.0,
            safety_margin_mm=0.0,
            obstacle_distance_scale=1.0,
        )
        scaled = plan_two_map_route(
            normal_map,
            obstacle_map,
            start=(8, 4),
            goal=(0, 4),
            initial_heading_deg=90.0,
            robot_radius_mm=0.0,
            safety_margin_mm=0.0,
            obstacle_distance_scale=1000.0 / 1018.2,
        )

        def pairs(route):
            return [
                (float(turn), float(distance))
                for turn, distance in MOTION_PAIR_PATTERN.findall(route.command)
            ]

        unscaled_pairs = pairs(unscaled)
        scaled_pairs = pairs(scaled)
        self.assertEqual(
            [turn for turn, _ in scaled_pairs],
            [turn for turn, _ in unscaled_pairs],
        )
        for (_, original_distance), (_, corrected_distance) in zip(
            unscaled_pairs,
            scaled_pairs,
        ):
            if original_distance == 0.0:
                self.assertEqual(corrected_distance, 0.0)
            else:
                self.assertAlmostEqual(
                    corrected_distance,
                    original_distance * (1000.0 / 1018.2),
                    delta=0.11,
                )

        self.assertEqual(
            scaled.command.split(",[")[0],
            unscaled.command.split(",[")[0],
        )
        self.assertEqual(
            scaled.command.rsplit("],", 1)[1],
            unscaled.command.rsplit("],", 1)[1],
        )

    def test_checked_in_maze_png_two_map_smoke(self) -> None:
        project_root = Path(__file__).resolve().parents[1]
        image_file = project_root / "mazemappingtask2" / "maze.png"

        output = run_two_map_demo(DemoConfig(image_file=image_file))
        route = output.route

        self.assertTrue(route.succeeded)
        self.assertEqual(len(output.obstacle_map.detected_centres), 5)
        self.assert_cardinal_cell_path(route.normal_before_cells)
        self.assert_cardinal_cell_path(route.normal_after_cells)
        self.assertFalse(
            set(route.normal_before_cells) & output.normal_map.excluded_cells
        )
        self.assertFalse(
            set(route.normal_after_cells) & output.normal_map.excluded_cells
        )
        self.assertTrue(
            all(
                not route.obstacle_result.inflated_grid.is_occupied(point)
                for point in route.obstacle_result.grid_path
            )
        )
        self.assertTrue(
            all(
                0 <= point.x < output.obstacle_map.grid.width
                and 0 <= point.y < output.obstacle_map.grid.height
                for point in route.obstacle_result.grid_path
            )
        )
        self.assert_route_joins(output.normal_map, route)
        self.assert_valid_hybrid_command(route)
        self.assertEqual(
            route.full_waypoints[0],
            tuple(map(float, output.normal_map.calibration.centre(route.normal_before_cells[0]))),
        )
        self.assertEqual(
            route.full_waypoints[-1],
            tuple(map(float, output.normal_map.calibration.centre(route.normal_after_cells[-1]))),
        )
        self.assertEqual(output.original_preview_bgr.shape, output.source_bgr.shape)
        self.assertEqual(output.normal_preview_bgr.shape, output.rectified_bgr.shape)
        self.assertEqual(
            output.obstacle_preview_bgr.shape,
            output.obstacle_map.crop_bgr.shape,
        )

    def test_maz4_thin_wall_survives_rectification_and_reroutes_path(self) -> None:
        project_root = Path(__file__).resolve().parents[1]
        config = DemoConfig(
            image_file=project_root / "mazemappingtask2" / "maz4.png",
            obstacle_top_left=(0, 2),
            obstacle_size=5,
            entrance=Portal((3, 1), (3, 2)),
            exit=Portal((1, 7), (1, 6)),
            start=(6, 0),
            goal=(5, 8),
        )

        output = run_two_map_demo(config)
        unsafe_edge = frozenset(((3, 7), (4, 7)))

        self.assertTrue(output.normal_map.walls[unsafe_edge].blocked)
        self.assertNotIn(
            ((3, 7), (4, 7)),
            list(zip(
                output.route.normal_after_cells,
                output.route.normal_after_cells[1:],
            )),
        )
        self.assertIn((3, 8), output.route.normal_after_cells)
        self.assertIn((4, 8), output.route.normal_after_cells)

    def test_coordinate_preview_does_not_require_valid_portals(self) -> None:
        project_root = Path(__file__).resolve().parents[1]
        image_file = project_root / "mazemappingtask2" / "maze2.png"
        bad_portals = DemoConfig(
            image_file=image_file,
            entrance=Portal((6, 3), (5, 3)),
            exit=Portal((1, 6), (1, 5)),
        )

        preview = prepare_mapping_preview(bad_portals)

        self.assertEqual(preview.coordinate_grid_bgr.shape, preview.rectified_bgr.shape)
        self.assertIsNotNone(preview.clip_grid_result)
        self.assertIsNotNone(preview.calibration.homography)
        self.assertGreaterEqual(preview.clip_grid_result.inlier_count, 20)
        self.assertEqual(preview.calibration.centre((0, 0)), (41, 41))
        self.assertEqual(preview.calibration.centre((8, 8)), (284, 284))

    def test_coordinate_preview_can_still_use_fixed_grid_fallback(self) -> None:
        project_root = Path(__file__).resolve().parents[1]
        preview = prepare_mapping_preview(
            DemoConfig(
                image_file=project_root / "mazemappingtask2" / "maze2.png",
                grid_from_clips=False,
            )
        )

        self.assertIsNone(preview.clip_grid_result)
        self.assertIsNone(preview.calibration.homography)
        self.assertEqual(preview.calibration.centre((0, 0)), (44, 42))
        self.assertEqual(preview.calibration.centre((8, 8)), (284, 282))


if __name__ == "__main__":
    unittest.main()
