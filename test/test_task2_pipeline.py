import unittest

import cv2
import numpy as np

from computer.task4 import ContinuousPlanner, OccupancyGrid, PlannerConfig
from mazemappingtask2 import (
    cell_center,
    clear_endpoint_footprints,
    create_demo_course,
    create_occupancy_mask,
    format_hybrid_command,
    plan_hybrid_route,
    project_points_to_original,
    repair_task1_wall_gaps,
    rectify_course,
)


class Task2PipelineTests(unittest.TestCase):
    def test_short_wall_gaps_are_repaired_directionally(self) -> None:
        occupancy = np.zeros((30, 30), dtype=np.uint8)
        occupancy[10, 2:12] = 255
        occupancy[10, 16:27] = 255
        occupancy[2:12, 20] = 255
        occupancy[16:27, 20] = 255

        repaired = repair_task1_wall_gaps(occupancy, gap_pixels=7)

        self.assertTrue(np.all(repaired[10, 2:27] == 255))
        self.assertTrue(np.all(repaired[2:27, 20] == 255))
        self.assertEqual(int(repaired[14, 14]), 0)

    def test_visible_robot_footprints_can_be_removed_at_known_endpoints(self) -> None:
        occupancy = np.zeros((40, 40), dtype=np.uint8)
        cv2.circle(occupancy, (10, 20), 5, 255, -1)
        cv2.circle(occupancy, (30, 20), 5, 255, -1)
        occupancy[5, 5] = 255

        cleaned = clear_endpoint_footprints(
            occupancy,
            endpoints=((10, 20), (30, 20)),
            radius_pixels=6,
        )

        self.assertEqual(int(cleaned[20, 10]), 0)
        self.assertEqual(int(cleaned[20, 30]), 0)
        self.assertEqual(int(cleaned[5, 5]), 255)

    def test_path_points_project_back_to_original_camera_image(self) -> None:
        projected = project_points_to_original(
            [(0, 0), (99, 99), (50, 50)],
            original_shape=(200, 300, 3),
            rectified_shape=(100, 100, 3),
            board_corners=((20, 10), (280, 20), (270, 190), (30, 180)),
        )

        self.assertEqual(projected[0], (20, 10))
        self.assertEqual(projected[1], (270, 190))
        self.assertTrue(140 <= projected[2][0] <= 160)
        self.assertTrue(95 <= projected[2][1] <= 110)

    def test_cell_centres_use_row_column_order(self) -> None:
        self.assertEqual(cell_center((0, 0), (100, 200), 5), (22, 11))
        self.assertEqual(cell_center((4, 4), (100, 200), 5), (178, 89))

    def test_dark_objects_become_occupied(self) -> None:
        image = np.full((40, 40, 3), 255, dtype=np.uint8)
        cv2.circle(image, (20, 20), 8, (0, 0, 0), -1)

        occupancy = create_occupancy_mask(
            image,
            dark_threshold=100,
            thin_line_threshold=18,
            gamma=1.21,
        )

        self.assertEqual(int(occupancy[20, 20]), 255)
        self.assertEqual(int(occupancy[20, 8]), 0)

    def test_demo_image_has_required_three_part_route(self) -> None:
        image = create_demo_course(324)
        rectified = rectify_course(image, 324)
        occupancy = create_occupancy_mask(rectified)
        grid = OccupancyGrid.from_rows(occupancy > 0, resolution_mm=5.0)
        planner = ContinuousPlanner(
            PlannerConfig(robot_radius_mm=50.0, safety_margin_mm=10.0)
        )

        anchors = (
            (54, 270),
            (162, 234),
            (162, 90),
            (270, 54),
        )
        result = plan_hybrid_route(
            planner,
            grid,
            anchors,
        )

        self.assertTrue(result.succeeded, result.failure_message)
        self.assertEqual(len(result.segment_results), 3)
        self.assertGreaterEqual(len(result.grid_waypoints), 2)
        for segment_index in (0, 2):
            waypoints = result.segment_results[segment_index].grid_waypoints
            self.assertTrue(
                all(
                    first.x == second.x or first.y == second.y
                    for first, second in zip(waypoints, waypoints[1:])
                ),
                "normal maze segment contains an angled movement",
            )

        hybrid = format_hybrid_command(
            result,
            initial_heading_deg=90.0,
            image_shape=occupancy.shape,
            cell_count=9,
        )
        self.assertTrue(hybrid.startswith('"'))
        self.assertTrue(hybrid.endswith('";'))
        before, remainder = hybrid[1:-2].split(",[", maxsplit=1)
        obstacle, after = remainder.rsplit("],", maxsplit=1)
        self.assertTrue(set(before) <= set("lrf"))
        self.assertTrue(obstacle.startswith("("))
        self.assertTrue(set(after) <= set("lrf"))


if __name__ == "__main__":
    unittest.main()
