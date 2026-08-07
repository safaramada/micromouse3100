import math
import unittest

from computer.task4 import (
    ContinuousPlanner,
    GridPoint,
    OccupancyGrid,
    PlanStatus,
    PlannerConfig,
    PointMM,
)


class ContinuousPlannerTests(unittest.TestCase):
    @staticmethod
    def zero_clearance_config() -> PlannerConfig:
        return PlannerConfig(robot_radius_mm=0.0, safety_margin_mm=0.0)

    def test_open_map_produces_straight_path(self) -> None:
        rows = [[0] * 20 for _ in range(20)]
        grid = OccupancyGrid.from_rows(rows, resolution_mm=10.0)
        planner = ContinuousPlanner(self.zero_clearance_config())

        result = planner.plan(grid, (1, 18), (18, 1))

        self.assertTrue(result.succeeded)
        self.assertEqual(result.grid_path[0], GridPoint(1, 18))
        self.assertEqual(result.grid_path[-1], GridPoint(18, 1))
        self.assertEqual(len(result.grid_waypoints), 2)
        self.assertEqual(result.relative_waypoints_mm, [PointMM(170.0, 170.0)])

    def test_border_to_border_line_does_not_step_outside(self) -> None:
        grid = OccupancyGrid.from_rows([[0] * 20 for _ in range(20)], 10.0)
        result = ContinuousPlanner(self.zero_clearance_config()).plan(
            grid, (0, 19), (19, 0)
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(len(result.grid_waypoints), 2)

    def test_wall_gap_is_found_and_obstacle_is_avoided(self) -> None:
        rows = [[0] * 30 for _ in range(20)]
        for y in range(20):
            if y < 8 or y > 12:
                rows[y][15] = 1
        config = PlannerConfig(robot_radius_mm=4.0, safety_margin_mm=0.0)

        result = ContinuousPlanner(config).plan(
            OccupancyGrid.from_rows(rows, 10.0), (2, 3), (27, 3)
        )

        self.assertTrue(result.succeeded)
        self.assertGreaterEqual(len(result.grid_waypoints), 3)
        self.assertTrue(
            all(
                not result.inflated_grid.is_occupied(point)
                for point in result.grid_path
            )
        )

    def test_diagonal_corner_cutting_is_rejected(self) -> None:
        grid = OccupancyGrid.from_rows([[0, 1], [1, 0]], 10.0)
        result = ContinuousPlanner(self.zero_clearance_config()).plan(
            grid, (0, 0), (1, 1)
        )
        self.assertIs(result.status, PlanStatus.NO_PATH)

    def test_inflation_rejects_unsafe_start(self) -> None:
        rows = [[0] * 12 for _ in range(12)]
        rows[6][6] = 1
        config = PlannerConfig(robot_radius_mm=15.0, safety_margin_mm=0.0)

        result = ContinuousPlanner(config).plan(
            OccupancyGrid.from_rows(rows, 10.0), (7, 6), (11, 11)
        )
        self.assertIs(result.status, PlanStatus.START_BLOCKED)

    def test_waypoints_convert_to_turn_and_drive_commands(self) -> None:
        commands = ContinuousPlanner.make_motion_commands(
            [PointMM(0.0, 0.0), PointMM(100.0, 0.0), PointMM(100.0, 100.0)],
            initial_heading_deg=90.0,
        )

        self.assertEqual(len(commands), 2)
        self.assertAlmostEqual(commands[0].turn_deg, -90.0)
        self.assertAlmostEqual(commands[0].distance_mm, 100.0)
        self.assertAlmostEqual(commands[1].turn_deg, 90.0)
        self.assertAlmostEqual(commands[1].distance_mm, 100.0)

    def test_invalid_config_is_reported(self) -> None:
        grid = OccupancyGrid.from_rows([[0]], 10.0)
        config = PlannerConfig(robot_radius_mm=math.nan)
        result = ContinuousPlanner(config).plan(grid, (0, 0), (0, 0))
        self.assertIs(result.status, PlanStatus.INVALID_GRID)


if __name__ == "__main__":
    unittest.main()
