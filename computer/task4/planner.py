"""Collision-safe occupancy-grid planner for MTRN3100 Task 4.2.

The computer-vision program is responsible for creating a calibrated binary
occupancy mask. This module inflates that mask by the robot footprint, finds an
A* path, simplifies it into waypoints, and converts the waypoints to motion
commands suitable for transmission to the robot.

Image/grid coordinates use +x right and +y down. Millimetre coordinates use
+x right and +y up. Positive turn angles are anticlockwise.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import heapq
import math
from typing import Iterable, List, Optional, Sequence, Tuple, Union


@dataclass(frozen=True)
class GridPoint:
    x: int
    y: int


@dataclass(frozen=True)
class PointMM:
    x: float
    y: float


@dataclass(frozen=True)
class MotionCommand:
    """Rotate by ``turn_deg``, then drive ``distance_mm`` forwards."""

    turn_deg: float
    distance_mm: float


class OccupancyGrid:
    """Row-major binary occupancy data.

    A false/zero cell is free and a true/non-zero cell is occupied. Positions
    outside the supplied image are treated as occupied.
    """

    def __init__(
        self,
        width: int,
        height: int,
        resolution_mm: float,
        cells: Sequence[object],
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("occupancy-grid dimensions must be positive")
        if not math.isfinite(resolution_mm) or resolution_mm <= 0.0:
            raise ValueError("resolution_mm must be positive and finite")
        if len(cells) != width * height:
            raise ValueError("cells must contain exactly width * height values")

        self.width = int(width)
        self.height = int(height)
        self.resolution_mm = float(resolution_mm)
        self.cells: Tuple[bool, ...] = tuple(bool(value) for value in cells)

    @classmethod
    def from_rows(
        cls,
        rows: Sequence[Sequence[object]],
        resolution_mm: float,
    ) -> "OccupancyGrid":
        """Build from a 2-D list or NumPy/OpenCV mask.

        The mask must use zero for free space and non-zero for obstacles. If CV
        produces the reverse convention, invert it before calling this method.
        """

        height = len(rows)
        if height == 0:
            raise ValueError("occupancy mask must not be empty")
        width = len(rows[0])
        if width == 0:
            raise ValueError("occupancy mask rows must not be empty")

        flattened: List[object] = []
        for row in rows:
            if len(row) != width:
                raise ValueError("all occupancy-mask rows must have equal width")
            flattened.extend(row)
        return cls(width, height, resolution_mm, flattened)

    def in_bounds(self, point: GridPoint) -> bool:
        return 0 <= point.x < self.width and 0 <= point.y < self.height

    def is_occupied(self, point: GridPoint) -> bool:
        if not self.in_bounds(point):
            return True
        return self.cells[self._index(point)]

    def _index(self, point: GridPoint) -> int:
        return point.y * self.width + point.x


class PlanStatus(Enum):
    SUCCESS = "success"
    INVALID_GRID = "invalid occupancy grid or planner configuration"
    START_OUT_OF_BOUNDS = "start is outside the grid"
    GOAL_OUT_OF_BOUNDS = "goal is outside the grid"
    START_BLOCKED = "start is blocked after inflation"
    GOAL_BLOCKED = "goal is blocked after inflation"
    NO_PATH = "no collision-free path exists"


@dataclass(frozen=True)
class PlannerConfig:
    # Measure from the robot centre to its furthest outside corner.
    robot_radius_mm: float = 50.0
    safety_margin_mm: float = 10.0
    simplify_path: bool = True


@dataclass
class PlanResult:
    status: PlanStatus
    inflated_grid: Optional[OccupancyGrid] = None
    # Complete adjacent-cell A* path, including start and goal.
    grid_path: List[GridPoint] = field(default_factory=list)
    # Collision-checked simplified path, including start and goal.
    grid_waypoints: List[GridPoint] = field(default_factory=list)
    # Absolute Cartesian map coordinates in millimetres.
    waypoints_mm: List[PointMM] = field(default_factory=list)
    # Displacements between consecutive waypoints; start is excluded.
    relative_waypoints_mm: List[PointMM] = field(default_factory=list)

    @property
    def succeeded(self) -> bool:
        return self.status is PlanStatus.SUCCESS


PointInput = Union[GridPoint, Tuple[int, int]]


class ContinuousPlanner:
    """Inflated-grid A* planner with collision-safe path simplification."""

    _DIRECTIONS = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),            (1, 0),
        (-1, 1),  (0, 1),  (1, 1),
    )

    def __init__(self, config: PlannerConfig = PlannerConfig()) -> None:
        self.config = config

    def plan(
        self,
        grid: OccupancyGrid,
        start: PointInput,
        goal: PointInput,
    ) -> PlanResult:
        """Return a safe path from ``start`` to ``goal``.

        Start and goal may be ``GridPoint`` instances or ``(x, y)`` tuples in
        image coordinates.
        """

        start_point = self._point(start)
        goal_point = self._point(goal)
        if not self._valid_config():
            return PlanResult(PlanStatus.INVALID_GRID)
        if not grid.in_bounds(start_point):
            return PlanResult(PlanStatus.START_OUT_OF_BOUNDS)
        if not grid.in_bounds(goal_point):
            return PlanResult(PlanStatus.GOAL_OUT_OF_BOUNDS)

        inflated = self._inflate(
            grid,
            self.config.robot_radius_mm + self.config.safety_margin_mm,
        )
        if inflated.is_occupied(start_point):
            return PlanResult(PlanStatus.START_BLOCKED, inflated)
        if inflated.is_occupied(goal_point):
            return PlanResult(PlanStatus.GOAL_BLOCKED, inflated)

        grid_path = self._a_star(inflated, start_point, goal_point)
        if not grid_path:
            return PlanResult(PlanStatus.NO_PATH, inflated)

        grid_waypoints = (
            self._simplify(inflated, grid_path)
            if self.config.simplify_path
            else list(grid_path)
        )
        waypoints_mm = self._to_metric(inflated, grid_waypoints)
        relative_waypoints_mm = self._to_relative(waypoints_mm)
        return PlanResult(
            status=PlanStatus.SUCCESS,
            inflated_grid=inflated,
            grid_path=grid_path,
            grid_waypoints=grid_waypoints,
            waypoints_mm=waypoints_mm,
            relative_waypoints_mm=relative_waypoints_mm,
        )

    @staticmethod
    def make_motion_commands(
        waypoints_mm: Sequence[PointMM],
        initial_heading_deg: float,
    ) -> List[MotionCommand]:
        """Convert absolute waypoints into turn-then-drive commands.

        Heading is measured from +x. Positive angles are anticlockwise.
        """

        commands: List[MotionCommand] = []
        heading_deg = initial_heading_deg
        for previous, current in zip(waypoints_mm, waypoints_mm[1:]):
            dx = current.x - previous.x
            dy = current.y - previous.y
            distance_mm = math.hypot(dx, dy)
            if distance_mm == 0.0:
                continue
            target_heading_deg = math.degrees(math.atan2(dy, dx))
            commands.append(
                MotionCommand(
                    turn_deg=ContinuousPlanner._wrap_angle_deg(
                        target_heading_deg - heading_deg
                    ),
                    distance_mm=distance_mm,
                )
            )
            heading_deg = target_heading_deg
        return commands

    def _valid_config(self) -> bool:
        return (
            math.isfinite(self.config.robot_radius_mm)
            and math.isfinite(self.config.safety_margin_mm)
            and self.config.robot_radius_mm >= 0.0
            and self.config.safety_margin_mm >= 0.0
        )

    @staticmethod
    def _point(value: PointInput) -> GridPoint:
        if isinstance(value, GridPoint):
            return value
        if len(value) != 2:
            raise ValueError("a grid point must contain exactly x and y")
        return GridPoint(int(value[0]), int(value[1]))

    @staticmethod
    def _inflate(grid: OccupancyGrid, clearance_mm: float) -> OccupancyGrid:
        output = list(grid.cells)
        if clearance_mm <= 0.0:
            return OccupancyGrid(
                grid.width, grid.height, grid.resolution_mm, output
            )

        # Occupied pixels describe square areas, not zero-radius points, so the
        # half-cell diagonal makes the dilation conservatively collision-safe.
        dilation_mm = clearance_mm + grid.resolution_mm / math.sqrt(2.0)
        extent = math.ceil(dilation_mm / grid.resolution_mm)
        max_distance_squared = dilation_mm * dilation_mm
        offsets = [
            (dx, dy)
            for dy in range(-extent, extent + 1)
            for dx in range(-extent, extent + 1)
            if (dx * grid.resolution_mm) ** 2
            + (dy * grid.resolution_mm) ** 2
            <= max_distance_squared
        ]

        occupied = [
            GridPoint(x, y)
            for y in range(grid.height)
            for x in range(grid.width)
            if grid.cells[y * grid.width + x]
        ]
        for obstacle in occupied:
            for dx, dy in offsets:
                x = obstacle.x + dx
                y = obstacle.y + dy
                if 0 <= x < grid.width and 0 <= y < grid.height:
                    output[y * grid.width + x] = True

        return OccupancyGrid(
            grid.width, grid.height, grid.resolution_mm, output
        )

    @classmethod
    def _a_star(
        cls,
        grid: OccupancyGrid,
        start: GridPoint,
        goal: GridPoint,
    ) -> List[GridPoint]:
        start_index = grid._index(start)
        goal_index = grid._index(goal)
        cell_count = grid.width * grid.height
        g_score = [math.inf] * cell_count
        parent = [-1] * cell_count
        closed = [False] * cell_count
        g_score[start_index] = 0.0

        counter = 0
        open_nodes = [(cls._octile(start, goal), counter, start_index)]
        while open_nodes:
            _, _, current_index = heapq.heappop(open_nodes)
            if closed[current_index]:
                continue
            closed[current_index] = True
            if current_index == goal_index:
                break

            current = cls._point_from_index(grid, current_index)
            for dx, dy in cls._DIRECTIONS:
                neighbour = GridPoint(current.x + dx, current.y + dy)
                if grid.is_occupied(neighbour):
                    continue

                # Do not squeeze diagonally between touching obstacles.
                if dx != 0 and dy != 0:
                    if grid.is_occupied(GridPoint(current.x + dx, current.y)):
                        continue
                    if grid.is_occupied(GridPoint(current.x, current.y + dy)):
                        continue

                neighbour_index = grid._index(neighbour)
                if closed[neighbour_index]:
                    continue
                step_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current_index] + step_cost
                if tentative_g >= g_score[neighbour_index]:
                    continue

                parent[neighbour_index] = current_index
                g_score[neighbour_index] = tentative_g
                counter += 1
                heapq.heappush(
                    open_nodes,
                    (
                        tentative_g + cls._octile(neighbour, goal),
                        counter,
                        neighbour_index,
                    ),
                )

        if start_index != goal_index and parent[goal_index] < 0:
            return []

        path = []
        current_index = goal_index
        while True:
            path.append(cls._point_from_index(grid, current_index))
            if current_index == start_index:
                break
            current_index = parent[current_index]
        path.reverse()
        return path

    @staticmethod
    def _octile(start: GridPoint, goal: GridPoint) -> float:
        dx = abs(start.x - goal.x)
        dy = abs(start.y - goal.y)
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        return straight + math.sqrt(2.0) * diagonal

    @staticmethod
    def _point_from_index(grid: OccupancyGrid, index: int) -> GridPoint:
        return GridPoint(index % grid.width, index // grid.width)

    @classmethod
    def _has_line_of_sight(
        cls,
        grid: OccupancyGrid,
        start: GridPoint,
        end: GridPoint,
    ) -> bool:
        """Check every grid cell touched by the segment (supercover line)."""

        x, y = start.x, start.y
        delta_x = abs(end.x - start.x)
        delta_y = abs(end.y - start.y)
        x_step = 1 if end.x > start.x else -1 if end.x < start.x else 0
        y_step = 1 if end.y > start.y else -1 if end.y < start.y else 0
        doubled_x = 2 * delta_x
        doubled_y = 2 * delta_y
        error = delta_x - delta_y
        remaining = 1 + delta_x + delta_y

        while remaining > 0:
            if grid.is_occupied(GridPoint(x, y)):
                return False
            if x == end.x and y == end.y:
                return True

            if error > 0:
                x += x_step
                error -= doubled_y
            elif error < 0:
                y += y_step
                error += doubled_x
            else:
                # The segment passes exactly through a cell corner; both sides
                # must be free to prevent diagonal corner cutting.
                if grid.is_occupied(GridPoint(x + x_step, y)):
                    return False
                if grid.is_occupied(GridPoint(x, y + y_step)):
                    return False
                x += x_step
                y += y_step
                error += doubled_x - doubled_y
                remaining -= 1
            remaining -= 1
        return False

    @classmethod
    def _simplify(
        cls,
        grid: OccupancyGrid,
        path: Sequence[GridPoint],
    ) -> List[GridPoint]:
        if len(path) <= 2:
            return list(path)

        waypoints = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            furthest = len(path) - 1
            while (
                furthest > anchor + 1
                and not cls._has_line_of_sight(
                    grid, path[anchor], path[furthest]
                )
            ):
                furthest -= 1
            waypoints.append(path[furthest])
            anchor = furthest
        return waypoints

    @staticmethod
    def _to_metric(
        grid: OccupancyGrid,
        points: Iterable[GridPoint],
    ) -> List[PointMM]:
        return [
            PointMM(
                x=(point.x + 0.5) * grid.resolution_mm,
                y=(grid.height - point.y - 0.5) * grid.resolution_mm,
            )
            for point in points
        ]

    @staticmethod
    def _to_relative(points: Sequence[PointMM]) -> List[PointMM]:
        return [
            PointMM(current.x - previous.x, current.y - previous.y)
            for previous, current in zip(points, points[1:])
        ]

    @staticmethod
    def _wrap_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0
