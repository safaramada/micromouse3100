"""Image-processing helpers for MTRN3100 Task 4.2.

Task 4.2 replaces a 5 x 5-cell area of the normal 9 x 9 maze with a continuous
obstacle course. This module rectifies the complete maze camera image,
extracts normal walls and cylindrical obstacles as one occupancy mask, and
draws the trajectory returned by the computer-side planner.

Generative AI disclosure: OpenAI Codex assisted with this implementation.
AI-assisted sections are identified by inline comments.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from computer.task4 import ContinuousPlanner, GridPoint, MotionCommand

from mazemapping.mask_maze import (
    DARK_THRESHOLD as TASK1_DARK_THRESHOLD,
    GAMMA as TASK1_GAMMA,
    THIN_LINE_THRESHOLD as TASK1_THIN_LINE_THRESHOLD,
    create_board_mask,
    create_maze_masks,
)


Point = Tuple[int, int]


def _task1_grid_bounds(
    board_mask: np.ndarray,
    inset_x_fraction: float,
    inset_y_fraction: float,
) -> Tuple[int, int, int, int]:
    """Match ``path_planning_nodes.calculate_grid_bounds`` package-safely."""
    if not 0.0 <= inset_x_fraction < 0.5:
        raise ValueError("horizontal grid inset must be between 0 and 0.5")
    if not 0.0 <= inset_y_fraction < 0.5:
        raise ValueError("vertical grid inset must be between 0 and 0.5")

    board_y, board_x = np.where(board_mask == 255)
    if board_x.size == 0 or board_y.size == 0:
        raise ValueError("the board mask contains no playable region")

    raw_left, raw_right = int(board_x.min()), int(board_x.max())
    raw_top, raw_bottom = int(board_y.min()), int(board_y.max())
    inset_x = int(round((raw_right - raw_left) * inset_x_fraction))
    inset_y = int(round((raw_bottom - raw_top) * inset_y_fraction))
    return (
        raw_left + inset_x,
        raw_top + inset_y,
        raw_right - inset_x,
        raw_bottom - inset_y,
    )


@dataclass
class HybridRoute:
    """Combined normal-maze, obstacle-course, normal-maze route."""

    segment_results: List[object] = field(default_factory=list)
    grid_path: List[object] = field(default_factory=list)
    grid_waypoints: List[object] = field(default_factory=list)
    waypoints_mm: List[object] = field(default_factory=list)
    inflated_grid: Optional[object] = None
    failure_message: Optional[str] = None

    @property
    def succeeded(self) -> bool:
        return self.failure_message is None and bool(self.segment_results)


class CardinalMazePlanner(ContinuousPlanner):
    """Occupancy-grid planner restricted to normal maze movements.

    Normal maze corridors are represented by horizontal and vertical motion
    only. The planner therefore searches four-connected neighbours and keeps
    only the endpoints of straight cardinal runs. It never creates a diagonal
    shortcut across a normal maze cell or an arbitrary-angle wall approach.
    """

    _DIRECTIONS = (
        (0, -1),
        (1, 0),
        (0, 1),
        (-1, 0),
    )
    _TURN_PENALTY = 12.0

    @staticmethod
    def _octile(start: GridPoint, goal: GridPoint) -> float:
        """Manhattan heuristic for four-connected A*."""
        return float(abs(start.x - goal.x) + abs(start.y - goal.y))

    @classmethod
    def _a_star(
        cls,
        grid: object,
        start: GridPoint,
        goal: GridPoint,
    ) -> List[GridPoint]:
        """Four-connected A* that prefers long straight corridor runs."""
        start_index = grid._index(start)
        goal_index = grid._index(goal)
        if start_index == goal_index:
            return [start]

        # State includes travel direction. A turn penalty resolves the many
        # equal-distance pixel paths in favour of the one with fewer turns.
        start_state = (start_index, -1)
        costs = {start_state: 0.0}
        parents = {start_state: None}
        frontier = [(cls._octile(start, goal), 0, start_state)]
        counter = 0
        goal_state = None

        while frontier:
            estimated_cost, _, state = heapq.heappop(frontier)
            current_index, previous_direction = state
            current_cost = costs.get(state, math.inf)
            current = cls._point_from_index(grid, current_index)
            if estimated_cost > current_cost + cls._octile(current, goal) + 1e-9:
                continue
            if current_index == goal_index:
                goal_state = state
                break

            for direction, (dx, dy) in enumerate(cls._DIRECTIONS):
                neighbour = GridPoint(current.x + dx, current.y + dy)
                if grid.is_occupied(neighbour):
                    continue
                neighbour_index = grid._index(neighbour)
                turn_cost = (
                    cls._TURN_PENALTY
                    if previous_direction >= 0 and direction != previous_direction
                    else 0.0
                )
                new_cost = current_cost + 1.0 + turn_cost
                neighbour_state = (neighbour_index, direction)
                if new_cost + 1e-9 >= costs.get(neighbour_state, math.inf):
                    continue

                costs[neighbour_state] = new_cost
                parents[neighbour_state] = state
                counter += 1
                heapq.heappush(
                    frontier,
                    (
                        new_cost + cls._octile(neighbour, goal),
                        counter,
                        neighbour_state,
                    ),
                )

        if goal_state is None:
            return []

        path = []
        state = goal_state
        while state is not None:
            path.append(cls._point_from_index(grid, state[0]))
            state = parents[state]
        path.reverse()
        return path

    @classmethod
    def _simplify(
        cls,
        grid: object,
        path: Sequence[GridPoint],
    ) -> List[GridPoint]:
        """Collapse adjacent pixels into collision-checked straight runs."""
        if len(path) <= 2:
            return list(path)

        waypoints = [path[0]]
        previous_dx = path[1].x - path[0].x
        previous_dy = path[1].y - path[0].y
        for index in range(1, len(path) - 1):
            dx = path[index + 1].x - path[index].x
            dy = path[index + 1].y - path[index].y
            if (dx, dy) != (previous_dx, previous_dy):
                waypoint = path[index]
                if not cls._has_line_of_sight(grid, waypoints[-1], waypoint):
                    raise RuntimeError("normal-maze straight run is not collision-free")
                waypoints.append(waypoint)
                previous_dx, previous_dy = dx, dy

        if not cls._has_line_of_sight(grid, waypoints[-1], path[-1]):
            raise RuntimeError("normal-maze final straight run is not collision-free")
        waypoints.append(path[-1])
        return waypoints


def rectify_course(
    image_bgr: np.ndarray,
    output_pixels: int,
    board_corners: Optional[Sequence[Sequence[float]]] = None,
    interpolation: int = cv2.INTER_AREA,
) -> np.ndarray:
    """Return a square, top-down view of the complete 9 x 9 maze.

    ``board_corners`` must be ordered top-left, top-right, bottom-right,
    bottom-left in the original image.  When it is ``None``, the complete
    input image is used.  For marking, measure these four points once for the
    fixed camera setup rather than tuning them after each photograph. The
    obstacle-course region remains part of this same rectified image.
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("the supplied course image is empty")
    if output_pixels < 2:
        raise ValueError("output_pixels must be at least 2")

    height, width = image_bgr.shape[:2]
    if board_corners is None:
        source = np.array(
            [
                [0.0, 0.0],
                [float(width - 1), 0.0],
                [float(width - 1), float(height - 1)],
                [0.0, float(height - 1)],
            ],
            dtype=np.float32,
        )
    else:
        source = np.asarray(board_corners, dtype=np.float32)
        if source.shape != (4, 2):
            raise ValueError("board_corners must contain four (x, y) points")
        if not np.isfinite(source).all():
            raise ValueError("board_corners must contain finite coordinates")

    maximum = float(output_pixels - 1)
    destination = np.array(
        [[0.0, 0.0], [maximum, 0.0], [maximum, maximum], [0.0, maximum]],
        dtype=np.float32,
    )

    # AI-assisted implementation: perspective calibration of the hard-coded
    # full maze, including its hard-coded 5 x 5 obstacle-course region.
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(
        image_bgr,
        transform,
        (output_pixels, output_pixels),
        flags=interpolation,
        borderMode=cv2.BORDER_REPLICATE,
    )


def project_points_to_original(
    points: Iterable[object],
    original_shape: Sequence[int],
    rectified_shape: Sequence[int],
    board_corners: Optional[Sequence[Sequence[float]]] = None,
) -> List[Point]:
    """Project rectified grid points back into the camera-image coordinates."""
    if len(original_shape) < 2 or len(rectified_shape) < 2:
        raise ValueError("image shapes must contain height and width")

    original_height, original_width = map(int, original_shape[:2])
    rectified_height, rectified_width = map(int, rectified_shape[:2])
    if min(original_height, original_width, rectified_height, rectified_width) < 2:
        raise ValueError("image dimensions must be at least 2 pixels")

    if board_corners is None:
        original_corners = np.array(
            [
                [0.0, 0.0],
                [float(original_width - 1), 0.0],
                [float(original_width - 1), float(original_height - 1)],
                [0.0, float(original_height - 1)],
            ],
            dtype=np.float32,
        )
    else:
        original_corners = np.asarray(board_corners, dtype=np.float32)
        if original_corners.shape != (4, 2):
            raise ValueError("board_corners must contain four (x, y) points")
        if not np.isfinite(original_corners).all():
            raise ValueError("board_corners must contain finite coordinates")

    rectified_corners = np.array(
        [
            [0.0, 0.0],
            [float(rectified_width - 1), 0.0],
            [float(rectified_width - 1), float(rectified_height - 1)],
            [0.0, float(rectified_height - 1)],
        ],
        dtype=np.float32,
    )
    inverse_transform = cv2.getPerspectiveTransform(
        rectified_corners,
        original_corners,
    )

    coordinates = [
        (float(point.x), float(point.y))
        if hasattr(point, "x") and hasattr(point, "y")
        else (float(point[0]), float(point[1]))
        for point in points
    ]
    if not coordinates:
        return []

    projected = cv2.perspectiveTransform(
        np.asarray(coordinates, dtype=np.float32).reshape((-1, 1, 2)),
        inverse_transform,
    ).reshape((-1, 2))
    return [
        (int(round(float(x))), int(round(float(y))))
        for x, y in projected
    ]


def overlay_trajectory_on_original(
    original_bgr: np.ndarray,
    rectified_shape: Sequence[int],
    grid_path: Iterable[object],
    waypoints: Sequence[object],
    board_corners: Optional[Sequence[Sequence[float]]] = None,
) -> np.ndarray:
    """Draw the planned trajectory on the untouched camera photograph."""
    if original_bgr is None or original_bgr.size == 0:
        raise ValueError("the original camera image is empty")

    display = original_bgr.copy()
    projected_path = project_points_to_original(
        grid_path,
        display.shape,
        rectified_shape,
        board_corners,
    )
    projected_waypoints = project_points_to_original(
        waypoints,
        display.shape,
        rectified_shape,
        board_corners,
    )

    scale = max(display.shape[:2]) / max(map(int, rectified_shape[:2]))
    detailed_thickness = max(1, int(round(scale)))
    waypoint_thickness = max(2, int(round(3.0 * scale)))
    waypoint_radius = max(3, int(round(4.0 * scale)))

    # AI-assisted implementation: inverse homography preserves alignment with
    # angled camera images instead of drawing rectified coordinates directly.
    if len(projected_path) >= 2:
        cv2.polylines(
            display,
            [np.asarray(projected_path, dtype=np.int32)],
            False,
            (0, 200, 255),
            detailed_thickness,
            cv2.LINE_AA,
        )
    if len(projected_waypoints) >= 2:
        cv2.polylines(
            display,
            [np.asarray(projected_waypoints, dtype=np.int32)],
            False,
            (0, 210, 0),
            waypoint_thickness,
            cv2.LINE_AA,
        )
    for point in projected_waypoints:
        cv2.circle(
            display,
            point,
            waypoint_radius,
            (255, 120, 0),
            -1,
            cv2.LINE_AA,
        )

    return display


def create_occupancy_mask(
    rectified_bgr: np.ndarray,
    dark_threshold: int = TASK1_DARK_THRESHOLD,
    thin_line_threshold: int = TASK1_THIN_LINE_THRESHOLD,
    gamma: float = TASK1_GAMMA,
) -> np.ndarray:
    """Create Task 2 occupancy using the existing Task 1 masking pipeline.

    This deliberately delegates to :func:`mazemapping.mask_maze.create_maze_masks`
    so both tasks use the same CLAHE enhancement, gamma correction, normal
    dark-wall threshold, horizontal/vertical thin-wall recovery, gap closing,
    connected-component filtering, octagonal board mask, and red overlay.
    Cylinders are retained by the same main dark-object threshold.
    """
    if rectified_bgr is None or rectified_bgr.size == 0:
        raise ValueError("the rectified course image is empty")
    if not 0 <= dark_threshold <= 255:
        raise ValueError("dark_threshold must be between 0 and 255")
    if not 0 <= thin_line_threshold <= 255:
        raise ValueError("thin_line_threshold must be between 0 and 255")
    if gamma <= 0:
        raise ValueError("gamma must be greater than zero")

    # AI-assisted integration: reuse the Task 1 CV output without duplicating
    # or subtly changing its image-colouring/masking implementation.
    masks = create_maze_masks(
        rectified_bgr,
        dark_threshold=dark_threshold,
        thin_line_threshold=thin_line_threshold,
        gamma=gamma,
    )
    return masks["06_occupancy_obstacles_white.png"]


def create_task1_mask_outputs(
    rectified_bgr: np.ndarray,
    dark_threshold: int = TASK1_DARK_THRESHOLD,
    thin_line_threshold: int = TASK1_THIN_LINE_THRESHOLD,
    gamma: float = TASK1_GAMMA,
) -> dict[str, np.ndarray]:
    """Expose all Task 1 intermediate images for notebook visual checks."""
    return create_maze_masks(
        rectified_bgr,
        dark_threshold=dark_threshold,
        thin_line_threshold=thin_line_threshold,
        gamma=gamma,
    )


def clear_endpoint_footprints(
    occupancy_mask: np.ndarray,
    endpoints: Sequence[Point],
    radius_pixels: int,
) -> np.ndarray:
    """Remove visible start/goal robots from a photograph-derived mask.

    The start and goal coordinates are supplied by the assessment, so their
    local robot footprints are not environmental obstacles. This operation is
    intentionally limited to circular regions around those two known points;
    cylinders elsewhere remain occupied.
    """
    if occupancy_mask.ndim != 2:
        raise ValueError("occupancy_mask must be a two-dimensional image")
    if radius_pixels < 0:
        raise ValueError("radius_pixels must be non-negative")

    cleaned = occupancy_mask.copy()
    # Use a span much longer than a robot body so round/irregular robot blobs
    # cannot survive the directional opening, while maze walls do.
    line_length = max(3, 4 * int(radius_pixels) + 1)
    horizontal_walls = cv2.morphologyEx(
        occupancy_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (line_length, 1)),
    )
    vertical_walls = cv2.morphologyEx(
        occupancy_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, line_length)),
    )
    linear_walls = cv2.bitwise_or(horizontal_walls, vertical_walls)
    for x, y in endpoints:
        if not (0 <= int(x) < cleaned.shape[1] and 0 <= int(y) < cleaned.shape[0]):
            raise ValueError("each endpoint must lie inside the occupancy mask")
        cv2.circle(
            cleaned,
            (int(x), int(y)),
            int(radius_pixels),
            0,
            -1,
            cv2.LINE_8,
        )
    # Restore long wall lines that cross an endpoint clearing circle. Round
    # robot bodies disappear under the long directional opening.
    cleaned = cv2.bitwise_or(cleaned, linear_walls)
    return cleaned


def repair_task1_wall_gaps(
    occupancy_mask: np.ndarray,
    gap_pixels: int = 7,
) -> np.ndarray:
    """Reconnect short tape gaps in Task 1's horizontal/vertical walls.

    This is deliberately applied after the unchanged Task 1 mask. Directional
    closing joins only aligned wall pixels; the small default span does not
    bridge normal corridors or the spaces between cylindrical obstacles.
    """
    if occupancy_mask.ndim != 2:
        raise ValueError("occupancy_mask must be a two-dimensional image")
    if gap_pixels <= 0 or gap_pixels % 2 == 0:
        raise ValueError("gap_pixels must be a positive odd integer")
    if gap_pixels == 1:
        return occupancy_mask.copy()

    horizontal = cv2.morphologyEx(
        occupancy_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (gap_pixels, 1)),
    )
    vertical = cv2.morphologyEx(
        occupancy_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, gap_pixels)),
    )
    return cv2.bitwise_or(
        occupancy_mask,
        cv2.bitwise_or(horizontal, vertical),
    )


def cell_center(
    cell: Tuple[int, int],
    image_shape: Sequence[int],
    cell_count: int = 9,
    inset_x_fraction: float = 0.08,
    inset_y_fraction: float = 0.08,
) -> Point:
    """Return a Task 1-aligned zero-indexed maze-cell centre.

    The lattice uses the same detected octagonal board bounds and 0.08 insets
    as ``mazemapping/path_planning_nodes.py`` instead of distributing cells
    across the complete camera-image rectangle.
    """
    if cell_count <= 0:
        raise ValueError("cell_count must be positive")
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")

    row, column = int(cell[0]), int(cell[1])
    if not (0 <= row < cell_count and 0 <= column < cell_count):
        raise ValueError(
            f"cell must be inside the 0..{cell_count - 1} row/column range"
        )

    height, width = int(image_shape[0]), int(image_shape[1])
    board_mask, _ = create_board_mask(height, width)
    left, top, right, bottom = _task1_grid_bounds(
        board_mask,
        inset_x_fraction,
        inset_y_fraction,
    )
    x_positions = np.linspace(left, right, cell_count)
    y_positions = np.linspace(top, bottom, cell_count)
    x = int(round(float(x_positions[column])))
    y = int(round(float(y_positions[row])))
    return x, y


def grid_to_mask(grid: object) -> np.ndarray:
    """Convert a ``computer.task4.OccupancyGrid`` to an 8-bit mask."""
    width = int(getattr(grid, "width"))
    height = int(getattr(grid, "height"))
    cells = np.asarray(getattr(grid, "cells"), dtype=np.uint8)
    if cells.size != width * height:
        raise ValueError("grid cells do not match its width and height")
    return cells.reshape((height, width)) * 255


def create_mapping_overlay(
    rectified_bgr: np.ndarray,
    occupancy_mask: np.ndarray,
) -> np.ndarray:
    """Show Task 1 detections without colouring outside-board pixels red."""
    if rectified_bgr.shape[:2] != occupancy_mask.shape[:2]:
        raise ValueError("image and occupancy mask dimensions must match")

    board_mask, _ = create_board_mask(*occupancy_mask.shape)
    outside_board = board_mask == 0
    detected_obstacles = (occupancy_mask != 0) & (board_mask != 0)

    display = rectified_bgr.copy()
    grey_layer = display.copy()
    grey_layer[outside_board] = (90, 90, 90)
    display = cv2.addWeighted(grey_layer, 0.48, display, 0.52, 0.0)

    obstacle_layer = display.copy()
    obstacle_layer[detected_obstacles] = (0, 0, 255)
    return cv2.addWeighted(obstacle_layer, 0.38, display, 0.62, 0.0)


def _append_without_duplicate(target: List[object], values: Iterable[object]) -> None:
    """Append a path segment without repeating its shared endpoint."""
    values_list = list(values)
    if target and values_list and target[-1] == values_list[0]:
        values_list = values_list[1:]
    target.extend(values_list)


def plan_hybrid_route(
    planner: object,
    grid: object,
    anchors: Sequence[Point],
    segment_names: Sequence[str] = (
        "normal maze before obstacle course",
        "continuous obstacle course",
        "normal maze after obstacle course",
    ),
    normal_maze_planner: Optional[object] = None,
) -> HybridRoute:
    """Plan required consecutive route segments through the whole maze.

    Four anchors represent start, obstacle entrance, obstacle exit, and goal.
    Planning the three segments separately guarantees that a globally shorter
    route cannot skip the designated continuous obstacle-course section.
    """
    if len(anchors) != 4:
        raise ValueError("anchors must contain start, entrance, exit, and goal")
    if len(segment_names) != 3:
        raise ValueError("segment_names must contain exactly three labels")

    # The continuous planner is used only between obstacle entrance and exit.
    # Normal maze segments use a cardinal planner unless one is supplied.
    if normal_maze_planner is None:
        normal_maze_planner = CardinalMazePlanner(planner.config)
    segment_planners = (normal_maze_planner, planner, normal_maze_planner)

    route = HybridRoute()
    for index, (start, goal) in enumerate(zip(anchors, anchors[1:])):
        result = segment_planners[index].plan(grid, start, goal)
        route.segment_results.append(result)
        if route.inflated_grid is None:
            route.inflated_grid = result.inflated_grid
        if not result.succeeded:
            route.failure_message = (
                f"No path through {segment_names[index]}: "
                f"{result.status.value}"
            )
            return route

        # AI-assisted implementation: preserve each segment's independently
        # collision-checked simplification so no shortcut can bypass an anchor.
        _append_without_duplicate(route.grid_path, result.grid_path)
        _append_without_duplicate(route.grid_waypoints, result.grid_waypoints)
        _append_without_duplicate(route.waypoints_mm, result.waypoints_mm)

    return route


def overlay_trajectory(
    rectified_bgr: np.ndarray,
    inflated_mask: np.ndarray,
    grid_path: Iterable[object],
    waypoints: Sequence[object],
    occupancy_mask: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Draw detected obstacles, clearance buffer, path, and waypoints.

    Red means a wall/cylinder detected by the reused Task 1 mask. Light amber
    means free floor reserved as robot-radius clearance; it is not a detected
    obstacle. Pixels outside the octagonal board are grey.
    """
    if rectified_bgr.shape[:2] != inflated_mask.shape[:2]:
        raise ValueError("image and inflated mask dimensions must match")
    if occupancy_mask is not None and occupancy_mask.shape != inflated_mask.shape:
        raise ValueError("occupancy and inflated mask dimensions must match")

    if occupancy_mask is None:
        occupancy_mask = inflated_mask

    display = create_mapping_overlay(rectified_bgr, occupancy_mask)
    board_mask, _ = create_board_mask(*inflated_mask.shape)
    clearance_only = (
        (inflated_mask != 0)
        & (occupancy_mask == 0)
        & (board_mask != 0)
    )
    clearance_layer = display.copy()
    clearance_layer[clearance_only] = (0, 190, 255)
    display = cv2.addWeighted(clearance_layer, 0.16, display, 0.84, 0.0)

    full_path = [(int(point.x), int(point.y)) for point in grid_path]
    if len(full_path) >= 2:
        cv2.polylines(
            display,
            [np.asarray(full_path, dtype=np.int32)],
            False,
            (0, 200, 255),
            1,
            cv2.LINE_AA,
        )

    simplified = [(int(point.x), int(point.y)) for point in waypoints]
    if len(simplified) >= 2:
        cv2.polylines(
            display,
            [np.asarray(simplified, dtype=np.int32)],
            False,
            (0, 210, 0),
            3,
            cv2.LINE_AA,
        )
    for point in simplified:
        cv2.circle(display, point, 4, (255, 120, 0), -1, cv2.LINE_AA)

    if simplified:
        cv2.circle(display, simplified[0], 6, (255, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(display, simplified[-1], 6, (255, 0, 255), -1, cv2.LINE_AA)

    return display


def format_motion_commands(commands: Iterable[object]) -> str:
    """Format turn-then-drive commands for copying into Arduino source."""
    command_list = list(commands)
    lines = [
        "// {turn_degrees, forward_distance_mm}",
        f"const MotionCommand TASK2_PATH[{len(command_list)}] = {{",
    ]
    for command in command_list:
        lines.append(
            "    {"
            f"{float(command.turn_deg):.2f}f, "
            f"{float(command.distance_mm):.1f}f"
            "},"
        )
    lines.append("};")
    return "\n".join(lines)


def _nearest_lattice_cell(
    point: object,
    image_shape: Sequence[int],
    cell_count: int,
) -> Tuple[int, int]:
    """Return the nearest Task 1 lattice ``(row, column)``."""
    point_x = int(point.x) if hasattr(point, "x") else int(point[0])
    point_y = int(point.y) if hasattr(point, "y") else int(point[1])
    best = None
    for row in range(cell_count):
        for column in range(cell_count):
            centre_x, centre_y = cell_center(
                (row, column),
                image_shape,
                cell_count,
            )
            candidate = (
                (centre_x - point_x) ** 2 + (centre_y - point_y) ** 2,
                row,
                column,
            )
            if best is None or candidate < best:
                best = candidate
    return int(best[1]), int(best[2])


def _normal_segment_directions(
    waypoints: Sequence[object],
    image_shape: Sequence[int],
    cell_count: int,
) -> List[float]:
    """Convert a cardinal pixel path to one heading per maze-cell edge."""
    cells: List[Tuple[int, int]] = []
    for waypoint in waypoints:
        cell = _nearest_lattice_cell(waypoint, image_shape, cell_count)
        if not cells or cells[-1] != cell:
            cells.append(cell)

    headings: List[float] = []
    for first, second in zip(cells, cells[1:]):
        row_delta = second[0] - first[0]
        column_delta = second[1] - first[1]
        if row_delta and column_delta:
            raise ValueError("normal-maze path changes row and column together")
        if column_delta > 0:
            headings.extend([0.0] * column_delta)
        elif column_delta < 0:
            headings.extend([180.0] * abs(column_delta))
        elif row_delta > 0:
            headings.extend([-90.0] * row_delta)
        elif row_delta < 0:
            headings.extend([90.0] * abs(row_delta))
    return headings


def _lfr_for_headings(
    headings: Sequence[float],
    initial_heading_deg: float,
) -> Tuple[str, float]:
    """Encode cardinal headings as turns followed by one ``f`` per cell."""
    current_heading = initial_heading_deg
    output: List[str] = []
    for target_heading in headings:
        turn = ContinuousPlanner._wrap_angle_deg(target_heading - current_heading)
        quarter_turns = int(round(turn / 90.0))
        if abs(turn - 90.0 * quarter_turns) > 1e-6:
            raise ValueError("normal-maze movement requires a non-90-degree turn")
        quarter_turns %= 4
        if quarter_turns == 1:
            output.append("l")
        elif quarter_turns == 3:
            output.append("r")
        elif quarter_turns == 2:
            output.extend(("r", "r"))
        output.append("f")
        current_heading = target_heading
    return "".join(output), current_heading


def _final_motion_heading(
    waypoints_mm: Sequence[object],
    initial_heading_deg: float,
) -> float:
    """Return the heading after following a waypoint sequence."""
    heading = initial_heading_deg
    for first, second in zip(waypoints_mm, waypoints_mm[1:]):
        dx = float(second.x) - float(first.x)
        dy = float(second.y) - float(first.y)
        if dx != 0.0 or dy != 0.0:
            heading = math.degrees(math.atan2(dy, dx))
    return heading


def format_hybrid_command(
    route: HybridRoute,
    initial_heading_deg: float,
    image_shape: Sequence[int],
    cell_count: int = 9,
) -> str:
    """Format Task 2 as ``LFR,[(angle, distance),...],LFR``.

    Each ``f`` represents one normal 180 mm maze-cell transition. Arbitrary
    angles and metric distances occur only in the middle obstacle list. A
    zero-distance final tuple is added when needed to align the robot with the
    first cardinal movement after leaving the obstacle course.
    """
    if not route.succeeded or len(route.segment_results) != 3:
        raise ValueError("a successful three-segment hybrid route is required")

    before, obstacle, after = route.segment_results
    before_headings = _normal_segment_directions(
        before.grid_waypoints,
        image_shape,
        cell_count,
    )
    before_lfr, heading_at_entrance = _lfr_for_headings(
        before_headings,
        initial_heading_deg,
    )

    obstacle_commands = ContinuousPlanner.make_motion_commands(
        obstacle.waypoints_mm,
        heading_at_entrance,
    )
    heading_at_exit = _final_motion_heading(
        obstacle.waypoints_mm,
        heading_at_entrance,
    )

    after_headings = _normal_segment_directions(
        after.grid_waypoints,
        image_shape,
        cell_count,
    )
    heading_for_after = heading_at_exit
    if after_headings:
        alignment_turn = ContinuousPlanner._wrap_angle_deg(
            after_headings[0] - heading_at_exit
        )
        if abs(alignment_turn) > 1e-6:
            obstacle_commands.append(
                MotionCommand(turn_deg=alignment_turn, distance_mm=0.0)
            )
        heading_for_after = after_headings[0]

    after_lfr, _ = _lfr_for_headings(after_headings, heading_for_after)
    tuple_text = ",".join(
        f"({command.turn_deg:.1f},{command.distance_mm:.1f})"
        for command in obstacle_commands
    )
    return f'"{before_lfr},[{tuple_text}],{after_lfr}";'


def create_demo_course(output_pixels: int = 324) -> np.ndarray:
    """Create a full 9 x 9 demo with a central 5 x 5 obstacle region."""
    if output_pixels < 180:
        raise ValueError("the full demo maze must be at least 180 pixels square")

    image = np.full((output_pixels, output_pixels, 3), 255, dtype=np.uint8)

    def point(x_fraction: float, y_fraction: float) -> Point:
        return (
            int(round(x_fraction * (output_pixels - 1))),
            int(round(y_fraction * (output_pixels - 1))),
        )

    wall_thickness = max(2, int(round(output_pixels / 162)))
    # The complete 9-cell board is 1620 mm across; cylinders are 100 mm wide.
    cylinder_radius = max(5, int(round(output_pixels * 50.0 / 1620.0)))

    # AI-assisted implementation: normal maze walls before and after the
    # continuous region. These are only for the self-contained notebook demo.
    normal_walls = (
        # Normal maze before the obstacle course (lower section).
        ((0.11, 0.78), (0.11, 0.91)),
        ((0.11, 0.78), (0.31, 0.78)),
        ((0.31, 0.89), (0.31, 1.00)),
        # Normal maze after the obstacle course (upper section).
        ((0.69, 0.00), (0.69, 0.11)),
        ((0.69, 0.11), (0.89, 0.11)),
        ((0.89, 0.11), (0.89, 0.22)),
        ((0.56, 0.12), (0.56, 0.22)),
        # Side areas of the standard maze.
        ((0.11, 0.22), (0.11, 0.56)),
        ((0.89, 0.44), (0.89, 0.78)),
        ((0.00, 0.56), (0.11, 0.56)),
        ((0.89, 0.44), (1.00, 0.44)),
    )
    for start, end in normal_walls:
        cv2.line(
            image,
            point(*start),
            point(*end),
            (0, 0, 0),
            wall_thickness,
            cv2.LINE_8,
        )

    # Central rows/columns 2..6 form the 5 x 5 continuous region. Its only
    # entrance is at the bottom and its only exit is at the top.
    region_left = 2.0 / 9.0
    region_right = 7.0 / 9.0
    region_top = 2.0 / 9.0
    region_bottom = 7.0 / 9.0
    door_left = 4.0 / 9.0
    door_right = 5.0 / 9.0
    boundary_segments = (
        ((region_left, region_top), (door_left, region_top)),
        ((door_right, region_top), (region_right, region_top)),
        ((region_left, region_bottom), (door_left, region_bottom)),
        ((door_right, region_bottom), (region_right, region_bottom)),
        ((region_left, region_top), (region_left, region_bottom)),
        ((region_right, region_top), (region_right, region_bottom)),
    )
    for start, end in boundary_segments:
        cv2.line(
            image,
            point(*start),
            point(*end),
            (0, 0, 0),
            wall_thickness,
            cv2.LINE_8,
        )

    # Four 100 mm cylinders require a continuous, non-grid path between them.
    for centre in ((0.40, 0.34), (0.59, 0.43), (0.40, 0.52), (0.59, 0.62)):
        cv2.circle(
            image,
            point(*centre),
            cylinder_radius,
            (0, 0, 0),
            -1,
            cv2.LINE_AA,
        )

    cv2.rectangle(
        image,
        (0, 0),
        (output_pixels - 1, output_pixels - 1),
        (0, 0, 0),
        wall_thickness,
    )
    return image
