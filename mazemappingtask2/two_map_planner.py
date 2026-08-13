"""Two-map planner for the MTRN3100 Task 4.2 hybrid maze.

The normal maze is represented as a discrete 9 x 9 cell graph built from the
directional masks produced by ``mazemapping.mask_maze.create_maze_masks``.
The configured 5 x 5 continuous region is separately warped into a metric
occupancy grid and solved with RRT*.

Generative AI disclosure: OpenAI Codex assisted with this implementation.
AI-assisted sections are identified by inline comments.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import random
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np

from computer.task4 import (
    ContinuousPlanner,
    GridPoint,
    MotionCommand,
    OccupancyGrid,
    PlanResult,
    PlanStatus,
    PlannerConfig,
)
from mazemapping.clip_grid import ClipGridResult, infer_clip_grid, project_points
from mazemapping.mask_maze import create_maze_masks


Cell = Tuple[int, int]
Point = Tuple[int, int]
Graph = Dict[Cell, List[Cell]]


class RRTStarPlanner(ContinuousPlanner):
    """Deterministic RRT* planner for the continuous cylinder region.

    Edges are checked against the inflated occupancy grid.  The returned route
    is resampled so the robot never receives one excessively long drive.
    """

    def __init__(
        self,
        config: PlannerConfig,
        iterations: int = 5000,
        step_pixels: int = 20,
        neighbour_pixels: int = 20,
        max_waypoint_spacing_mm: float = 75.0,
        seed: int = 3100,
    ) -> None:
        super().__init__(config)
        self.iterations = iterations
        self.step_pixels = step_pixels
        self.neighbour_pixels = neighbour_pixels
        self.max_waypoint_spacing_mm = max_waypoint_spacing_mm
        self.seed = seed

    def plan(self, grid, start, goal) -> PlanResult:
        start_point, goal_point = self._point(start), self._point(goal)
        if not self._valid_config() or self.iterations <= 0 or self.step_pixels <= 0:
            return PlanResult(PlanStatus.INVALID_GRID)
        if not grid.in_bounds(start_point):
            return PlanResult(PlanStatus.START_OUT_OF_BOUNDS)
        if not grid.in_bounds(goal_point):
            return PlanResult(PlanStatus.GOAL_OUT_OF_BOUNDS)

        inflated = self._inflate(
            grid, self.config.robot_radius_mm + self.config.safety_margin_mm
        )
        if inflated.is_occupied(start_point):
            return PlanResult(PlanStatus.START_BLOCKED, inflated)
        if inflated.is_occupied(goal_point):
            return PlanResult(PlanStatus.GOAL_BLOCKED, inflated)

        rng = random.Random(self.seed)
        nodes = [start_point]
        parents = [-1]
        costs = [0.0]
        children = [set()]

        for iteration in range(self.iterations):
            sample = goal_point if iteration % 10 == 0 else GridPoint(
                rng.randrange(inflated.width), rng.randrange(inflated.height)
            )
            if inflated.is_occupied(sample):
                continue
            nearest = min(
                range(len(nodes)), key=lambda i: self._distance(nodes[i], sample)
            )
            new = self._steer(nodes[nearest], sample)
            if new in nodes or inflated.is_occupied(new):
                continue

            near = [
                i for i, point in enumerate(nodes)
                if self._distance(point, new) <= self.neighbour_pixels
                and self._has_line_of_sight(inflated, point, new)
            ]
            if not near:
                continue
            parent = min(near, key=lambda i: costs[i] + self._distance(nodes[i], new))
            new_cost = costs[parent] + self._distance(nodes[parent], new)
            nodes.append(new)
            parents.append(parent)
            costs.append(new_cost)
            children.append(set())
            new_index = len(nodes) - 1
            children[parent].add(new_index)

            for index in near:
                rewired = new_cost + self._distance(new, nodes[index])
                if index != 0 and rewired + 1e-9 < costs[index]:
                    old_cost = costs[index]
                    children[parents[index]].discard(index)
                    parents[index] = new_index
                    costs[index] = rewired
                    children[new_index].add(index)
                    self._shift_descendant_costs(
                        index, rewired - old_cost, children, costs
                    )

        goal_candidates = [
            index for index, point in enumerate(nodes)
            if self._has_line_of_sight(inflated, point, goal_point)
        ]
        if not goal_candidates:
            return PlanResult(PlanStatus.NO_PATH, inflated)
        goal_parent = min(
            goal_candidates,
            key=lambda i: costs[i] + self._distance(nodes[i], goal_point),
        )

        path = [goal_point]
        index = goal_parent
        while index >= 0:
            path.append(nodes[index])
            index = parents[index]
        path.reverse()
        # RRT* needs many closely spaced tree nodes to search reliably, but
        # those internal nodes should not all become robot commands. First
        # remove any node that can be bypassed with a collision-free edge,
        # then add checkpoints at the requested maximum spacing. This makes
        # max_waypoint_spacing_mm directly control the blue output dots.
        collision_safe_segments = self._simplify(inflated, path)
        waypoints = self._resample(
            collision_safe_segments,
            grid.resolution_mm,
        )
        metric = self._to_metric(inflated, waypoints)
        return PlanResult(
            PlanStatus.SUCCESS,
            inflated,
            path,
            waypoints,
            metric,
            self._to_relative(metric),
        )

    @staticmethod
    def _shift_descendant_costs(index, delta, children, costs) -> None:
        pending = list(children[index])
        while pending:
            child = pending.pop()
            costs[child] += delta
            pending.extend(children[child])

    @staticmethod
    def _distance(first: GridPoint, second: GridPoint) -> float:
        return math.hypot(second.x - first.x, second.y - first.y)

    def _steer(self, start: GridPoint, target: GridPoint) -> GridPoint:
        distance = self._distance(start, target)
        if distance <= self.step_pixels:
            return target
        scale = self.step_pixels / distance
        return GridPoint(
            int(round(start.x + (target.x - start.x) * scale)),
            int(round(start.y + (target.y - start.y) * scale)),
        )

    def _resample(self, path: Sequence[GridPoint], resolution_mm: float) -> List[GridPoint]:
        maximum = max(1, int(self.max_waypoint_spacing_mm / resolution_mm))
        output = [path[0]]
        for start, end in zip(path, path[1:]):
            pieces = max(1, int(math.ceil(self._distance(start, end) / maximum)))
            for piece in range(1, pieces + 1):
                point = GridPoint(
                    int(round(start.x + (end.x - start.x) * piece / pieces)),
                    int(round(start.y + (end.y - start.y) * piece / pieces)),
                )
                if point != output[-1]:
                    output.append(point)
        return output


HEADINGS_DEG = {
    "E": 0.0,
    "N": 90.0,
    "W": 180.0,
    "S": -90.0,
}


@dataclass(frozen=True)
class GridCalibration:
    """Calibrated full-maze cell boundaries in rectified-image pixels."""

    x_edges: Tuple[float, ...]
    y_edges: Tuple[float, ...]
    homography: Optional[Tuple[Tuple[float, float, float], ...]] = None

    @property
    def rows(self) -> int:
        return len(self.y_edges) - 1

    @property
    def columns(self) -> int:
        return len(self.x_edges) - 1

    def validate(self) -> None:
        if self.rows <= 0 or self.columns <= 0:
            raise ValueError("grid calibration needs at least one cell")
        if any(b <= a for a, b in zip(self.x_edges, self.x_edges[1:])):
            raise ValueError("x_edges must be strictly increasing")
        if any(b <= a for a, b in zip(self.y_edges, self.y_edges[1:])):
            raise ValueError("y_edges must be strictly increasing")
        if self.homography is not None:
            matrix = np.asarray(self.homography, dtype=np.float64)
            if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
                raise ValueError("grid homography must be a finite 3 x 3 matrix")
            if abs(float(np.linalg.det(matrix))) < 1e-12:
                raise ValueError("grid homography must be invertible")

    def project(self, logical_points: Sequence[Sequence[float]]) -> np.ndarray:
        """Project logical ``(column, row)`` points into image pixels."""
        points = np.asarray(logical_points, dtype=np.float32).reshape((-1, 2))
        if self.homography is None:
            projected = []
            for column, row in points:
                x = np.interp(column, np.arange(len(self.x_edges)), self.x_edges)
                y = np.interp(row, np.arange(len(self.y_edges)), self.y_edges)
                projected.append((x, y))
            return np.asarray(projected, dtype=np.float32)
        return project_points(points, np.asarray(self.homography, dtype=np.float64))

    def centre(self, cell: Cell) -> Point:
        row, column = cell
        if not (0 <= row < self.rows and 0 <= column < self.columns):
            raise ValueError(f"cell {cell} is outside the calibrated maze")
        x, y = self.project(((column + 0.5, row + 0.5),))[0]
        return int(round(float(x))), int(round(float(y)))

    def cell_quad(self, cell: Cell) -> np.ndarray:
        """Return a cell's four image-space corners clockwise from top-left."""
        row, column = cell
        if not (0 <= row < self.rows and 0 <= column < self.columns):
            raise ValueError(f"cell {cell} is outside the calibrated maze")
        return self.project(
            (
                (column, row),
                (column + 1, row),
                (column + 1, row + 1),
                (column, row + 1),
            )
        )

    def boundary_segment(self, first: Cell, second: Cell) -> np.ndarray:
        """Return the image-space endpoints of a shared cardinal boundary."""
        row, column = first
        next_row, next_column = second
        if abs(next_row - row) + abs(next_column - column) != 1:
            raise ValueError("boundary cells must be cardinally adjacent")
        if row == next_row:
            boundary_column = max(column, next_column)
            logical = ((boundary_column, row), (boundary_column, row + 1))
        else:
            boundary_row = max(row, next_row)
            logical = ((column, boundary_row), (column + 1, boundary_row))
        return self.project(logical)

    def region_quad(self, top: int, left: int, size: int) -> np.ndarray:
        bottom = top + size
        right = left + size
        if not (0 <= top < bottom <= self.rows):
            raise ValueError("continuous-region row bounds are outside the maze")
        if not (0 <= left < right <= self.columns):
            raise ValueError("continuous-region column bounds are outside the maze")
        return self.project(
            ((left, top), (right, top), (right, bottom), (left, bottom))
        )


@dataclass(frozen=True)
class Portal:
    """One permitted transition between normal and continuous maps."""

    outside_cell: Cell
    inside_cell: Cell

    def validate(self) -> None:
        distance = abs(self.outside_cell[0] - self.inside_cell[0]) + abs(
            self.outside_cell[1] - self.inside_cell[1]
        )
        if distance != 1:
            raise ValueError("portal cells must share one cardinal boundary")


@dataclass(frozen=True)
class WallEvidence:
    blocked: bool
    uncertain: bool
    score: float
    coverage: float
    longest_run: float
    offset_pixels: int


@dataclass
class NormalMazeMap:
    graph: Graph
    walls: Dict[frozenset[Cell], WallEvidence]
    excluded_cells: Set[Cell]
    entrance: Portal
    exit: Portal
    calibration: GridCalibration


@dataclass
class ObstacleMap:
    crop_bgr: np.ndarray
    occupancy_mask: np.ndarray
    grid: OccupancyGrid
    full_to_local: np.ndarray
    local_to_full: np.ndarray
    entrance_local: Point
    exit_local: Point
    resolution_mm: float
    detected_centres: List[Point]


@dataclass
class TwoMapRoute:
    normal_before_cells: List[Cell]
    obstacle_result: PlanResult
    normal_after_cells: List[Cell]
    obstacle_waypoints_full: List[Tuple[float, float]]
    command: str
    full_waypoints: List[Tuple[float, float]]

    @property
    def succeeded(self) -> bool:
        return (
            bool(self.normal_before_cells)
            and self.obstacle_result.succeeded
            and bool(self.normal_after_cells)
        )


def make_grid_calibration(
    left: float,
    top: float,
    right: float,
    bottom: float,
    rows: int = 9,
    columns: int = 9,
) -> GridCalibration:
    """Create equally spaced cell boundaries from fixed-camera calibration."""
    if rows <= 0 or columns <= 0 or right <= left or bottom <= top:
        raise ValueError("invalid maze grid calibration")
    calibration = GridCalibration(
        x_edges=tuple(float(value) for value in np.linspace(left, right, columns + 1)),
        y_edges=tuple(float(value) for value in np.linspace(top, bottom, rows + 1)),
    )
    calibration.validate()
    return calibration


def make_clip_grid_calibration(
    image_bgr: np.ndarray,
    rows: int = 9,
    columns: int = 9,
    image_transform: Optional[np.ndarray] = None,
    excluded_logical_regions: Sequence[Tuple[int, int, int, int]] = (),
) -> Tuple[GridCalibration, ClipGridResult]:
    """Fit Task 1's cyan-clip grid and express it in planner coordinates.

    Clip detection runs on the original full-resolution photograph. The
    optional ``image_transform`` is then composed with that fit, avoiding the
    loss of small cyan clips when the image is downsampled for Task 4.2.
    """
    result = infer_clip_grid(
        image_bgr,
        rows=rows,
        columns=columns,
        excluded_logical_regions=excluded_logical_regions,
    )
    homography = np.asarray(result.homography, dtype=np.float64)
    if image_transform is not None:
        image_transform = np.asarray(image_transform, dtype=np.float64)
        if image_transform.shape != (3, 3) or not np.isfinite(image_transform).all():
            raise ValueError("image_transform must be a finite 3 x 3 matrix")
        homography = image_transform @ homography

    x_samples = project_points(
        np.asarray([(column, rows / 2.0) for column in range(columns + 1)]),
        homography,
    )[:, 0]
    y_samples = project_points(
        np.asarray([(columns / 2.0, row) for row in range(rows + 1)]),
        homography,
    )[:, 1]
    calibration = GridCalibration(
        x_edges=tuple(float(value) for value in x_samples),
        y_edges=tuple(float(value) for value in y_samples),
        homography=tuple(tuple(float(value) for value in row) for row in homography),
    )
    calibration.validate()
    return calibration, result


def _longest_true_run(signal: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in signal:
        current = current + 1 if bool(value) else 0
        longest = max(longest, current)
    return longest


def wall_evidence_between(
    first: Cell,
    second: Cell,
    calibration: GridCalibration,
    main_dark_mask: np.ndarray,
    horizontal_line_mask: np.ndarray,
    vertical_line_mask: np.ndarray,
    trim_fraction: float = 0.14,
    search_pixels: int = 3,
    band_half_width: int = 2,
    close_pixels: int = 5,
    open_score: float = 0.30,
    blocked_score: float = 0.45,
) -> WallEvidence:
    """Classify the shared cell boundary using Task 1 directional masks."""
    row, column = first
    next_row, next_column = second
    if abs(next_row - row) + abs(next_column - column) != 1:
        raise ValueError("wall evidence requires cardinally adjacent cells")
    if not 0.0 <= trim_fraction < 0.5:
        raise ValueError("trim_fraction must be between 0 and 0.5")
    if close_pixels <= 0:
        raise ValueError("close_pixels must be positive")

    horizontal_evidence = cv2.bitwise_or(main_dark_mask, horizontal_line_mask)
    vertical_evidence = cv2.bitwise_or(main_dark_mask, vertical_line_mask)

    vertical_boundary = row == next_row
    if calibration.homography is not None:
        mask = vertical_evidence if vertical_boundary else horizontal_evidence
        endpoints = calibration.boundary_segment(first, second).astype(np.float64)
        direction = endpoints[1] - endpoints[0]
        endpoints[0] += trim_fraction * direction
        endpoints[1] -= trim_fraction * direction
        direction = endpoints[1] - endpoints[0]
        length = float(np.linalg.norm(direction))
        if length <= 0.0:
            raise ValueError("wall sample became empty after trimming")
        tangent = direction / length
        normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
        sample_count = max(2, int(round(length)) + 1)
        along = np.linspace(0.0, 1.0, sample_count)[:, None]
        centre_line = endpoints[0] + along * direction

        best = (0.0, 0.0, 0.0, 0)
        kernel = np.ones((1, close_pixels), dtype=np.uint8)
        for offset in range(-search_pixels, search_pixels + 1):
            signal = np.zeros(sample_count, dtype=bool)
            for band_offset in range(-band_half_width, band_half_width + 1):
                samples = centre_line + (offset + band_offset) * normal
                x = np.rint(samples[:, 0]).astype(np.int32)
                y = np.rint(samples[:, 1]).astype(np.int32)
                valid = (
                    (x >= 0)
                    & (x < mask.shape[1])
                    & (y >= 0)
                    & (y < mask.shape[0])
                )
                signal[valid] |= mask[y[valid], x[valid]] > 0

            signal = cv2.morphologyEx(
                signal.astype(np.uint8).reshape(1, -1),
                cv2.MORPH_CLOSE,
                kernel,
            ).ravel() > 0
            coverage = float(np.mean(signal))
            longest_run = float(_longest_true_run(signal)) / float(signal.size)
            score = 0.65 * coverage + 0.35 * longest_run
            candidate = (score, coverage, longest_run, offset)
            if candidate > best:
                best = candidate

        score, coverage, longest_run, offset = best
        return WallEvidence(
            blocked=score >= blocked_score,
            uncertain=open_score < score < blocked_score,
            score=score,
            coverage=coverage,
            longest_run=longest_run,
            offset_pixels=offset,
        )

    if vertical_boundary:
        mask = vertical_evidence
        fixed = calibration.x_edges[max(column, next_column)]
        segment_start = calibration.y_edges[row]
        segment_end = calibration.y_edges[row + 1]
    else:
        mask = horizontal_evidence
        fixed = calibration.y_edges[max(row, next_row)]
        segment_start = calibration.x_edges[column]
        segment_end = calibration.x_edges[column + 1]

    trim = trim_fraction * (segment_end - segment_start)
    start = int(round(segment_start + trim))
    end = int(round(segment_end - trim))
    if end < start:
        raise ValueError("wall sample became empty after trimming")

    best = (0.0, 0.0, 0.0, 0)
    kernel = np.ones((1, close_pixels), dtype=np.uint8)
    for offset in range(-search_pixels, search_pixels + 1):
        fixed_pixel = int(round(fixed + offset))
        if vertical_boundary:
            band = mask[
                start : end + 1,
                max(0, fixed_pixel - band_half_width) : min(
                    mask.shape[1], fixed_pixel + band_half_width + 1
                ),
            ]
            signal = np.any(band > 0, axis=1).astype(np.uint8)
        else:
            band = mask[
                max(0, fixed_pixel - band_half_width) : min(
                    mask.shape[0], fixed_pixel + band_half_width + 1
                ),
                start : end + 1,
            ]
            signal = np.any(band > 0, axis=0).astype(np.uint8)

        signal = cv2.morphologyEx(
            signal.reshape(1, -1),
            cv2.MORPH_CLOSE,
            kernel,
        ).ravel() > 0
        coverage = float(np.mean(signal)) if signal.size else 0.0
        longest_run = (
            float(_longest_true_run(signal)) / float(signal.size)
            if signal.size
            else 0.0
        )
        score = 0.65 * coverage + 0.35 * longest_run
        candidate = (score, coverage, longest_run, offset)
        if candidate > best:
            best = candidate

    score, coverage, longest_run, offset = best
    uncertain = open_score < score < blocked_score
    return WallEvidence(
        # Borderline evidence is traversable. Only a confident score creates
        # a graph wall; there is no separate "uncertain wall" category.
        blocked=score >= blocked_score,
        uncertain=uncertain,
        score=score,
        coverage=coverage,
        longest_run=longest_run,
        offset_pixels=offset,
    )


def _all_cells(calibration: GridCalibration) -> Set[Cell]:
    return {
        (row, column)
        for row in range(calibration.rows)
        for column in range(calibration.columns)
    }


def build_normal_maze_map(
    mask_outputs: Dict[str, np.ndarray],
    calibration: GridCalibration,
    obstacle_top_left: Cell,
    obstacle_size: int,
    entrance: Portal,
    exit: Portal,
    unavailable_cells: Iterable[Cell] = (),
    endpoint_clearings: Sequence[Tuple[Point, int]] = (),
) -> NormalMazeMap:
    """Build the normal 9 x 9 graph from Task 1's exact mask outputs."""
    calibration.validate()
    entrance.validate()
    exit.validate()
    required_keys = (
        "03a_main_dark_mask.png",
        "02b_horizontal_line_mask.png",
        "02c_vertical_line_mask.png",
    )
    for key in required_keys:
        if key not in mask_outputs:
            raise KeyError(f"Task 1 mask output is missing {key}")

    main_dark = mask_outputs["03a_main_dark_mask.png"].copy()
    horizontal = mask_outputs["02b_horizontal_line_mask.png"].copy()
    vertical = mask_outputs["02c_vertical_line_mask.png"].copy()
    # Visible robots at supplied endpoints can otherwise look like short walls.
    # This local clearing changes only graph evidence, not Task 1's CV code.
    for centre, radius in endpoint_clearings:
        cv2.circle(main_dark, centre, radius, 0, -1)
        cv2.circle(horizontal, centre, radius, 0, -1)
        cv2.circle(vertical, centre, radius, 0, -1)

    top, left = obstacle_top_left
    obstacle_cells = {
        (row, column)
        for row in range(top, top + obstacle_size)
        for column in range(left, left + obstacle_size)
    }
    if not obstacle_cells <= _all_cells(calibration):
        raise ValueError("continuous region extends outside the normal maze")
    if entrance.inside_cell not in obstacle_cells or exit.inside_cell not in obstacle_cells:
        raise ValueError("inside portal cells must lie in the continuous region")
    if entrance.outside_cell in obstacle_cells or exit.outside_cell in obstacle_cells:
        raise ValueError("outside portal cells must lie in the normal maze")

    unavailable = set(unavailable_cells)
    if not unavailable <= _all_cells(calibration):
        raise ValueError("unavailable cells extend outside the normal maze")
    active = _all_cells(calibration) - obstacle_cells - unavailable
    active.update((entrance.inside_cell, exit.inside_cell))
    graph: Graph = {cell: [] for cell in active}
    walls: Dict[frozenset[Cell], WallEvidence] = {}
    portal_edges = {
        frozenset((entrance.outside_cell, entrance.inside_cell)),
        frozenset((exit.inside_cell, exit.outside_cell)),
    }

    # AI-assisted implementation: each expected grid boundary is sampled once
    # from the original Task 1 directional masks and shared both directions.
    for row, column in sorted(active):
        first = (row, column)
        for row_delta, column_delta in ((0, 1), (1, 0)):
            second = (row + row_delta, column + column_delta)
            if second not in active:
                continue
            edge_key = frozenset((first, second))
            crosses_region = (
                first in obstacle_cells or second in obstacle_cells
            )
            if crosses_region and edge_key not in portal_edges:
                continue
            if edge_key in portal_edges:
                evidence = WallEvidence(False, False, 0.0, 0.0, 0.0, 0)
            else:
                evidence = wall_evidence_between(
                    first,
                    second,
                    calibration,
                    main_dark,
                    horizontal,
                    vertical,
                )
            walls[edge_key] = evidence
            if not evidence.blocked:
                graph[first].append(second)
                graph[second].append(first)

    for neighbours in graph.values():
        neighbours.sort()
    return NormalMazeMap(
        graph=graph,
        walls=walls,
        excluded_cells=obstacle_cells - {entrance.inside_cell, exit.inside_cell},
        entrance=entrance,
        exit=exit,
        calibration=calibration,
    )


def shortest_cell_path(graph: Graph, start: Cell, goal: Cell) -> List[Cell]:
    """Return the shortest cell-count path with turns as the tie-breaker."""
    if start not in graph:
        raise ValueError(f"normal-maze start cell {start} is unavailable")
    if goal not in graph:
        raise ValueError(f"normal-maze goal cell {goal} is unavailable")
    if start == goal:
        return [start]

    start_state = (start, None)
    cost = {start_state: (0, 0)}
    parent = {start_state: None}
    frontier = [(0, 0, 0, start_state)]
    counter = 0
    goal_state = None
    while frontier:
        steps, turns, _, state = heapq.heappop(frontier)
        cell, previous_direction = state
        if (steps, turns) != cost.get(state):
            continue
        if cell == goal:
            goal_state = state
            break
        for neighbour in graph[cell]:
            direction = (
                neighbour[0] - cell[0],
                neighbour[1] - cell[1],
            )
            new_cost = (
                steps + 1,
                turns + int(previous_direction is not None and direction != previous_direction),
            )
            next_state = (neighbour, direction)
            if new_cost >= cost.get(next_state, (math.inf, math.inf)):
                continue
            cost[next_state] = new_cost
            parent[next_state] = state
            counter += 1
            heapq.heappush(frontier, (*new_cost, counter, next_state))

    if goal_state is None:
        return []
    cells = []
    state = goal_state
    while state is not None:
        cells.append(state[0])
        state = parent[state]
    cells.reverse()
    return cells


def _direction_heading(first: Cell, second: Cell) -> float:
    row_change = second[0] - first[0]
    column_change = second[1] - first[1]
    if (row_change, column_change) == (0, 1):
        return 0.0
    if (row_change, column_change) == (-1, 0):
        return 90.0
    if (row_change, column_change) == (0, -1):
        return 180.0
    if (row_change, column_change) == (1, 0):
        return -90.0
    raise ValueError("normal cell path contains a non-cardinal edge")


def encode_cell_path(
    cells: Sequence[Cell],
    initial_heading_deg: float,
) -> Tuple[str, float]:
    """Encode one ``f`` per cell edge plus 90-degree ``l``/``r`` turns."""
    heading = float(initial_heading_deg)
    commands: List[str] = []
    for first, second in zip(cells, cells[1:]):
        required = _direction_heading(first, second)
        turn = ContinuousPlanner._wrap_angle_deg(required - heading)
        quarter_turns = int(round(turn / 90.0)) % 4
        if abs(turn - round(turn / 90.0) * 90.0) > 1e-6:
            raise ValueError("normal maze needs a non-90-degree turn")
        if quarter_turns == 1:
            commands.append("l")
        elif quarter_turns == 3:
            commands.append("r")
        elif quarter_turns == 2:
            commands.extend(("r", "r"))
        commands.append("f")
        heading = required
    return "".join(commands), heading


def encode_cardinal_turn(
    initial_heading_deg: float,
    final_heading_deg: float,
) -> str:
    """Encode a final in-place turn using only 90-degree ``l``/``r``."""
    turn = ContinuousPlanner._wrap_angle_deg(
        float(final_heading_deg) - float(initial_heading_deg)
    )
    quarter_turns = int(round(turn / 90.0)) % 4
    if abs(turn - round(turn / 90.0) * 90.0) > 1e-6:
        raise ValueError("normal maze goal heading must be a multiple of 90 degrees")
    if quarter_turns == 1:
        return "l"
    if quarter_turns == 3:
        return "r"
    if quarter_turns == 2:
        return "rr"
    return ""


def _portal_local_point(
    portal: Portal,
    region_top_left: Cell,
    region_size: int,
    output_pixels: int,
) -> Point:
    """Return the centre of the portal's inside cell in local crop pixels."""
    top, left = region_top_left
    local_row = portal.inside_cell[0] - top
    local_column = portal.inside_cell[1] - left
    if not (0 <= local_row < region_size and 0 <= local_column < region_size):
        raise ValueError("portal inside cell is outside the continuous crop")
    x = int(round((local_column + 0.5) * output_pixels / region_size - 0.5))
    y = int(round((local_row + 0.5) * output_pixels / region_size - 0.5))
    return x, y


def build_obstacle_map(
    rectified_bgr: np.ndarray,
    task1_outputs: Dict[str, np.ndarray],
    calibration: GridCalibration,
    obstacle_top_left: Cell,
    obstacle_size: int,
    entrance: Portal,
    exit: Portal,
    resolution_mm: float = 5.0,
    cell_size_mm: float = 180.0,
    minimum_component_area: int = 35,
) -> ObstacleMap:
    """Build an independent metric crop from Task 1's detected obstacle mask."""
    if resolution_mm <= 0.0 or cell_size_mm <= 0.0:
        raise ValueError("metric resolutions must be positive")
    region_mm = obstacle_size * cell_size_mm
    output_pixels = int(round(region_mm / resolution_mm))
    if output_pixels < 2:
        raise ValueError("continuous crop is too small")

    source_quad = calibration.region_quad(
        obstacle_top_left[0],
        obstacle_top_left[1],
        obstacle_size,
    )
    maximum = float(output_pixels - 1)
    destination_quad = np.asarray(
        [[0.0, 0.0], [maximum, 0.0], [maximum, maximum], [0.0, maximum]],
        dtype=np.float32,
    )
    full_to_local = cv2.getPerspectiveTransform(source_quad, destination_quad)
    local_to_full = np.linalg.inv(full_to_local)
    crop_bgr = cv2.warpPerspective(
        rectified_bgr,
        full_to_local,
        (output_pixels, output_pixels),
        flags=cv2.INTER_AREA,
    )

    # Reuse the exact Task 1 mask implementation as requested. The main dark
    # mask detects cylinders, while component geometry removes long wall pieces
    # and small floor/tape artefacts from this independent crop.
    main_dark_full = task1_outputs["03a_main_dark_mask.png"]
    local_dark = cv2.warpPerspective(
        main_dark_full,
        full_to_local,
        (output_pixels, output_pixels),
        flags=cv2.INTER_NEAREST,
    )
    # Perimeter walls can touch a nearby cylinder in the thresholded mask.
    # Remove only a narrow 10 px (~50 mm) rim before component filtering; the
    # crop boundary itself is reconstructed explicitly below.
    component_margin = max(1, int(round(50.0 / resolution_mm)))
    local_dark[:component_margin, :] = 0
    local_dark[-component_margin:, :] = 0
    local_dark[:, :component_margin] = 0
    local_dark[:, -component_margin:] = 0
    count, labels, statistics, centroids = cv2.connectedComponentsWithStats(
        local_dark,
        connectivity=8,
    )
    expected_diameter = 100.0 / resolution_mm
    expected_area = math.pi * (expected_diameter / 2.0) ** 2
    occupancy = np.zeros_like(local_dark)
    centres: List[Point] = []
    for component in range(1, count):
        x, y, width, height, area = map(int, statistics[component])
        aspect = width / float(height) if height else math.inf
        if not (minimum_component_area <= area <= 2.2 * expected_area):
            continue
        if not (0.55 * expected_diameter <= width <= 1.8 * expected_diameter):
            continue
        if not (0.55 * expected_diameter <= height <= 1.8 * expected_diameter):
            continue
        if not 0.55 <= aspect <= 1.8:
            continue
        centre = (
            int(round(float(centroids[component][0]))),
            int(round(float(centroids[component][1]))),
        )
        centres.append(centre)
        cv2.circle(
            occupancy,
            centre,
            int(round(expected_diameter / 2.0)),
            255,
            -1,
            cv2.LINE_AA,
        )

    entrance_local = _portal_local_point(
        entrance,
        obstacle_top_left,
        obstacle_size,
        output_pixels,
    )
    exit_local = _portal_local_point(
        exit,
        obstacle_top_left,
        obstacle_size,
        output_pixels,
    )
    # Block the synthetic crop perimeter, then open only the two configured
    # portal apertures. Portals themselves sit at inside-cell centres, safely
    # away from boundary inflation.
    border = max(1, int(round(5.0 / resolution_mm)))
    cv2.rectangle(
        occupancy,
        (0, 0),
        (output_pixels - 1, output_pixels - 1),
        255,
        border,
    )
    portal_half_width = int(round((cell_size_mm / resolution_mm) / 2.0))
    for portal, point in ((entrance, entrance_local), (exit, exit_local)):
        row_change = portal.outside_cell[0] - portal.inside_cell[0]
        column_change = portal.outside_cell[1] - portal.inside_cell[1]
        if row_change != 0:
            boundary_y = output_pixels - 1 if row_change > 0 else 0
            cv2.line(
                occupancy,
                (max(0, point[0] - portal_half_width), boundary_y),
                (min(output_pixels - 1, point[0] + portal_half_width), boundary_y),
                0,
                border,
            )
        else:
            boundary_x = output_pixels - 1 if column_change > 0 else 0
            cv2.line(
                occupancy,
                (boundary_x, max(0, point[1] - portal_half_width)),
                (boundary_x, min(output_pixels - 1, point[1] + portal_half_width)),
                0,
                border,
            )

    grid = OccupancyGrid.from_rows(occupancy > 0, resolution_mm)
    return ObstacleMap(
        crop_bgr=crop_bgr,
        occupancy_mask=occupancy,
        grid=grid,
        full_to_local=full_to_local,
        local_to_full=local_to_full,
        entrance_local=entrance_local,
        exit_local=exit_local,
        resolution_mm=resolution_mm,
        detected_centres=centres,
    )


def transform_points(
    points: Iterable[object],
    transform: np.ndarray,
) -> List[Tuple[float, float]]:
    coordinates = [
        (float(point.x), float(point.y))
        if hasattr(point, "x") and hasattr(point, "y")
        else (float(point[0]), float(point[1]))
        for point in points
    ]
    if not coordinates:
        return []
    transformed = cv2.perspectiveTransform(
        np.asarray(coordinates, dtype=np.float32).reshape((-1, 1, 2)),
        np.asarray(transform, dtype=np.float64),
    ).reshape((-1, 2))
    return [(float(x), float(y)) for x, y in transformed]


def _obstacle_final_heading(
    result: PlanResult,
    initial_heading: float,
) -> float:
    heading = initial_heading
    for first, second in zip(result.waypoints_mm, result.waypoints_mm[1:]):
        dx = second.x - first.x
        dy = second.y - first.y
        if dx != 0.0 or dy != 0.0:
            heading = math.degrees(math.atan2(dy, dx))
    return heading


def plan_two_map_route(
    normal_map: NormalMazeMap,
    obstacle_map: ObstacleMap,
    start: Cell,
    goal: Cell,
    initial_heading_deg: float,
    robot_radius_mm: float,
    safety_margin_mm: float,
    goal_heading_deg: Optional[float] = None,
    obstacle_distance_scale: float = 1.0,
    rrt_max_waypoint_spacing_mm: float = 150.0,
) -> TwoMapRoute:
    """Run normal graph -> obstacle RRT* -> normal graph and join commands."""
    if not math.isfinite(obstacle_distance_scale) or obstacle_distance_scale <= 0.0:
        raise ValueError("obstacle_distance_scale must be positive and finite")
    before = shortest_cell_path(
        normal_map.graph,
        start,
        normal_map.entrance.inside_cell,
    )
    after = shortest_cell_path(
        normal_map.graph,
        normal_map.exit.inside_cell,
        goal,
    )
    if not before:
        raise RuntimeError("no normal-maze path exists from start to entrance")
    if not after:
        raise RuntimeError("no normal-maze path exists from exit to goal")

    before_lfr, entrance_heading = encode_cell_path(before, initial_heading_deg)
    if not math.isfinite(rrt_max_waypoint_spacing_mm) or rrt_max_waypoint_spacing_mm <= 0:
        raise ValueError("rrt_max_waypoint_spacing_mm must be positive and finite")
    planner = RRTStarPlanner(
        PlannerConfig(
            robot_radius_mm=robot_radius_mm,
            safety_margin_mm=safety_margin_mm,
            simplify_path=False,
        ),
        max_waypoint_spacing_mm=rrt_max_waypoint_spacing_mm,
    )
    obstacle_result = planner.plan(
        obstacle_map.grid,
        obstacle_map.entrance_local,
        obstacle_map.exit_local,
    )
    if not obstacle_result.succeeded:
        raise RuntimeError(
            f"continuous obstacle planning failed: {obstacle_result.status.value}"
        )

    obstacle_commands = planner.make_motion_commands(
        obstacle_result.waypoints_mm,
        entrance_heading,
    )
    # Physical calibration applies only to positive continuous-section drive
    # distances. It does not alter angles, zero-distance heading alignment, or
    # the normal maze's fixed 180 mm `f` commands.
    obstacle_commands = [
        MotionCommand(
            command.turn_deg,
            command.distance_mm * obstacle_distance_scale,
        )
        for command in obstacle_commands
    ]
    exit_heading = _obstacle_final_heading(obstacle_result, entrance_heading)
    after_first_heading = (
        _direction_heading(after[0], after[1])
        if len(after) > 1
        else exit_heading
    )
    alignment_turn = ContinuousPlanner._wrap_angle_deg(
        after_first_heading - exit_heading
    )
    if abs(alignment_turn) > 1e-6:
        obstacle_commands.append(MotionCommand(alignment_turn, 0.0))
    after_lfr, final_heading = encode_cell_path(after, after_first_heading)
    if goal_heading_deg is not None:
        after_lfr += encode_cardinal_turn(final_heading, goal_heading_deg)

    tuple_text = ",".join(
        f"({command.turn_deg:.1f},{command.distance_mm:.1f})"
        for command in obstacle_commands
    )
    command = f'"{before_lfr},[{tuple_text}],{after_lfr}";'

    obstacle_full = transform_points(
        obstacle_result.grid_waypoints,
        obstacle_map.local_to_full,
    )
    before_full = [normal_map.calibration.centre(cell) for cell in before]
    after_full = [normal_map.calibration.centre(cell) for cell in after]
    full_waypoints: List[Tuple[float, float]] = [
        (float(x), float(y)) for x, y in before_full
    ]
    for points in (obstacle_full, after_full):
        for point in points:
            value = (float(point[0]), float(point[1]))
            if not full_waypoints or full_waypoints[-1] != value:
                full_waypoints.append(value)

    return TwoMapRoute(
        normal_before_cells=before,
        obstacle_result=obstacle_result,
        normal_after_cells=after,
        obstacle_waypoints_full=obstacle_full,
        command=command,
        full_waypoints=full_waypoints,
    )


def draw_normal_maze_map(
    image_bgr: np.ndarray,
    normal_map: NormalMazeMap,
    before: Sequence[Cell] = (),
    after: Sequence[Cell] = (),
) -> np.ndarray:
    """Draw the clean cell graph and normal routes without raw red masking."""
    display = image_bgr.copy()
    for cell, neighbours in normal_map.graph.items():
        for neighbour in neighbours:
            if cell < neighbour:
                cv2.line(
                    display,
                    normal_map.calibration.centre(cell),
                    normal_map.calibration.centre(neighbour),
                    (160, 160, 160),
                    1,
                    cv2.LINE_AA,
                )
    for cell in normal_map.graph:
        cv2.circle(
            display,
            normal_map.calibration.centre(cell),
            2,
            (90, 90, 90),
            -1,
            cv2.LINE_AA,
        )
    for path, colour in ((before, (255, 0, 0)), (after, (255, 0, 255))):
        points = [normal_map.calibration.centre(cell) for cell in path]
        if len(points) >= 2:
            cv2.polylines(
                display,
                [np.asarray(points, dtype=np.int32)],
                False,
                colour,
                3,
                cv2.LINE_AA,
            )
    return display


def draw_available_normal_graph(
    image_bgr: np.ndarray,
    normal_map: NormalMazeMap,
    show_labels: bool = True,
) -> np.ndarray:
    """Clearly show every available normal-maze node and traversable edge.

    Green lines are graph edges the normal planner may traverse. Green nodes
    are active cells, cyan/purple nodes are the continuous-region portals, and
    the excluded continuous white-space cells are shaded grey.
    """
    display = cv2.addWeighted(
        image_bgr,
        0.38,
        np.full_like(image_bgr, 245),
        0.62,
        0.0,
    )

    excluded_layer = display.copy()
    for cell in normal_map.excluded_cells:
        cv2.fillConvexPoly(
            excluded_layer,
            np.rint(normal_map.calibration.cell_quad(cell)).astype(np.int32),
            (105, 105, 105),
            cv2.LINE_AA,
        )
    display = cv2.addWeighted(display, 0.48, excluded_layer, 0.52, 0.0)

    for cell, neighbours in normal_map.graph.items():
        for neighbour in neighbours:
            if cell < neighbour:
                cv2.line(
                    display,
                    normal_map.calibration.centre(cell),
                    normal_map.calibration.centre(neighbour),
                    (40, 175, 40),
                    3,
                    cv2.LINE_AA,
                )

    portal_colours = {
        normal_map.entrance.inside_cell: (255, 200, 0),
        normal_map.exit.inside_cell: (210, 60, 190),
    }
    for cell in normal_map.graph:
        centre = normal_map.calibration.centre(cell)
        colour = portal_colours.get(cell, (0, 150, 0))
        cv2.circle(display, centre, 5, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(display, centre, 4, colour, -1, cv2.LINE_AA)
        if show_labels:
            label = f"{cell[0]},{cell[1]}"
            cv2.putText(
                display,
                label,
                (centre[0] + 5, centre[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.27,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                label,
                (centre[0] + 5, centre[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.27,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
    return display


def draw_normal_wall_diagnostics(
    image_bgr: np.ndarray,
    normal_map: NormalMazeMap,
    line_thickness: int = 3,
) -> np.ndarray:
    """Draw only confidently blocked normal-graph boundaries in red."""
    if line_thickness <= 0:
        raise ValueError("line_thickness must be positive")

    display = image_bgr.copy()
    calibration = normal_map.calibration

    # Match wall_evidence_between(): leave the ends of each boundary out so
    # neighbouring coloured decisions remain visually distinct at junctions.
    trim_fraction = 0.14
    for edge, evidence in normal_map.walls.items():
        if not evidence.blocked:
            continue
        first, second = tuple(edge)
        segment = calibration.boundary_segment(first, second).astype(np.float64)
        direction = segment[1] - segment[0]
        start_array = segment[0] + trim_fraction * direction
        end_array = segment[1] - trim_fraction * direction
        start = tuple(np.rint(start_array).astype(int))
        end = tuple(np.rint(end_array).astype(int))

        cv2.line(
            display,
            start,
            end,
            (0, 0, 255),
            line_thickness,
            cv2.LINE_AA,
        )

    return display


def draw_obstacle_map(
    obstacle_map: ObstacleMap,
    result: Optional[PlanResult] = None,
) -> np.ndarray:
    """Draw only the continuous crop, detected cylinders, and angled path."""
    display = obstacle_map.crop_bgr.copy()
    red = display.copy()
    red[obstacle_map.occupancy_mask > 0] = (0, 0, 255)
    display = cv2.addWeighted(display, 0.7, red, 0.3, 0.0)
    if result is not None and result.inflated_grid is not None:
        inflated = np.asarray(result.inflated_grid.cells, dtype=bool).reshape(
            result.inflated_grid.height,
            result.inflated_grid.width,
        )
        amber = display.copy()
        amber[inflated & (obstacle_map.occupancy_mask == 0)] = (0, 190, 255)
        display = cv2.addWeighted(display, 0.84, amber, 0.16, 0.0)
        points = [(point.x, point.y) for point in result.grid_waypoints]
        if len(points) >= 2:
            cv2.polylines(
                display,
                [np.asarray(points, dtype=np.int32)],
                False,
                (0, 210, 0),
                3,
                cv2.LINE_AA,
            )
    cv2.circle(display, obstacle_map.entrance_local, 5, (255, 255, 0), -1)
    cv2.circle(display, obstacle_map.exit_local, 5, (0, 165, 255), -1)
    return display
