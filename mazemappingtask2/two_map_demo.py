"""Executable example for the checked-in Task 4.2 camera image.

Generative AI disclosure: OpenAI Codex assisted with this implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from mazemapping.mask_maze import create_maze_masks

from .task2_pipeline import project_points_to_original, rectify_course
from .two_map_planner import (
    Portal,
    TwoMapRoute,
    build_normal_maze_map,
    build_obstacle_map,
    draw_normal_maze_map,
    draw_obstacle_map,
    make_grid_calibration,
    plan_two_map_route,
)


Cell = Tuple[int, int]


@dataclass(frozen=True)
class DemoConfig:
    image_file: Path
    output_pixels: int = 324
    board_corners: Optional[Sequence[Sequence[float]]] = None
    grid_bounds: Tuple[float, float, float, float] = (29.0, 27.0, 299.0, 297.0)
    obstacle_top_left: Cell = (1, 1)
    obstacle_size: int = 5
    entrance: Portal = Portal(outside_cell=(6, 3), inside_cell=(5, 3))
    exit: Portal = Portal(outside_cell=(1, 6), inside_cell=(1, 5))
    start: Cell = (5, 0)
    goal: Cell = (0, 6)
    initial_heading_deg: float = 90.0
    goal_heading_deg: Optional[float] = None
    robot_radius_mm: float = 30.0
    safety_margin_mm: float = 5.0
    obstacle_resolution_mm: float = 5.0
    endpoint_clear_radius_pixels: int = 25


@dataclass
class DemoOutput:
    source_bgr: np.ndarray
    rectified_bgr: np.ndarray
    task1_masks: dict[str, np.ndarray]
    normal_map: object
    obstacle_map: object
    route: TwoMapRoute
    normal_preview_bgr: np.ndarray
    obstacle_preview_bgr: np.ndarray
    original_preview_bgr: np.ndarray


def default_unavailable_cells() -> set[Cell]:
    """Return the 12 black corner squares excluded from the 69-cell maze."""
    return {
        (0, 0), (0, 1), (0, 7), (0, 8),
        (1, 0), (1, 8),
        (7, 0), (7, 8),
        (8, 0), (8, 1), (8, 7), (8, 8),
    }


def run_two_map_demo(config: DemoConfig) -> DemoOutput:
    source = cv2.imread(str(config.image_file), cv2.IMREAD_COLOR)
    if source is None:
        raise FileNotFoundError(f"Could not load maze image: {config.image_file}")
    rectified = rectify_course(
        source,
        config.output_pixels,
        config.board_corners,
    )

    # This is the exact Task 1 masking entry point requested by the user.
    task1_masks = create_maze_masks(rectified)
    calibration = make_grid_calibration(*config.grid_bounds)
    start_point = calibration.centre(config.start)
    goal_point = calibration.centre(config.goal)
    normal_map = build_normal_maze_map(
        task1_masks,
        calibration,
        config.obstacle_top_left,
        config.obstacle_size,
        config.entrance,
        config.exit,
        unavailable_cells=default_unavailable_cells(),
        endpoint_clearings=(
            (start_point, config.endpoint_clear_radius_pixels),
            (goal_point, config.endpoint_clear_radius_pixels),
        ),
    )
    obstacle_map = build_obstacle_map(
        rectified,
        task1_masks,
        calibration,
        config.obstacle_top_left,
        config.obstacle_size,
        config.entrance,
        config.exit,
        resolution_mm=config.obstacle_resolution_mm,
    )
    route = plan_two_map_route(
        normal_map,
        obstacle_map,
        config.start,
        config.goal,
        config.initial_heading_deg,
        config.robot_radius_mm,
        config.safety_margin_mm,
        config.goal_heading_deg,
    )

    normal_preview = draw_normal_maze_map(
        rectified,
        normal_map,
        route.normal_before_cells,
        route.normal_after_cells,
    )
    obstacle_preview = draw_obstacle_map(obstacle_map, route.obstacle_result)

    original_preview = source.copy()
    original_points = project_points_to_original(
        route.full_waypoints,
        source.shape,
        rectified.shape,
        config.board_corners,
    )
    if len(original_points) >= 2:
        cv2.polylines(
            original_preview,
            [np.asarray(original_points, dtype=np.int32)],
            False,
            (0, 210, 0),
            7,
            cv2.LINE_AA,
        )
    for point in original_points:
        cv2.circle(original_preview, point, 7, (255, 120, 0), -1, cv2.LINE_AA)

    return DemoOutput(
        source_bgr=source,
        rectified_bgr=rectified,
        task1_masks=task1_masks,
        normal_map=normal_map,
        obstacle_map=obstacle_map,
        route=route,
        normal_preview_bgr=normal_preview,
        obstacle_preview_bgr=obstacle_preview,
        original_preview_bgr=original_preview,
    )
