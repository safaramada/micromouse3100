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

from .task2_pipeline import (
    course_rectification_transform,
    project_points_to_original,
    rectify_course,
    rectify_task1_mask_outputs,
)
from .two_map_planner import (
    GridCalibration,
    Portal,
    TwoMapRoute,
    build_normal_maze_map,
    build_obstacle_map,
    draw_normal_maze_map,
    draw_obstacle_map,
    make_clip_grid_calibration,
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
    # Measured calibration: a planned 1018.2 mm drive travels correctly when
    # the robot is commanded to drive 1000 mm.
    obstacle_distance_scale: float = 1000.0 / 1018.2
    rrt_max_waypoint_spacing_mm: float = 150.0
    grid_from_clips: bool = True


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


@dataclass
class MappingPreview:
    """Image-processing output available before portal coordinates are known."""

    source_bgr: np.ndarray
    rectified_bgr: np.ndarray
    task1_masks: dict[str, np.ndarray]
    calibration: GridCalibration
    coordinate_grid_bgr: np.ndarray
    clip_grid_result: Optional[object] = None


def default_unavailable_cells() -> set[Cell]:
    """Return the 12 black corner squares excluded from the 69-cell maze."""
    return {
        (0, 0), (0, 1), (0, 7), (0, 8),
        (1, 0), (1, 8),
        (7, 0), (7, 8),
        (8, 0), (8, 1), (8, 7), (8, 8),
    }


def draw_coordinate_grid(
    rectified_bgr: np.ndarray,
    calibration: GridCalibration,
) -> np.ndarray:
    """Overlay every logical ``(row, column)`` before route configuration."""
    display = rectified_bgr.copy()
    unavailable = default_unavailable_cells()

    for column in range(calibration.columns + 1):
        endpoints = calibration.project(((column, 0), (column, calibration.rows)))
        cv2.line(
            display,
            tuple(np.rint(endpoints[0]).astype(int)),
            tuple(np.rint(endpoints[1]).astype(int)),
            (255, 180, 0),
            1,
            cv2.LINE_AA,
        )
    for row in range(calibration.rows + 1):
        endpoints = calibration.project(((0, row), (calibration.columns, row)))
        cv2.line(
            display,
            tuple(np.rint(endpoints[0]).astype(int)),
            tuple(np.rint(endpoints[1]).astype(int)),
            (255, 180, 0),
            1,
            cv2.LINE_AA,
        )

    overlay = display.copy()
    for cell in unavailable:
        cv2.fillConvexPoly(
            overlay,
            np.rint(calibration.cell_quad(cell)).astype(np.int32),
            (35, 35, 35),
            cv2.LINE_AA,
        )
    display = cv2.addWeighted(display, 0.72, overlay, 0.28, 0.0)

    for row in range(calibration.rows):
        for column in range(calibration.columns):
            centre_x, centre_y = calibration.centre((row, column))
            label = f"{row},{column}"
            text_size, _ = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.25,
                1,
            )
            origin = (
                centre_x - text_size[0] // 2,
                centre_y + text_size[1] // 2,
            )
            cv2.putText(
                display,
                label,
                origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.25,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                label,
                origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.25,
                (0, 40, 255),
                1,
                cv2.LINE_AA,
            )
    return display


def prepare_mapping_preview(config: DemoConfig) -> MappingPreview:
    """Mask the source photo, rectify its masks, and label the planning grid."""
    source = cv2.imread(str(config.image_file), cv2.IMREAD_COLOR)
    if source is None:
        raise FileNotFoundError(f"Could not load maze image: {config.image_file}")
    source_task1_masks = create_maze_masks(source)
    rectified = rectify_course(
        source,
        config.output_pixels,
        config.board_corners,
    )
    task1_masks = rectify_task1_mask_outputs(
        source_task1_masks,
        config.output_pixels,
        config.board_corners,
    )
    clip_grid_result = None
    if config.grid_from_clips:
        image_transform = course_rectification_transform(
            source.shape,
            config.output_pixels,
            config.board_corners,
        )
        top, left = config.obstacle_top_left
        calibration, clip_grid_result = make_clip_grid_calibration(
            source,
            rows=9,
            columns=9,
            image_transform=image_transform,
            excluded_logical_regions=(
                (top, left, config.obstacle_size, config.obstacle_size),
            ),
        )
    else:
        calibration = make_grid_calibration(*config.grid_bounds)
    return MappingPreview(
        source_bgr=source,
        rectified_bgr=rectified,
        task1_masks=task1_masks,
        calibration=calibration,
        coordinate_grid_bgr=draw_coordinate_grid(rectified, calibration),
        clip_grid_result=clip_grid_result,
    )


def run_two_map_demo(
    config: DemoConfig,
    preview: Optional[MappingPreview] = None,
) -> DemoOutput:
    """Plan from a prepared Task 1 mask, or prepare it when not supplied."""
    if preview is None:
        preview = prepare_mapping_preview(config)
    source = preview.source_bgr
    rectified = preview.rectified_bgr
    task1_masks = preview.task1_masks
    calibration = preview.calibration
    normal_map = build_normal_maze_map(
        task1_masks,
        calibration,
        config.obstacle_top_left,
        config.obstacle_size,
        config.entrance,
        config.exit,
        unavailable_cells=default_unavailable_cells(),
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
        config.obstacle_distance_scale,
        config.rrt_max_waypoint_spacing_mm,
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
