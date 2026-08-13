"""Run the complete Task 2 two-map workflow without Jupyter.

Usage from the project root::

    python mazemappingtask2/maze_mapping_task2.py mazemappingtask2/maz4.png

The input is the original camera image. This script calls Task 1's
``mazemapping.mask_maze.create_maze_masks`` through
``prepare_mapping_preview``; pre-generated mask files are not required.

Edit the configuration constants below when the maze layout, camera
calibration, entrance, exit, start, or goal changes.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Optional, Sequence, Tuple


# Make ``python mazemappingtask2/maze_mapping_task2.py ...`` work from any
# current directory before importing project packages.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

from computer.task4 import ContinuousPlanner
from mazemapping.mask_maze import (
    DARK_THRESHOLD,
    GAMMA,
    THIN_LINE_THRESHOLD,
    create_maze_masks,
)
from mazemappingtask2 import (
    DemoConfig,
    MappingPreview,
    Portal,
    build_normal_maze_map,
    build_obstacle_map,
    default_unavailable_cells,
    draw_coordinate_grid,
    draw_normal_maze_map,
    draw_obstacle_map,
    make_grid_calibration,
    plan_two_map_route,
)
from mazemappingtask2.task2_pipeline import project_points_to_original, rectify_course


Cell = Tuple[int, int]
Point = Tuple[int, int]


# ---------------------------------------------------------------------------
# Maze and robot configuration
# ---------------------------------------------------------------------------

OUTPUT_PIXELS = 324
BOARD_CORNERS = None  # Optional order: top-left, top-right, bottom-right, bottom-left.
GRID_BOUNDS = (29.0, 27.0, 299.0, 297.0)  # left, top, right, bottom after rectification.

OBSTACLE_TOP_LEFT: Cell = (0, 2)
OBSTACLE_SIZE = 5

START: Cell = (6, 0)
GOAL: Cell = (5, 7)
INITIAL_HEADING_DEG = 90.0
GOAL_HEADING_DEG = None

ENTRANCE_OUTSIDE: Cell = (3, 1)
ENTRANCE_INSIDE: Cell = (3, 2)
EXIT_INSIDE: Cell = (1, 6)
EXIT_OUTSIDE: Cell = (1, 7)

ROBOT_RADIUS_MM = 30.0
SAFETY_MARGIN_MM = 5.0
OBSTACLE_RESOLUTION_MM = 5.0
OBSTACLE_DISTANCE_SCALE = 0.97
RRT_MAX_WAYPOINT_SPACING_MM = 150.0
ENDPOINT_CLEAR_RADIUS_PIXELS = 25

# Use the exact Task 1 defaults. Masking is performed on the original image
# before rectification, matching a direct mask_maze.py invocation.
MASK_DARK_THRESHOLD = DARK_THRESHOLD
MASK_THIN_LINE_THRESHOLD = THIN_LINE_THRESHOLD
MASK_GAMMA = GAMMA


def make_config(image_file: Path) -> DemoConfig:
    """Create the same configuration previously assembled in the notebook."""
    return DemoConfig(
        image_file=image_file,
        output_pixels=OUTPUT_PIXELS,
        board_corners=BOARD_CORNERS,
        grid_bounds=GRID_BOUNDS,
        obstacle_top_left=OBSTACLE_TOP_LEFT,
        obstacle_size=OBSTACLE_SIZE,
        entrance=Portal(
            outside_cell=ENTRANCE_OUTSIDE,
            inside_cell=ENTRANCE_INSIDE,
        ),
        exit=Portal(
            outside_cell=EXIT_OUTSIDE,
            inside_cell=EXIT_INSIDE,
        ),
        start=START,
        goal=GOAL,
        initial_heading_deg=INITIAL_HEADING_DEG,
        goal_heading_deg=GOAL_HEADING_DEG,
        robot_radius_mm=ROBOT_RADIUS_MM,
        safety_margin_mm=SAFETY_MARGIN_MM,
        obstacle_resolution_mm=OBSTACLE_RESOLUTION_MM,
        obstacle_distance_scale=OBSTACLE_DISTANCE_SCALE,
        rrt_max_waypoint_spacing_mm=RRT_MAX_WAYPOINT_SPACING_MM,
        endpoint_clear_radius_pixels=ENDPOINT_CLEAR_RADIUS_PIXELS,
    )


def prepare_input(
    config: DemoConfig,
    dark_threshold: int,
    thin_line_threshold: int,
    gamma: float,
) -> MappingPreview:
    """Load, rectify, and pass the image through Task 1 masking exactly once."""
    source = cv2.imread(str(config.image_file), cv2.IMREAD_COLOR)
    if source is None:
        raise FileNotFoundError(f"Could not load maze image: {config.image_file}")
    source_task1_masks = create_maze_masks(
        source,
        dark_threshold=dark_threshold,
        thin_line_threshold=thin_line_threshold,
        gamma=gamma,
    )
    rectified = rectify_course(
        source,
        config.output_pixels,
        config.board_corners,
    )
    # Detect walls at full camera resolution, then align those completed masks
    # to planner coordinates. Nearest-neighbour preserves binary mask values.
    task1_masks = {
        name: rectify_course(
            output,
            config.output_pixels,
            config.board_corners,
            interpolation=(
                cv2.INTER_NEAREST if output.ndim == 2 else cv2.INTER_AREA
            ),
        )
        for name, output in source_task1_masks.items()
    }
    calibration = make_grid_calibration(*config.grid_bounds)
    return MappingPreview(
        source_bgr=source,
        rectified_bgr=rectified,
        task1_masks=task1_masks,
        calibration=calibration,
        coordinate_grid_bgr=draw_coordinate_grid(rectified, calibration),
    )


def _wall_segment(
    edge: frozenset,
    calibration,
    trim_fraction: float = 0.14,
) -> Tuple[Point, Point]:
    """Return the nominal shared cell boundary drawn by the wall diagnostic."""
    first, second = tuple(edge)
    row, column = first
    next_row, next_column = second

    if row == next_row:
        fixed = calibration.x_edges[max(column, next_column)]
        segment_start = calibration.y_edges[row]
        segment_end = calibration.y_edges[row + 1]
        trim = trim_fraction * (segment_end - segment_start)
        return (
            (int(round(fixed)), int(round(segment_start + trim))),
            (int(round(fixed)), int(round(segment_end - trim))),
        )

    fixed = calibration.y_edges[max(row, next_row)]
    segment_start = calibration.x_edges[column]
    segment_end = calibration.x_edges[column + 1]
    trim = trim_fraction * (segment_end - segment_start)
    return (
        (int(round(segment_start + trim)), int(round(fixed))),
        (int(round(segment_end - trim)), int(round(fixed))),
    )


def draw_blocked_wall_outlines(
    image_bgr: np.ndarray,
    normal_map,
    line_thickness: int = 3,
) -> np.ndarray:
    """Draw only boundaries that the normal planner treats as blocked.

    Every boundary treated as blocked is red. Open boundaries are omitted.
    """
    display = image_bgr.copy()
    for edge, evidence in normal_map.walls.items():
        if not evidence.blocked:
            continue
        start, end = _wall_segment(edge, normal_map.calibration)
        cv2.line(display, start, end, (0, 0, 255), line_thickness, cv2.LINE_AA)
    return display


def build_safety_view(obstacle_map, clearance_mm: float) -> np.ndarray:
    """Return the exact pre-RRT* obstacle and clearance visualization in BGR."""
    inflated_grid = ContinuousPlanner._inflate(obstacle_map.grid, clearance_mm)
    raw_obstacles = obstacle_map.occupancy_mask > 0
    inflated_obstacles = np.asarray(inflated_grid.cells, dtype=bool).reshape(
        inflated_grid.height,
        inflated_grid.width,
    )
    buffer_only = inflated_obstacles & ~raw_obstacles

    view = np.full((*raw_obstacles.shape, 3), 242, dtype=np.uint8)
    view[buffer_only] = (11, 158, 245)  # amber in BGR
    view[raw_obstacles] = (38, 38, 220)  # red in BGR
    return view


def _labelled_panel(
    image_bgr: np.ndarray,
    title: str,
    target_height: int = 620,
) -> np.ndarray:
    """Resize an image and add a simple title bar for OpenCV previews."""
    if image_bgr.ndim == 2:
        image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)
    scale = target_height / float(image_bgr.shape[0])
    resized = cv2.resize(
        image_bgr,
        (max(1, int(round(image_bgr.shape[1] * scale))), target_height),
        interpolation=cv2.INTER_NEAREST,
    )
    title_height = 48
    panel = cv2.copyMakeBorder(
        resized,
        title_height,
        0,
        0,
        0,
        cv2.BORDER_CONSTANT,
        value=(245, 245, 245),
    )
    cv2.putText(
        panel,
        title,
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (25, 25, 25),
        2,
        cv2.LINE_AA,
    )
    return panel


def _combine_panels(*panels: Tuple[str, np.ndarray]) -> np.ndarray:
    """Join labelled images horizontally at a common height."""
    return cv2.hconcat([_labelled_panel(image, title) for title, image in panels])


def present_image(
    image_bgr: np.ndarray,
    name: str,
    show: bool,
    output_directory: Optional[Path],
) -> None:
    """Save a diagnostic if requested, then display it with OpenCV."""
    if output_directory is not None:
        output_directory.mkdir(parents=True, exist_ok=True)
        output_path = output_directory / f"{name}.png"
        if not cv2.imwrite(str(output_path), image_bgr):
            raise OSError(f"Could not save preview: {output_path}")
        print(f"Saved: {output_path}")
    if show:
        window_name = name.replace("_", " ").title()
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, image_bgr)
        print(f"Showing '{window_name}'. Press any key in the image window to continue.")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)


def show_task1_mask(
    mapping_preview,
    show: bool,
    output_directory: Optional[Path],
) -> None:
    """Stage 1 visual: show the exact Task 1 mask output."""
    detected = mapping_preview.task1_masks["05_cleaned_wall_mask.png"] > 0
    detected_layer = mapping_preview.rectified_bgr.copy()
    detected_layer[detected] = (0, 0, 255)
    detected_overlay = cv2.addWeighted(
        mapping_preview.rectified_bgr,
        0.68,
        detected_layer,
        0.32,
        0.0,
    )
    preview = _combine_panels(
        (
            "Task 1 detected walls (red)",
            detected_overlay,
        ),
        (
            "Planning mask: white = free; outside black",
            mapping_preview.task1_masks["07_planning_map_free_white.png"],
        ),
    )
    present_image(preview, "01_task1_mask", show, output_directory)


def show_wall_outlines(
    wall_view_bgr: np.ndarray,
    show: bool,
    output_directory: Optional[Path],
) -> None:
    """Stage 2 visual: show only assumed blocked normal-maze walls."""
    preview = _labelled_panel(
        wall_view_bgr,
        "Normal-map assumed walls (red)",
    )
    present_image(preview, "02_assumed_walls", show, output_directory)


def show_cylinder_safety_map(
    obstacle_map,
    safety_view_bgr: np.ndarray,
    clearance_mm: float,
    show: bool,
    output_directory: Optional[Path],
) -> None:
    """Stage 3 visual: show the cylinder crop and pre-RRT* clearance mask."""
    marked_safety = safety_view_bgr.copy()
    cv2.circle(marked_safety, obstacle_map.entrance_local, 5, (255, 255, 0), -1)
    cv2.circle(marked_safety, obstacle_map.exit_local, 5, (255, 0, 180), -1)
    preview = _combine_panels(
        ("Selected 5 x 5 cylinder crop", obstacle_map.crop_bgr),
        (
            f"Red = blocked, amber = {clearance_mm:.1f} mm clearance",
            marked_safety,
        ),
    )
    present_image(preview, "03_cylinder_safety_map", show, output_directory)


def draw_route_on_original(source_bgr, rectified_bgr, route, board_corners):
    """Project the complete joined path back onto the input camera image."""
    display = source_bgr.copy()
    original_points = project_points_to_original(
        route.full_waypoints,
        source_bgr.shape,
        rectified_bgr.shape,
        board_corners,
    )
    if len(original_points) >= 2:
        cv2.polylines(
            display,
            [np.asarray(original_points, dtype=np.int32)],
            False,
            (0, 210, 0),
            7,
            cv2.LINE_AA,
        )
    for point in original_points:
        cv2.circle(display, point, 7, (255, 120, 0), -1, cv2.LINE_AA)
    return display


def show_planned_maps(
    mapping_preview,
    normal_map,
    obstacle_map,
    route,
    config: DemoConfig,
    show: bool,
    output_directory: Optional[Path],
) -> None:
    """Show the two solved maps followed by the complete camera-image route."""
    normal_preview = draw_normal_maze_map(
        mapping_preview.rectified_bgr,
        normal_map,
        route.normal_before_cells,
        route.normal_after_cells,
    )
    obstacle_preview = draw_obstacle_map(obstacle_map, route.obstacle_result)

    planned_maps = _combine_panels(
        ("Map 1: normal maze cell graph", normal_preview),
        ("Map 2: continuous cylinder RRT*", obstacle_preview),
    )
    present_image(planned_maps, "04_planned_maps", show, output_directory)

    original_preview = draw_route_on_original(
        mapping_preview.source_bgr,
        mapping_preview.rectified_bgr,
        route,
        config.board_corners,
    )
    complete_route = _labelled_panel(
        original_preview,
        "Normal graph -> cylinder RRT* -> normal graph",
    )
    present_image(complete_route, "05_complete_route", show, output_directory)


def run(
    image_file: Path,
    show: bool = True,
    output_directory: Optional[Path] = None,
    dark_threshold: int = MASK_DARK_THRESHOLD,
    thin_line_threshold: int = MASK_THIN_LINE_THRESHOLD,
    gamma: float = MASK_GAMMA,
) -> str:
    """Run masking, diagnostics, both planners, and return the command string."""
    image_file = image_file.expanduser().resolve()
    if not image_file.is_file():
        raise FileNotFoundError(f"Image file not found: {image_file}")
    if not 0 <= dark_threshold <= 255:
        raise ValueError("dark threshold must be between 0 and 255")
    if not 0 <= thin_line_threshold <= 255:
        raise ValueError("thin-line threshold must be between 0 and 255")
    if gamma <= 0.0:
        raise ValueError("mask gamma must be greater than zero")

    config = make_config(image_file)

    # Calls Task 1's create_maze_masks(rectified) exactly once.
    mapping_preview = prepare_input(
        config,
        dark_threshold,
        thin_line_threshold,
        gamma,
    )
    print(
        "Mask settings: "
        f"dark threshold={dark_threshold}, "
        f"thin-line threshold={thin_line_threshold}, gamma={gamma:.2f}"
    )
    show_task1_mask(mapping_preview, show, output_directory)

    normal_map = build_normal_maze_map(
        mapping_preview.task1_masks,
        mapping_preview.calibration,
        config.obstacle_top_left,
        config.obstacle_size,
        config.entrance,
        config.exit,
        unavailable_cells=default_unavailable_cells(),
        endpoint_clearings=(
            (
                mapping_preview.calibration.centre(config.start),
                config.endpoint_clear_radius_pixels,
            ),
            (
                mapping_preview.calibration.centre(config.goal),
                config.endpoint_clear_radius_pixels,
            ),
        ),
    )
    wall_view = draw_blocked_wall_outlines(
        mapping_preview.rectified_bgr,
        normal_map,
    )
    show_wall_outlines(wall_view, show, output_directory)

    obstacle_map = build_obstacle_map(
        mapping_preview.rectified_bgr,
        mapping_preview.task1_masks,
        mapping_preview.calibration,
        config.obstacle_top_left,
        config.obstacle_size,
        config.entrance,
        config.exit,
        resolution_mm=config.obstacle_resolution_mm,
    )
    clearance_mm = config.robot_radius_mm + config.safety_margin_mm
    safety_view = build_safety_view(obstacle_map, clearance_mm)
    show_cylinder_safety_map(
        obstacle_map,
        safety_view,
        clearance_mm,
        show,
        output_directory,
    )

    print(f"Detected cylinder count: {len(obstacle_map.detected_centres)}")
    print(f"Detected cylinder centres: {obstacle_map.detected_centres}")
    print(f"Total cylinder clearance: {clearance_mm:.1f} mm")

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

    print(f"Stage 1 - normal cells: {route.normal_before_cells}")
    print(f"Stage 2 - cylinder RRT*: {route.obstacle_result.status.value}")
    print(f"Stage 3 - normal cells: {route.normal_after_cells}")

    show_planned_maps(
        mapping_preview,
        normal_map,
        obstacle_map,
        route,
        config,
        show,
        output_directory,
    )

    print("\nFinal command string:")
    print(route.command)
    return route.command


def parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Mask a Task 2 maze image, preview assumed walls and cylinder "
            "clearance, run both planners, and print the robot command string."
        )
    )
    parser.add_argument("image", type=Path, help="Original maze camera image")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory in which to save every displayed figure",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open figure windows (useful with --output-dir)",
    )
    parser.add_argument(
        "--dark-threshold",
        type=int,
        default=MASK_DARK_THRESHOLD,
        help=(
            "Maximum grayscale value treated as a thick wall; lower this "
            f"to reject more floor shadows (default: {MASK_DARK_THRESHOLD})"
        ),
    )
    parser.add_argument(
        "--thin-line-threshold",
        type=int,
        default=MASK_THIN_LINE_THRESHOLD,
        help=(
            "Thin-wall response threshold; raise this to reject fine details "
            f"(default: {MASK_THIN_LINE_THRESHOLD})"
        ),
    )
    parser.add_argument(
        "--mask-gamma",
        type=float,
        default=MASK_GAMMA,
        help=f"Task 1 mask gamma (default: {MASK_GAMMA})",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_arguments(argv)
    try:
        run(
            args.image,
            show=not args.no_show,
            output_directory=args.output_dir,
            dark_threshold=args.dark_threshold,
            thin_line_threshold=args.thin_line_threshold,
            gamma=args.mask_gamma,
        )
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"\nPlanning stopped: {error}", file=sys.stderr)
        print(
            "The mask and assumed-wall previews above are available before "
            "the path search, so use them to locate the rejected opening.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
