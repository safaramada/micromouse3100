"""
path_planning_nodes_fixed_grid.py

Creates a fixed node lattice from the maze image.

Important behaviour:
- The node lattice is generated independently of the obstacle mask.
- A node is still created when its coordinate lies on a detected wall.
- Nodes are then classified as FREE or BLOCKED using the planning mask.
- Later path-planning stages should prevent travel through blocked nodes
  and prevent edges from crossing walls.

Nothing is saved unless --save is supplied.

Required folder layout:
    your_project/
        mask_maze.py
        path_planning_nodes_fixed_grid.py
        maze.png

Run:
    py path_planning_nodes_fixed_grid.py "maze.png"

If "9 rows and 9 columns" means 9 maze cells rather than 9 nodes,
run with 10 rows and 10 columns:
    py path_planning_nodes_fixed_grid.py "maze.png" --rows 10 --columns 10
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

import cv2
import numpy as np

try:
    from clip_grid import infer_clip_grid
    from mask_maze import create_board_mask, create_maze_masks
except ImportError as error:
    raise ImportError(
        "Could not import clip_grid.py or mask_maze.py. Put both files in "
        "the same folder as this file."
    ) from error


Point = Tuple[int, int]


class GridNode(NamedTuple):
    """One fixed lattice node."""

    node_id: int
    row: int
    column: int
    x: int
    y: int
    blocked: bool


# ------------------------- Main tuning variables ------------------------- #

# Number of NODE positions, not number of spaces between nodes.
NODE_ROWS = 9
NODE_COLUMNS = 9

# Move the fixed grid inward from the octagonal board's bounding rectangle.
# Increase these values if outer nodes are too close to the board boundary.
# Decrease them if the grid does not cover enough of the maze.
GRID_INSET_X_FRACTION = 0.08
GRID_INSET_Y_FRACTION = 0.08

# Optional additional wall clearance before classifying nodes.
# This does not affect whether nodes are created. It only changes whether
# each generated node is labelled free or blocked.
CLEARANCE_PIXELS = 0

NODE_RADIUS = 4
NODE_LABELS = True
FREE_THRESHOLD = 127

# Display colours use OpenCV's BGR ordering.
FREE_NODE_COLOUR = (0, 255, 0)
BLOCKED_NODE_COLOUR = (0, 0, 255)
GRID_LINE_COLOUR = (100, 220, 100)
LABEL_COLOUR = (255, 0, 255)


def ensure_binary_planning_map(planning_map: np.ndarray) -> np.ndarray:
    """Return a strict binary map: 255 free and 0 blocked."""
    if planning_map.ndim == 3:
        planning_map = cv2.cvtColor(planning_map, cv2.COLOR_BGR2GRAY)

    _, binary_map = cv2.threshold(
        planning_map,
        FREE_THRESHOLD,
        255,
        cv2.THRESH_BINARY,
    )

    return binary_map


def add_clearance(
    planning_map: np.ndarray,
    clearance_pixels: int,
) -> np.ndarray:
    """
    Inflate obstacles while preserving the fixed node lattice.

    Nodes are still generated at every lattice coordinate. Inflation only
    changes whether each node is classified as free or blocked.
    """
    if clearance_pixels < 0:
        raise ValueError("Clearance must be zero or greater.")

    if clearance_pixels == 0:
        return planning_map.copy()

    obstacle_mask = cv2.bitwise_not(planning_map)

    diameter = 2 * clearance_pixels + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (diameter, diameter),
    )

    inflated_obstacles = cv2.dilate(
        obstacle_mask,
        kernel,
        iterations=1,
    )

    return cv2.bitwise_not(inflated_obstacles)


def calculate_grid_bounds(
    board_mask: np.ndarray,
    inset_x_fraction: float,
    inset_y_fraction: float,
) -> Tuple[int, int, int, int]:
    """
    Calculate an axis-aligned grid region inside the octagonal board.

    Returns:
        left, top, right, bottom
    """
    if not 0.0 <= inset_x_fraction < 0.5:
        raise ValueError("Horizontal grid inset must be between 0 and 0.5.")

    if not 0.0 <= inset_y_fraction < 0.5:
        raise ValueError("Vertical grid inset must be between 0 and 0.5.")

    board_y, board_x = np.where(board_mask == 255)

    if board_x.size == 0 or board_y.size == 0:
        raise ValueError("The board mask contains no playable board region.")

    raw_left = int(board_x.min())
    raw_right = int(board_x.max())
    raw_top = int(board_y.min())
    raw_bottom = int(board_y.max())

    board_width = raw_right - raw_left
    board_height = raw_bottom - raw_top

    inset_x = int(round(board_width * inset_x_fraction))
    inset_y = int(round(board_height * inset_y_fraction))

    left = raw_left + inset_x
    right = raw_right - inset_x
    top = raw_top + inset_y
    bottom = raw_bottom - inset_y

    if left >= right or top >= bottom:
        raise ValueError("Grid inset is too large for the detected board.")

    return left, top, right, bottom


def generate_fixed_grid_nodes(
    planning_map: np.ndarray,
    board_mask: np.ndarray,
    rows: int,
    columns: int,
    inset_x_fraction: float,
    inset_y_fraction: float,
) -> List[GridNode]:
    """
    Generate every node in a fixed rows-by-columns lattice.

    Unlike the previous version, obstacle pixels do not stop node creation.
    Each node is generated first, then classified using the planning mask.
    """
    if rows < 2:
        raise ValueError("The fixed grid needs at least two rows.")

    if columns < 2:
        raise ValueError("The fixed grid needs at least two columns.")

    height, width = planning_map.shape

    left, top, right, bottom = calculate_grid_bounds(
        board_mask,
        inset_x_fraction,
        inset_y_fraction,
    )

    x_positions = np.linspace(left, right, columns)
    y_positions = np.linspace(top, bottom, rows)

    nodes: List[GridNode] = []
    node_id = 0

    for row, y_float in enumerate(y_positions):
        y = int(round(float(y_float)))

        for column, x_float in enumerate(x_positions):
            x = int(round(float(x_float)))

            inside_image = 0 <= x < width and 0 <= y < height
            inside_board = inside_image and board_mask[y, x] == 255

            # A node is blocked when it is outside the playable board or when
            # its coordinate lies on a black obstacle pixel.
            blocked = (
                not inside_board
                or planning_map[y, x] != 255
            )

            nodes.append(
                GridNode(
                    node_id=node_id,
                    row=row,
                    column=column,
                    x=x,
                    y=y,
                    blocked=blocked,
                )
            )

            node_id += 1

    return nodes


def generate_nodes_from_positions(
    planning_map: np.ndarray,
    board_mask: np.ndarray,
    rows: int,
    columns: int,
    positions: Sequence[Point],
) -> List[GridNode]:
    """Classify row-major nodes at externally calculated image positions."""
    if rows < 2 or columns < 2:
        raise ValueError("The grid needs at least two rows and columns.")

    expected_position_count = rows * columns
    if len(positions) != expected_position_count:
        raise ValueError(
            "Expected {} node positions for a {} x {} grid, but received {}."
            .format(expected_position_count, rows, columns, len(positions))
        )

    height, width = planning_map.shape
    nodes: List[GridNode] = []

    for node_id, position in enumerate(positions):
        x = int(round(float(position[0])))
        y = int(round(float(position[1])))
        row = node_id // columns
        column = node_id % columns

        inside_image = 0 <= x < width and 0 <= y < height
        inside_board = inside_image and board_mask[y, x] == 255
        blocked = (
            not inside_board
            or planning_map[y, x] != 255
        )

        nodes.append(
            GridNode(
                node_id=node_id,
                row=row,
                column=column,
                x=x,
                y=y,
                blocked=blocked,
            )
        )

    return nodes


def build_node_lookup(
    nodes: List[GridNode],
) -> dict:
    """Map (row, column) to GridNode for drawing the lattice."""
    return {
        (node.row, node.column): node
        for node in nodes
    }


def draw_lattice_edges(
    image: np.ndarray,
    nodes: List[GridNode],
    rows: int,
    columns: int,
) -> None:
    """
    Draw the complete fixed lattice for preview only.

    These lines are not yet collision-checked graph edges. The next stage
    should remove any edge that crosses a wall.
    """
    lookup = build_node_lookup(nodes)

    for row in range(rows):
        for column in range(columns):
            current = lookup[(row, column)]

            if column + 1 < columns:
                right = lookup[(row, column + 1)]
                cv2.line(
                    image,
                    (current.x, current.y),
                    (right.x, right.y),
                    GRID_LINE_COLOUR,
                    thickness=1,
                    lineType=cv2.LINE_AA,
                )

            if row + 1 < rows:
                below = lookup[(row + 1, column)]
                cv2.line(
                    image,
                    (current.x, current.y),
                    (below.x, below.y),
                    GRID_LINE_COLOUR,
                    thickness=1,
                    lineType=cv2.LINE_AA,
                )


def draw_nodes(
    base_image: np.ndarray,
    nodes: List[GridNode],
    rows: int,
    columns: int,
    radius: int,
    show_labels: bool,
) -> np.ndarray:
    """
    Draw every generated node.

    Green node = free according to the mask.
    Red node = generated normally, but located on a blocked pixel.
    """
    output = base_image.copy()

    draw_lattice_edges(
        output,
        nodes,
        rows,
        columns,
    )

    for node in nodes:
        colour = (
            BLOCKED_NODE_COLOUR
            if node.blocked
            else FREE_NODE_COLOUR
        )

        cv2.circle(
            output,
            (node.x, node.y),
            radius,
            colour,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )

        if show_labels:
            cv2.putText(
                output,
                str(node.node_id),
                (node.x + radius + 2, node.y - radius - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                LABEL_COLOUR,
                thickness=1,
                lineType=cv2.LINE_AA,
            )

    return output


def resize_for_display(
    image: np.ndarray,
    maximum_width: int = 1100,
    maximum_height: int = 850,
) -> np.ndarray:
    """Resize only the displayed copy when necessary."""
    height, width = image.shape[:2]

    scale = min(
        maximum_width / width,
        maximum_height / height,
        1.0,
    )

    if scale == 1.0:
        return image

    return cv2.resize(
        image,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a fixed node lattice independently of obstacles, then "
            "classify each node using the mask from mask_maze.py."
        ),
    )

    parser.add_argument(
        "image",
        type=Path,
        help="Path to the original maze photograph.",
    )

    parser.add_argument(
        "--rows",
        type=int,
        default=NODE_ROWS,
        help=f"Number of node rows. Default: {NODE_ROWS}.",
    )

    parser.add_argument(
        "--columns",
        type=int,
        default=NODE_COLUMNS,
        help=f"Number of node columns. Default: {NODE_COLUMNS}.",
    )

    parser.add_argument(
        "--inset-x",
        type=float,
        default=GRID_INSET_X_FRACTION,
        help=(
            "Horizontal grid inset as a fraction of board width. "
            f"Default: {GRID_INSET_X_FRACTION}."
        ),
    )

    parser.add_argument(
        "--inset-y",
        type=float,
        default=GRID_INSET_Y_FRACTION,
        help=(
            "Vertical grid inset as a fraction of board height. "
            f"Default: {GRID_INSET_Y_FRACTION}."
        ),
    )

    parser.add_argument(
        "--grid-from-clips",
        action="store_true",
        help=(
            "Detect cyan wall clips and fit a perspective-aware node grid. "
            "When omitted, the original fixed inset grid is used."
        ),
    )

    parser.add_argument(
        "--clearance",
        type=int,
        default=CLEARANCE_PIXELS,
        help=(
            "Additional obstacle inflation used only for node classification. "
            f"Default: {CLEARANCE_PIXELS}."
        ),
    )

    parser.add_argument(
        "--node-radius",
        type=int,
        default=NODE_RADIUS,
        help=f"Displayed node radius. Default: {NODE_RADIUS}.",
    )

    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="Hide node ID labels.",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the planning map and fixed-grid previews.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("fixed_grid_outputs"),
        help="Output directory used with --save.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    if args.rows < 2:
        raise ValueError("--rows must be at least 2.")

    if args.columns < 2:
        raise ValueError("--columns must be at least 2.")

    if args.clearance < 0:
        raise ValueError("--clearance must be zero or greater.")

    if args.node_radius <= 0:
        raise ValueError("--node-radius must be greater than zero.")

    image_path = args.image.expanduser().resolve()

    if not image_path.is_file():
        raise FileNotFoundError(f"Maze image not found: {image_path}")

    original_image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if original_image is None:
        raise ValueError(
            f"OpenCV could not read the image: {image_path}"
        )

    # Generate the mask in memory.
    mask_outputs = create_maze_masks(original_image)

    planning_map = mask_outputs[
        "07_planning_map_free_white.png"
    ]
    planning_map = ensure_binary_planning_map(planning_map)

    classified_map = add_clearance(
        planning_map,
        args.clearance,
    )

    height, width = classified_map.shape
    board_mask, _ = create_board_mask(height, width)

    # Generate ALL lattice nodes first, including nodes on walls.
    if args.grid_from_clips:
        clip_grid = infer_clip_grid(
            image=original_image,
            rows=args.rows,
            columns=args.columns,
        )
        nodes = generate_nodes_from_positions(
            planning_map=classified_map,
            board_mask=board_mask,
            rows=args.rows,
            columns=args.columns,
            positions=clip_grid.node_positions,
        )
        print(
            "Clip-derived grid: {} detections, {} fit inliers, "
            "median error {:.2f} pixels."
            .format(
                len(clip_grid.clip_centres),
                clip_grid.inlier_count,
                clip_grid.median_fit_error,
            )
        )
    else:
        nodes = generate_fixed_grid_nodes(
            planning_map=classified_map,
            board_mask=board_mask,
            rows=args.rows,
            columns=args.columns,
            inset_x_fraction=args.inset_x,
            inset_y_fraction=args.inset_y,
        )

    free_nodes = [
        node for node in nodes
        if not node.blocked
    ]
    blocked_nodes = [
        node for node in nodes
        if node.blocked
    ]

    original_preview = draw_nodes(
        base_image=original_image,
        nodes=nodes,
        rows=args.rows,
        columns=args.columns,
        radius=args.node_radius,
        show_labels=not args.no_labels,
    )

    mask_colour = cv2.cvtColor(
        classified_map,
        cv2.COLOR_GRAY2BGR,
    )
    mask_preview = draw_nodes(
        base_image=mask_colour,
        nodes=nodes,
        rows=args.rows,
        columns=args.columns,
        radius=args.node_radius,
        show_labels=not args.no_labels,
    )

    print(
        f"Generated all {len(nodes)} fixed lattice nodes "
        f"({args.rows} rows x {args.columns} columns)."
    )
    print(f"Free nodes: {len(free_nodes)}")
    print(f"Blocked nodes: {len(blocked_nodes)}")
    print("Green = free node")
    print("Red = node exists but is blocked by the current mask")
    print(
        "The visible grid lines are only the candidate lattice. "
        "They are not collision-checked path-planning edges yet."
    )
    print("Press any key in an image window to close both previews.")

    if args.save:
        output_directory = args.output.expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(
            str(output_directory / "01_classified_planning_map.png"),
            classified_map,
        )
        cv2.imwrite(
            str(output_directory / "02_fixed_nodes_on_mask.png"),
            mask_preview,
        )
        cv2.imwrite(
            str(output_directory / "03_fixed_nodes_on_maze.png"),
            original_preview,
        )

        print(f"Saved previews to: {output_directory}")

    cv2.imshow(
        "Fixed nodes on planning mask",
        resize_for_display(mask_preview),
    )
    cv2.imshow(
        "Fixed nodes on original maze",
        resize_for_display(original_preview),
    )

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
