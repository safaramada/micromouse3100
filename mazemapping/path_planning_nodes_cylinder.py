"""
path_planning_nodes_cylinders.py

Creates a fixed 9 x 9 node lattice using the cylinder-aware planning mask.

Behaviour:
- Calls create_maze_masks() from mask_maze_cylinders.py.
- Uses the buffered-cylinder planning map directly in memory.
- Generates every fixed lattice node, even when it lies on an obstacle.
- Marks a node red when any blocked pixel lies within NODE_COLLISION_RADIUS.
- Marks a node green when its surrounding region is free.
- Does not add another global clearance layer by default, preventing
  accidental double inflation of walls and cylinders.

Required folder layout:
    your_project/
        mask_maze_cylinders.py
        path_planning_nodes_cylinders.py
        cylinder_maze.png

Run:
    py path_planning_nodes_cylinders.py "cylinder_maze.png"

Example:
    py path_planning_nodes_cylinders.py "cylinder_maze.png" ^
        --cylinder-buffer 10 --node-collision-radius 3 --show-mask
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, NamedTuple, Tuple

import cv2
import numpy as np

try:
    from mask_maze_cylinders import (
        create_board_mask,
        create_maze_masks,
    )
except ImportError as error:
    raise ImportError(
        "Could not import mask_maze_cylinders.py. Put it in the same "
        "folder as path_planning_nodes_cylinders.py."
    ) from error


class GridNode(NamedTuple):
    """One fixed lattice node."""

    node_id: int
    row: int
    column: int
    x: int
    y: int
    blocked: bool


# ------------------------- Main tuning variables ------------------------- #

# Number of node positions.
NODE_ROWS = 9
NODE_COLUMNS = 9

# Move the rectangular node lattice inward from the board bounds.
GRID_INSET_X_FRACTION = 0.08
GRID_INSET_Y_FRACTION = 0.08

# The cylinder mask already includes its own buffer. Leave this at zero
# unless you intentionally want to inflate every obstacle again.
GLOBAL_CLEARANCE_PIXELS = 0

# A node is blocked when any black planning-map pixel exists within this
# circular radius around the node centre.
NODE_COLLISION_RADIUS = 3

# Passed into mask_maze_cylinders.create_maze_masks().
CYLINDER_BUFFER_PIXELS = 10

NODE_RADIUS = 4
FREE_THRESHOLD = 127

# OpenCV uses BGR colour ordering.
FREE_NODE_COLOUR = (0, 255, 0)
BLOCKED_NODE_COLOUR = (0, 0, 255)
GRID_LINE_COLOUR = (100, 220, 100)
LABEL_COLOUR = (255, 0, 255)


def ensure_binary_planning_map(planning_map: np.ndarray) -> np.ndarray:
    """Return a strict binary map: 255 free and 0 blocked."""
    if planning_map.ndim == 3:
        planning_map = cv2.cvtColor(
            planning_map,
            cv2.COLOR_BGR2GRAY,
        )

    _, binary_map = cv2.threshold(
        planning_map,
        FREE_THRESHOLD,
        255,
        cv2.THRESH_BINARY,
    )

    return binary_map


def add_global_clearance(
    planning_map: np.ndarray,
    clearance_pixels: int,
) -> np.ndarray:
    """
    Optionally inflate every obstacle.

    Normally leave this at zero because mask_maze_cylinders.py already
    applies a dedicated safety buffer to the cylinders.
    """
    if clearance_pixels < 0:
        raise ValueError("Global clearance must be zero or greater.")

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


def node_is_blocked(
    planning_map: np.ndarray,
    board_mask: np.ndarray,
    x: int,
    y: int,
    collision_radius: int,
) -> bool:
    """
    Return True when the node's circular footprint touches a blocked pixel.

    This checks the node region against:
    - walls,
    - cylinders,
    - cylinder safety buffers,
    - the board boundary,
    - space outside the board.
    """
    if collision_radius < 0:
        raise ValueError("Node collision radius must be zero or greater.")

    height, width = planning_map.shape

    if not (0 <= x < width and 0 <= y < height):
        return True

    if board_mask[y, x] != 255:
        return True

    # A zero-radius check is exactly the centre-pixel test.
    if collision_radius == 0:
        return planning_map[y, x] != 255

    node_region_mask = np.zeros_like(planning_map)

    cv2.circle(
        node_region_mask,
        (x, y),
        collision_radius,
        255,
        thickness=-1,
        lineType=cv2.LINE_8,
    )

    node_region = node_region_mask > 0
    obstacle_region = planning_map == 0
    outside_board = board_mask == 0

    return bool(
        np.any(node_region & obstacle_region)
        or np.any(node_region & outside_board)
    )


def calculate_grid_bounds(
    board_mask: np.ndarray,
    inset_x_fraction: float,
    inset_y_fraction: float,
) -> Tuple[int, int, int, int]:
    """Return left, top, right, and bottom bounds for the fixed grid."""
    if not 0.0 <= inset_x_fraction < 0.5:
        raise ValueError("--inset-x must be between 0 and 0.5.")

    if not 0.0 <= inset_y_fraction < 0.5:
        raise ValueError("--inset-y must be between 0 and 0.5.")

    board_y, board_x = np.where(board_mask == 255)

    if board_x.size == 0 or board_y.size == 0:
        raise ValueError("The board mask contains no playable region.")

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
        raise ValueError("The selected grid inset is too large.")

    return left, top, right, bottom


def generate_fixed_grid_nodes(
    planning_map: np.ndarray,
    board_mask: np.ndarray,
    rows: int,
    columns: int,
    inset_x_fraction: float,
    inset_y_fraction: float,
    node_collision_radius: int,
) -> List[GridNode]:
    """
    Generate every fixed node, then classify it as free or blocked.

    Nodes are not deleted when they lie on an obstacle. They remain in the
    lattice and are coloured red so later graph construction can exclude them.
    """
    if rows < 2:
        raise ValueError("The fixed grid needs at least two rows.")

    if columns < 2:
        raise ValueError("The fixed grid needs at least two columns.")

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

            blocked = node_is_blocked(
                planning_map=planning_map,
                board_mask=board_mask,
                x=x,
                y=y,
                collision_radius=node_collision_radius,
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


def build_node_lookup(nodes: List[GridNode]) -> dict:
    """Map each (row, column) pair to its GridNode."""
    return {
        (node.row, node.column): node
        for node in nodes
    }


def draw_candidate_lattice(
    image: np.ndarray,
    nodes: List[GridNode],
    rows: int,
    columns: int,
) -> None:
    """
    Draw all candidate lattice connections.

    These are visual guides only. The shortest-path file must separately
    reject any edge that crosses a black obstacle region.
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
    """Draw the fixed grid with green free nodes and red blocked nodes."""
    output = base_image.copy()

    draw_candidate_lattice(
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
    """Resize only the displayed copy when required."""
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
            "Create and classify a fixed node lattice using the buffered "
            "cylinder planning map."
        ),
    )

    parser.add_argument(
        "image",
        type=Path,
        help="Path to the original cylinder-maze image.",
    )

    parser.add_argument(
        "--rows",
        type=int,
        default=NODE_ROWS,
        help="Number of node rows. Default: {}.".format(NODE_ROWS),
    )

    parser.add_argument(
        "--columns",
        type=int,
        default=NODE_COLUMNS,
        help="Number of node columns. Default: {}.".format(NODE_COLUMNS),
    )

    parser.add_argument(
        "--inset-x",
        type=float,
        default=GRID_INSET_X_FRACTION,
        help="Horizontal fixed-grid inset fraction.",
    )

    parser.add_argument(
        "--inset-y",
        type=float,
        default=GRID_INSET_Y_FRACTION,
        help="Vertical fixed-grid inset fraction.",
    )

    parser.add_argument(
        "--cylinder-buffer",
        type=int,
        default=CYLINDER_BUFFER_PIXELS,
        help=(
            "Cylinder safety buffer passed to mask_maze_cylinders.py. "
            "Default: {} pixels.".format(CYLINDER_BUFFER_PIXELS)
        ),
    )

    parser.add_argument(
        "--node-collision-radius",
        type=int,
        default=NODE_COLLISION_RADIUS,
        help=(
            "Radius checked around every node. Default: {} pixels."
            .format(NODE_COLLISION_RADIUS)
        ),
    )

    parser.add_argument(
        "--global-clearance",
        type=int,
        default=GLOBAL_CLEARANCE_PIXELS,
        help=(
            "Optional extra inflation applied to every obstacle. "
            "Default: 0."
        ),
    )

    parser.add_argument(
        "--node-radius",
        type=int,
        default=NODE_RADIUS,
        help="Displayed node radius.",
    )

    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="Hide node ID labels.",
    )

    parser.add_argument(
        "--show-mask",
        action="store_true",
        help="Also display the buffered binary planning map.",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the node previews.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("cylinder_node_outputs"),
        help="Output directory used with --save.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    if args.rows < 2:
        raise ValueError("--rows must be at least 2.")

    if args.columns < 2:
        raise ValueError("--columns must be at least 2.")

    if args.cylinder_buffer < 0:
        raise ValueError("--cylinder-buffer must be zero or greater.")

    if args.node_collision_radius < 0:
        raise ValueError(
            "--node-collision-radius must be zero or greater."
        )

    if args.global_clearance < 0:
        raise ValueError("--global-clearance must be zero or greater.")

    if args.node_radius <= 0:
        raise ValueError("--node-radius must be greater than zero.")

    image_path = args.image.expanduser().resolve()

    if not image_path.is_file():
        raise FileNotFoundError(
            "Maze image not found: {}".format(image_path)
        )

    original_image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if original_image is None:
        raise ValueError(
            "OpenCV could not read the image: {}".format(image_path)
        )

    # Generate the cylinder-aware mask directly in memory.
    mask_outputs = create_maze_masks(
        image=original_image,
        cylinder_buffer_pixels=args.cylinder_buffer,
    )

    planning_map = mask_outputs[
        "07_planning_map_free_white.png"
    ]
    planning_map = ensure_binary_planning_map(planning_map)

    classified_map = add_global_clearance(
        planning_map,
        args.global_clearance,
    )

    height, width = classified_map.shape
    board_mask, _ = create_board_mask(height, width)

    nodes = generate_fixed_grid_nodes(
        planning_map=classified_map,
        board_mask=board_mask,
        rows=args.rows,
        columns=args.columns,
        inset_x_fraction=args.inset_x,
        inset_y_fraction=args.inset_y,
        node_collision_radius=args.node_collision_radius,
    )

    free_nodes = [
        node
        for node in nodes
        if not node.blocked
    ]

    blocked_nodes = [
        node
        for node in nodes
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
        "Generated all {} fixed lattice nodes ({} rows x {} columns)."
        .format(len(nodes), args.rows, args.columns)
    )
    print("Free nodes: {}".format(len(free_nodes)))
    print("Blocked nodes: {}".format(len(blocked_nodes)))
    print(
        "Blocked node IDs: {}".format(
            ", ".join(
                str(node.node_id)
                for node in blocked_nodes
            )
            if blocked_nodes
            else "none"
        )
    )
    print("Green = free node")
    print("Red = node touches a wall, cylinder, cylinder buffer, or boundary")
    print(
        "Cylinder buffer: {} pixels".format(args.cylinder_buffer)
    )
    print(
        "Node collision radius: {} pixels"
        .format(args.node_collision_radius)
    )
    print(
        "Candidate grid lines are not yet collision-checked graph edges."
    )
    print("Press any key in an image window to close the previews.")

    if args.save:
        output_directory = args.output.expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(
            str(output_directory / "01_buffered_planning_map.png"),
            classified_map,
        )
        cv2.imwrite(
            str(output_directory / "02_nodes_on_mask.png"),
            mask_preview,
        )
        cv2.imwrite(
            str(output_directory / "03_nodes_on_maze.png"),
            original_preview,
        )

        print(
            "Saved previews to: {}".format(output_directory)
        )

    if args.show_mask:
        cv2.imshow(
            "Buffered cylinder planning map",
            resize_for_display(mask_preview),
        )

    cv2.imshow(
        "Cylinder-aware fixed nodes",
        resize_for_display(original_preview),
    )

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()