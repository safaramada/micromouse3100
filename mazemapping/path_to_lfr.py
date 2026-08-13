"""
path_to_lfr.py

Calculates the same Dijkstra shortest path as shortest_path_dijkstra.py,
then converts that node path into relative micromouse commands:

    l = turn left 90 degrees
    r = turn right 90 degrees
    f = move forward by one grid edge

The robot is assumed to start facing UP relative to the image unless
--heading is supplied.

Required files in the same folder:
    mask_maze.py
    path_planning_nodes_fixed_grid.py
    shortest_path_dijkstra.py
    path_to_lfr.py
    maze.png

Run:
    py path_to_lfr.py "maze.png" --start 1 --goal 79

Nothing is saved unless --save is supplied.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import cv2

try:
    from clip_grid import infer_clip_grid
    from mask_maze import create_board_mask, create_maze_masks
    from path_planning_nodes import (
        GridNode,
        add_clearance,
        ensure_binary_planning_map,
        generate_fixed_grid_nodes,
        generate_nodes_from_positions,
    )
    from shortest_path_dijkstra import (
        build_collision_free_graph,
        dijkstra_shortest_path,
        nearest_traversable_node,
    )
except ImportError as error:
    raise ImportError(
        "Put mask_maze.py, path_planning_nodes_fixed_grid.py, and "
        "shortest_path_dijkstra.py in the same folder as path_to_lfr.py."
    ) from error


# Must match the settings used to draw/calculate the path.
NODE_ROWS = 9
NODE_COLUMNS = 9

GRID_INSET_X_FRACTION = 0.08
GRID_INSET_Y_FRACTION = 0.08

CLEARANCE_PIXELS = 0
EDGE_CHECK_THICKNESS = 3

# Clockwise ordering is important for calculating relative turns.
HEADINGS = ("up", "right", "down", "left")


def direction_between_nodes(
    first: GridNode,
    second: GridNode,
) -> str:
    """
    Return the absolute image direction from first node to second node.

    Image coordinates increase:
        x toward the right
        y toward the bottom
    """
    row_change = second.row - first.row
    column_change = second.column - first.column

    if row_change == -1 and column_change == 0:
        return "up"

    if row_change == 1 and column_change == 0:
        return "down"

    if row_change == 0 and column_change == 1:
        return "right"

    if row_change == 0 and column_change == -1:
        return "left"

    raise ValueError(
        "Nodes {} and {} are not immediate cardinal neighbours.".format(
            first.node_id,
            second.node_id,
        )
    )


def commands_for_direction_change(
    current_heading: str,
    required_heading: str,
) -> List[str]:
    """
    Return the turn command(s) followed by one forward movement.

    Examples:
        up -> up       gives ["f"]
        up -> right    gives ["r", "f"]
        up -> left     gives ["l", "f"]
        up -> down     gives ["r", "r", "f"]
    """
    current_index = HEADINGS.index(current_heading)
    required_index = HEADINGS.index(required_heading)

    clockwise_quarter_turns = (
        required_index - current_index
    ) % 4

    if clockwise_quarter_turns == 0:
        return ["f"]

    if clockwise_quarter_turns == 1:
        return ["r", "f"]

    if clockwise_quarter_turns == 3:
        return ["l", "f"]

    # A 180-degree turn. Two right turns are used consistently.
    return ["r", "r", "f"]


def path_to_lfr_commands(
    path: List[int],
    nodes: List[GridNode],
    initial_heading: str = "up",
) -> Tuple[List[str], List[str]]:
    """
    Convert a node-ID path into relative l/f/r commands.

    Returns:
        commands:
            Individual commands such as ["r", "f", "r", "f", "f"].

        absolute_directions:
            Direction travelled for every graph edge, such as
            ["right", "down", "down"].
    """
    if initial_heading not in HEADINGS:
        raise ValueError(
            "Initial heading must be one of: {}".format(
                ", ".join(HEADINGS)
            )
        )

    if len(path) < 2:
        return [], []

    node_by_id: Dict[int, GridNode] = {
        node.node_id: node
        for node in nodes
    }

    commands: List[str] = []
    absolute_directions: List[str] = []

    current_heading = initial_heading

    for index in range(len(path) - 1):
        first = node_by_id[path[index]]
        second = node_by_id[path[index + 1]]

        required_heading = direction_between_nodes(
            first,
            second,
        )

        commands.extend(
            commands_for_direction_change(
                current_heading,
                required_heading,
            )
        )

        absolute_directions.append(required_heading)
        current_heading = required_heading

    return commands, absolute_directions


def compress_forward_commands(commands: List[str]) -> str:
    """
    Produce a readable compressed form.

    Example:
        ["r", "f", "f", "f", "l", "f"]
    becomes:
        r f3 l f
    """
    output: List[str] = []
    forward_count = 0

    def flush_forward_count() -> None:
        nonlocal forward_count

        if forward_count == 1:
            output.append("f")
        elif forward_count > 1:
            output.append("f{}".format(forward_count))

        forward_count = 0

    for command in commands:
        if command == "f":
            forward_count += 1
        else:
            flush_forward_count()
            output.append(command)

    flush_forward_count()

    return " ".join(output)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate the Dijkstra path and convert it into relative "
            "left/forward/right micromouse commands."
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
        help="Number of fixed node rows.",
    )

    parser.add_argument(
        "--columns",
        type=int,
        default=NODE_COLUMNS,
        help="Number of fixed node columns.",
    )

    parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="Requested start node ID. Default: 0.",
    )

    parser.add_argument(
        "--goal",
        type=int,
        default=None,
        help="Requested goal node ID. Default: final node.",
    )

    parser.add_argument(
        "--heading",
        choices=HEADINGS,
        default="up",
        help="Initial robot orientation relative to the image. Default: up.",
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
        help="Additional obstacle inflation in pixels.",
    )

    parser.add_argument(
        "--edge-thickness",
        type=int,
        default=EDGE_CHECK_THICKNESS,
        help="Collision-check width for graph edges.",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the generated commands to a text file.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("movement_commands.txt"),
        help="Text output path used with --save.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    if args.rows < 2 or args.columns < 2:
        raise ValueError("Rows and columns must both be at least 2.")

    if args.clearance < 0:
        raise ValueError("--clearance must be zero or greater.")

    if args.edge_thickness <= 0:
        raise ValueError("--edge-thickness must be greater than zero.")

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

    # Recreate the same mask and graph used by the path-planning script.
    mask_outputs = create_maze_masks(original_image)

    planning_map = mask_outputs[
        "07_planning_map_free_white.png"
    ]
    planning_map = ensure_binary_planning_map(planning_map)

    safe_planning_map = add_clearance(
        planning_map,
        args.clearance,
    )

    height, width = safe_planning_map.shape
    board_mask, _ = create_board_mask(height, width)

    if args.grid_from_clips:
        clip_grid = infer_clip_grid(
            image=original_image,
            rows=args.rows,
            columns=args.columns,
        )
        nodes = generate_nodes_from_positions(
            planning_map=safe_planning_map,
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
            planning_map=safe_planning_map,
            board_mask=board_mask,
            rows=args.rows,
            columns=args.columns,
            inset_x_fraction=args.inset_x,
            inset_y_fraction=args.inset_y,
        )

    graph = build_collision_free_graph(
        nodes=nodes,
        planning_map=safe_planning_map,
        rows=args.rows,
        columns=args.columns,
        edge_check_thickness=args.edge_thickness,
    )

    requested_goal = (
        len(nodes) - 1
        if args.goal is None
        else args.goal
    )

    start_node = nearest_traversable_node(
        args.start,
        nodes,
        graph,
    )

    goal_node = nearest_traversable_node(
        requested_goal,
        nodes,
        graph,
    )

    path, total_distance = dijkstra_shortest_path(
        graph,
        start_node.node_id,
        goal_node.node_id,
    )

    if not path:
        raise RuntimeError(
            "No path was found between node {} and node {}.".format(
                start_node.node_id,
                goal_node.node_id,
            )
        )

    commands, absolute_directions = path_to_lfr_commands(
        path=path,
        nodes=nodes,
        initial_heading=args.heading,
    )

    command_string = "".join(commands)
    spaced_commands = " ".join(commands)
    compressed_commands = compress_forward_commands(commands)

    print("Initial heading: {}".format(args.heading))
    print(
        "Path node IDs:\n{}".format(
            " -> ".join(str(node_id) for node_id in path)
        )
    )
    print(
        "\nAbsolute movement directions:\n{}".format(
            " -> ".join(absolute_directions)
        )
    )
    print("\nL/F/R command string:")
    print(command_string)

    print("\nSpaced commands:")
    print(spaced_commands)

    print("\nCompressed commands:")
    print(compressed_commands)

    print("\nGraph moves: {}".format(len(path) - 1))
    print("Individual robot commands: {}".format(len(commands)))
    print("Path length: {:.2f} pixels".format(total_distance))

    if args.save:
        output_path = args.output.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        output_text = (
            "Initial heading: {heading}\n"
            "Start node: {start}\n"
            "Goal node: {goal}\n"
            "Path: {path}\n"
            "Absolute directions: {directions}\n"
            "Command string: {command_string}\n"
            "Spaced commands: {spaced}\n"
            "Compressed commands: {compressed}\n"
        ).format(
            heading=args.heading,
            start=start_node.node_id,
            goal=goal_node.node_id,
            path=" -> ".join(str(node_id) for node_id in path),
            directions=" -> ".join(absolute_directions),
            command_string=command_string,
            spaced=spaced_commands,
            compressed=compressed_commands,
        )

        output_path.write_text(
            output_text,
            encoding="utf-8",
        )

        print("\nSaved movement commands to:")
        print(output_path)


if __name__ == "__main__":
    main()
