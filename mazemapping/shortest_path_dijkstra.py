"""
shortest_path_dijkstra.py

Builds a collision-free graph from the fixed maze-node grid and calculates
the shortest path using Dijkstra's algorithm.

Why Dijkstra here:
- The map is already represented as a small fixed graph.
- DFS does not guarantee the shortest path.
- RRT* is unnecessary for a known 9 x 9 grid.
- BFS is also valid when every edge has exactly the same cost.
- Dijkstra also remains correct when horizontal and vertical pixel distances
  are slightly different.

Required files in the same folder:
    mask_maze.py
    path_planning_nodes_fixed_grid.py
    shortest_path_dijkstra.py
    maze.png

Run:
    py shortest_path_dijkstra.py "maze.png"

The default requested start is node 0 and the default requested goal is the
last node. If either is blocked, the program automatically uses the nearest
free node that has at least one valid graph connection.

Nothing is saved unless --save is supplied.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

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
except ImportError as error:
    raise ImportError(
        "Put clip_grid.py, mask_maze.py, and path_planning_nodes.py in the "
        "same folder as shortest_path_dijkstra.py."
    ) from error


# ------------------------- Main tuning variables ------------------------- #

NODE_ROWS = 9
NODE_COLUMNS = 9

GRID_INSET_X_FRACTION = 0.08
GRID_INSET_Y_FRACTION = 0.08

CLEARANCE_PIXELS = 0

# Thickness of the band checked between neighbouring nodes.
# Increase this to reject edges that pass too close to a wall.
EDGE_CHECK_THICKNESS = 3

NODE_RADIUS = 4
PATH_THICKNESS = 7
VALID_EDGE_THICKNESS = 1

FREE_NODE_COLOUR = (0, 255, 0)
BLOCKED_NODE_COLOUR = (0, 0, 255)
VALID_EDGE_COLOUR = (150, 150, 150)
PATH_COLOUR = (255, 0, 0)
START_COLOUR = (0, 255, 255)
GOAL_COLOUR = (255, 0, 255)
LABEL_COLOUR = (255, 0, 255)


Adjacency = Dict[int, List[Tuple[int, float]]]


def build_node_lookup(nodes: List[GridNode]) -> Dict[Tuple[int, int], GridNode]:
    """Map each logical grid coordinate to its node."""
    return {
        (node.row, node.column): node
        for node in nodes
    }


def edge_is_collision_free(
    planning_map: np.ndarray,
    first: GridNode,
    second: GridNode,
    check_thickness: int,
) -> bool:
    """
    Return True only when the entire band between two nodes is free.

    The mask convention is:
        255 = free
          0 = blocked
    """
    if first.blocked or second.blocked:
        return False

    if check_thickness <= 0:
        raise ValueError("Edge-check thickness must be greater than zero.")

    test_band = np.zeros_like(planning_map)

    cv2.line(
        test_band,
        (first.x, first.y),
        (second.x, second.y),
        255,
        thickness=check_thickness,
        lineType=cv2.LINE_8,
    )

    checked_pixels = test_band > 0
    blocked_pixels = planning_map == 0

    return not bool(np.any(checked_pixels & blocked_pixels))


def build_collision_free_graph(
    nodes: List[GridNode],
    planning_map: np.ndarray,
    rows: int,
    columns: int,
    edge_check_thickness: int,
) -> Adjacency:
    """
    Connect only immediate up/down/left/right neighbours.

    Every candidate edge is checked against the complete obstacle mask.
    """
    lookup = build_node_lookup(nodes)

    graph: Adjacency = {
        node.node_id: []
        for node in nodes
        if not node.blocked
    }

    # Check right and down only, then insert each valid edge in both directions.
    candidate_offsets = [
        (0, 1),
        (1, 0),
    ]

    for row in range(rows):
        for column in range(columns):
            current = lookup[(row, column)]

            if current.blocked:
                continue

            for row_offset, column_offset in candidate_offsets:
                neighbour_row = row + row_offset
                neighbour_column = column + column_offset

                if neighbour_row >= rows or neighbour_column >= columns:
                    continue

                neighbour = lookup[(neighbour_row, neighbour_column)]

                if not edge_is_collision_free(
                    planning_map,
                    current,
                    neighbour,
                    edge_check_thickness,
                ):
                    continue

                distance = math.hypot(
                    neighbour.x - current.x,
                    neighbour.y - current.y,
                )

                graph[current.node_id].append(
                    (neighbour.node_id, distance)
                )
                graph[neighbour.node_id].append(
                    (current.node_id, distance)
                )

    return graph


def nearest_traversable_node(
    requested_node_id: int,
    nodes: List[GridNode],
    graph: Adjacency,
) -> GridNode:
    """
    Return the requested node when traversable, otherwise the nearest node
    that is free and has at least one valid edge.
    """
    node_by_id = {
        node.node_id: node
        for node in nodes
    }

    if requested_node_id not in node_by_id:
        raise ValueError(
            "Requested node ID {} is outside the valid range 0 to {}.".format(
                requested_node_id,
                len(nodes) - 1,
            )
        )

    requested = node_by_id[requested_node_id]

    if (
        not requested.blocked
        and requested.node_id in graph
        and len(graph[requested.node_id]) > 0
    ):
        return requested

    candidates = [
        node
        for node in nodes
        if (
            not node.blocked
            and node.node_id in graph
            and len(graph[node.node_id]) > 0
        )
    ]

    if not candidates:
        raise RuntimeError(
            "No traversable nodes exist. Check the mask, node placement, "
            "clearance, and edge-check thickness."
        )

    return min(
        candidates,
        key=lambda node: (
            abs(node.row - requested.row) + abs(node.column - requested.column),
            math.hypot(node.x - requested.x, node.y - requested.y),
            node.node_id,
        ),
    )


def dijkstra_shortest_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
) -> Tuple[List[int], float]:
    """Return the minimum-cost path and its total pixel length."""
    infinity = float("inf")

    distances: Dict[int, float] = {
        node_id: infinity
        for node_id in graph
    }
    previous: Dict[int, Optional[int]] = {
        node_id: None
        for node_id in graph
    }

    distances[start_id] = 0.0

    queue: List[Tuple[float, int]] = [
        (0.0, start_id)
    ]

    visited = set()

    while queue:
        current_distance, current_id = heapq.heappop(queue)

        if current_id in visited:
            continue

        visited.add(current_id)

        if current_id == goal_id:
            break

        for neighbour_id, edge_cost in graph.get(current_id, []):
            new_distance = current_distance + edge_cost

            if new_distance < distances[neighbour_id]:
                distances[neighbour_id] = new_distance
                previous[neighbour_id] = current_id

                heapq.heappush(
                    queue,
                    (new_distance, neighbour_id),
                )

    if distances.get(goal_id, infinity) == infinity:
        return [], infinity

    path: List[int] = []
    current: Optional[int] = goal_id

    while current is not None:
        path.append(current)
        current = previous[current]

    path.reverse()

    return path, distances[goal_id]


def draw_graph_and_path(
    original_image: np.ndarray,
    nodes: List[GridNode],
    graph: Adjacency,
    path: List[int],
    start_id: int,
    goal_id: int,
    show_labels: bool,
) -> np.ndarray:
    """Draw valid graph edges, nodes, and the shortest path."""
    output = original_image.copy()

    node_by_id = {
        node.node_id: node
        for node in nodes
    }

    # Draw each undirected valid edge only once.
    drawn_edges = set()

    for first_id, neighbours in graph.items():
        for second_id, _ in neighbours:
            edge_key = tuple(sorted((first_id, second_id)))

            if edge_key in drawn_edges:
                continue

            drawn_edges.add(edge_key)

            first = node_by_id[first_id]
            second = node_by_id[second_id]

            cv2.line(
                output,
                (first.x, first.y),
                (second.x, second.y),
                VALID_EDGE_COLOUR,
                VALID_EDGE_THICKNESS,
                lineType=cv2.LINE_AA,
            )

    # Draw the shortest path over the graph edges.
    for index in range(len(path) - 1):
        first = node_by_id[path[index]]
        second = node_by_id[path[index + 1]]

        cv2.line(
            output,
            (first.x, first.y),
            (second.x, second.y),
            PATH_COLOUR,
            PATH_THICKNESS,
            lineType=cv2.LINE_AA,
        )

    # Draw all nodes.
    for node in nodes:
        if node.blocked:
            colour = BLOCKED_NODE_COLOUR
        else:
            colour = FREE_NODE_COLOUR

        if node.node_id == start_id:
            colour = START_COLOUR
        elif node.node_id == goal_id:
            colour = GOAL_COLOUR

        cv2.circle(
            output,
            (node.x, node.y),
            NODE_RADIUS,
            colour,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )

        if show_labels:
            cv2.putText(
                output,
                str(node.node_id),
                (node.x + NODE_RADIUS + 2, node.y - NODE_RADIUS - 2),
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
    """Resize only the displayed image when necessary."""
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


def print_path(
    path: List[int],
    nodes: List[GridNode],
    total_distance: float,
) -> None:
    """Print node IDs, coordinates, and total length."""
    node_by_id = {
        node.node_id: node
        for node in nodes
    }

    print("\nShortest path node IDs:")
    print(" -> ".join(str(node_id) for node_id in path))

    print("\nShortest path coordinates:")
    print(
        " -> ".join(
            "({}, {})".format(
                node_by_id[node_id].x,
                node_by_id[node_id].y,
            )
            for node_id in path
        )
    )

    print("\nNumber of moves: {}".format(max(0, len(path) - 1)))
    print("Total path length: {:.2f} pixels".format(total_distance))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate and draw the Dijkstra shortest path through the "
            "fixed collision-checked maze graph."
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
        default=1,
        help="Requested start node ID. Default: 0.",
    )

    parser.add_argument(
        "--goal",
        type=int,
        default=79,
        help="Requested goal node ID. Default: final node.",
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
        help=(
            "Width of the collision-check band around each candidate edge. "
            "Default: {}.".format(EDGE_CHECK_THICKNESS)
        ),
    )

    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="Hide node number labels.",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the resulting shortest-path image.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("shortest_path_outputs"),
        help="Output folder used with --save.",
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
        raise FileNotFoundError("Maze image not found: {}".format(image_path))

    original_image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if original_image is None:
        raise ValueError(
            "OpenCV could not read the image: {}".format(image_path)
        )

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

    if start_node.node_id != args.start:
        print(
            "Requested start node {} is blocked or isolated; using node {}."
            .format(args.start, start_node.node_id)
        )

    if goal_node.node_id != requested_goal:
        print(
            "Requested goal node {} is blocked or isolated; using node {}."
            .format(requested_goal, goal_node.node_id)
        )

    path, total_distance = dijkstra_shortest_path(
        graph,
        start_node.node_id,
        goal_node.node_id,
    )

    if not path:
        raise RuntimeError(
            "No path was found between node {} and node {}. "
            "This usually means the mask contains a false wall, the fixed grid "
            "does not align with the maze corridors, or edge checking is too "
            "strict. Try reducing --edge-thickness, reducing --clearance, or "
            "adjusting --inset-x/--inset-y."
            .format(start_node.node_id, goal_node.node_id)
        )

    result = draw_graph_and_path(
        original_image=original_image,
        nodes=nodes,
        graph=graph,
        path=path,
        start_id=start_node.node_id,
        goal_id=goal_node.node_id,
        show_labels=not args.no_labels,
    )

    print("\nAlgorithm: Dijkstra")
    print("Start node: {}".format(start_node.node_id))
    print("Goal node: {}".format(goal_node.node_id))
    print("Traversable graph nodes: {}".format(len(graph)))
    print(
        "Valid undirected graph edges: {}".format(
            sum(len(neighbours) for neighbours in graph.values()) // 2
        )
    )

    print_path(
        path,
        nodes,
        total_distance,
    )

    if args.save:
        output_directory = args.output.expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)

        output_path = output_directory / "dijkstra_shortest_path.png"

        if not cv2.imwrite(str(output_path), result):
            raise OSError(
                "Could not save shortest-path image: {}".format(output_path)
            )

        print("\nSaved path image to: {}".format(output_path))

    cv2.imshow(
        "Dijkstra shortest path",
        resize_for_display(result),
    )

    print("\nPress any key in the image window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
