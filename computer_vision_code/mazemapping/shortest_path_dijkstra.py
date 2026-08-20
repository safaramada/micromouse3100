"""
shortest_path_dijkstra.py

Builds a collision-free graph from the fixed maze-node grid and calculates a
wall-guided shortest path using Dijkstra's algorithm.

Each graph edge is classified from the camera mask as wall-supported or open.
The default route permits at most two consecutive open moves.  If that hard
rule makes the goal unreachable, a warning is emitted and a strongly
wall-biased fallback route is returned unless --strict-wall-rule is supplied.

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
from typing import Dict, List, Mapping, Optional, Set, Tuple
import warnings

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
    from wall_collision import segment_is_collision_free
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

# Prefer routes that keep a usable wall beside the robot.  A move is treated
# as open when neither side of its corridor contains a wall.  The strict
# search permits at most this many open moves in a row.
MAX_CONSECUTIVE_OPEN_MOVES = 2

# Perspective fitting can make two logically equal grid routes differ by about
# one image pixel. Add this small cost to every open move so the planner still
# prefers a wall-supported route over that insignificant apparent shortcut.
# A normal grid edge is roughly 110 pixels in the current images.
OPEN_MOVE_PREFERENCE_PENALTY_PIXELS = 1.0

# Score each 90-degree turn like this fraction of one additional graph move.
# This changes path selection only; it does not change the robot controller.
TURN_PENALTY_EDGE_FRACTION = 0.35

CARDINAL_HEADINGS = ("up", "right", "down", "left")

# A maze wall normally lies halfway between neighbouring path nodes.  For each
# edge, probe both side corridors across this fraction of the distance to the
# next parallel row/column of nodes.
WALL_PROBE_MIN_FRACTION = 0.25
WALL_PROBE_MAX_FRACTION = 0.75
WALL_PROBE_SAMPLES = 9
WALL_REQUIRED_SAMPLE_FRACTION = 0.60

# If the strict rule cannot reach the goal, open moves receive this multiplier
# during a fallback search.  The fallback remains strongly wall-biased while
# still returning a route when the camera map makes the hard rule impossible.
OPEN_MOVE_FALLBACK_MULTIPLIER = 6.0

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


EdgeKey = Tuple[int, int]
Adjacency = Dict[int, List[Tuple[int, float]]]


class CollisionGraph(dict):
    """Adjacency graph plus camera-derived side-wall information."""

    def __init__(self) -> None:
        super().__init__()
        self.wall_supported_edges: Set[EdgeKey] = set()
        self.directed_edge_headings: Dict[Tuple[int, int], int] = {}


def edge_key(first_id: int, second_id: int) -> EdgeKey:
    """Return one direction-independent key for a graph edge."""
    return (
        (first_id, second_id)
        if first_id < second_id
        else (second_id, first_id)
    )


def heading_between_nodes(first: GridNode, second: GridNode) -> int:
    """Return the cardinal-heading index for one directed graph edge."""
    row_change = second.row - first.row
    column_change = second.column - first.column

    if row_change == -1 and column_change == 0:
        return 0
    if row_change == 0 and column_change == 1:
        return 1
    if row_change == 1 and column_change == 0:
        return 2
    if row_change == 0 and column_change == -1:
        return 3

    raise ValueError("Graph headings require cardinal-neighbour nodes.")


def normalise_heading(initial_heading: Optional[str]) -> int:
    """Convert an optional heading name to its clockwise index."""
    if initial_heading is None:
        return -1

    heading = initial_heading.lower()
    if heading not in CARDINAL_HEADINGS:
        raise ValueError(
            "Initial heading must be one of: {}."
            .format(", ".join(CARDINAL_HEADINGS))
        )
    return CARDINAL_HEADINGS.index(heading)


def quarter_turn_count(first_heading: int, second_heading: int) -> int:
    """Return the fewest 90-degree turns between two headings."""
    if first_heading < 0 or second_heading < 0:
        return 0

    clockwise = (second_heading - first_heading) % 4
    return min(clockwise, 4 - clockwise)


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

    return segment_is_collision_free(
        planning_map,
        (first.x, first.y),
        (second.x, second.y),
        check_thickness,
    )


def _extrapolated_side_nodes(
    first: GridNode,
    second: GridNode,
    lookup: Mapping[Tuple[int, int], GridNode],
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Return reference lines one node-spacing to both sides of an edge."""
    if first.row == second.row:
        side_offsets = ((-1, 0), (1, 0))
    elif first.column == second.column:
        side_offsets = ((0, -1), (0, 1))
    else:
        raise ValueError("Wall probing requires cardinal-neighbour edges.")

    references: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

    for row_offset, column_offset in side_offsets:
        first_side = lookup.get(
            (first.row + row_offset, first.column + column_offset)
        )
        second_side = lookup.get(
            (second.row + row_offset, second.column + column_offset)
        )

        if first_side is not None and second_side is not None:
            references.append(
                (
                    (float(first_side.x), float(first_side.y)),
                    (float(second_side.x), float(second_side.y)),
                )
            )
            continue

        # At the outside row/column there is no node beyond the edge.  Mirror
        # the available inside row/column to estimate the same spacing.
        opposite_first = lookup.get(
            (first.row - row_offset, first.column - column_offset)
        )
        opposite_second = lookup.get(
            (second.row - row_offset, second.column - column_offset)
        )

        if opposite_first is not None and opposite_second is not None:
            references.append(
                (
                    (
                        2.0 * first.x - opposite_first.x,
                        2.0 * first.y - opposite_first.y,
                    ),
                    (
                        2.0 * second.x - opposite_second.x,
                        2.0 * second.y - opposite_second.y,
                    ),
                )
            )

    return references


def _side_reference_contains_wall(
    planning_map: np.ndarray,
    first: GridNode,
    second: GridNode,
    reference: Tuple[Tuple[float, float], Tuple[float, float]],
    sample_count: int,
    required_sample_fraction: float,
) -> bool:
    """Check whether a mostly continuous wall lies beside one graph edge."""
    if sample_count <= 0:
        raise ValueError("Wall probe sample count must be greater than zero.")
    if not 0.0 < required_sample_fraction <= 1.0:
        raise ValueError(
            "Required wall-sample fraction must be greater than zero and "
            "no more than one."
        )

    height, width = planning_map.shape[:2]
    first_reference, second_reference = reference
    supported_samples = 0

    # Avoid the edge endpoints, where a front/rear wall can look like a side
    # wall.  Each remaining sample casts a short ray toward the expected wall.
    along_fractions = np.linspace(0.15, 0.85, sample_count)
    probe_fractions = np.linspace(
        WALL_PROBE_MIN_FRACTION,
        WALL_PROBE_MAX_FRACTION,
        max(3, int(round(math.hypot(
            first_reference[0] - first.x,
            first_reference[1] - first.y,
        ) * (WALL_PROBE_MAX_FRACTION - WALL_PROBE_MIN_FRACTION))) + 1),
    )

    for along in along_fractions:
        centre_x = first.x + along * (second.x - first.x)
        centre_y = first.y + along * (second.y - first.y)
        side_x = first_reference[0] + along * (
            second_reference[0] - first_reference[0]
        )
        side_y = first_reference[1] + along * (
            second_reference[1] - first_reference[1]
        )

        sample_has_wall = False
        for outward in probe_fractions:
            x = int(round(centre_x + outward * (side_x - centre_x)))
            y = int(round(centre_y + outward * (side_y - centre_y)))

            # Outside the image/board is a wall for route-support purposes.
            if not (0 <= x < width and 0 <= y < height):
                sample_has_wall = True
                break
            if planning_map[y, x] == 0:
                sample_has_wall = True
                break

        if sample_has_wall:
            supported_samples += 1

    required_samples = int(math.ceil(
        sample_count * required_sample_fraction
    ))
    return supported_samples >= required_samples


def edge_has_side_wall(
    planning_map: np.ndarray,
    first: GridNode,
    second: GridNode,
    lookup: Mapping[Tuple[int, int], GridNode],
    sample_count: int = WALL_PROBE_SAMPLES,
    required_sample_fraction: float = WALL_REQUIRED_SAMPLE_FRACTION,
) -> bool:
    """Return whether the camera mask shows a wall along either side."""
    for reference in _extrapolated_side_nodes(first, second, lookup):
        if _side_reference_contains_wall(
            planning_map,
            first,
            second,
            reference,
            sample_count,
            required_sample_fraction,
        ):
            return True

    return False


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

    graph = CollisionGraph()
    graph.update({
        node.node_id: []
        for node in nodes
        if not node.blocked
    })

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

                outward_heading = heading_between_nodes(current, neighbour)
                graph.directed_edge_headings[
                    (current.node_id, neighbour.node_id)
                ] = outward_heading
                graph.directed_edge_headings[
                    (neighbour.node_id, current.node_id)
                ] = (outward_heading + 2) % 4

                if edge_has_side_wall(
                    planning_map,
                    current,
                    neighbour,
                    lookup,
                ):
                    graph.wall_supported_edges.add(
                        edge_key(current.node_id, neighbour.node_id)
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


def _reconstruct_state_path(
    previous: Mapping[
        Tuple[int, int, int], Optional[Tuple[int, int, int]]
    ],
    goal_state: Tuple[int, int, int],
) -> List[int]:
    """Convert a wall-aware state chain back to ordinary node IDs."""
    path: List[int] = []
    current: Optional[Tuple[int, int, int]] = goal_state

    while current is not None:
        path.append(current[0])
        current = previous[current]

    path.reverse()
    return path


def _strict_wall_guided_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
    wall_supported_edges: Set[EdgeKey],
    max_consecutive_open_moves: int,
    directed_edge_headings: Mapping[Tuple[int, int], int],
    initial_heading: Optional[str],
    turn_penalty_edge_fraction: float,
) -> Tuple[List[int], float]:
    """Find the shortest path satisfying the consecutive-open-move limit."""
    if max_consecutive_open_moves < 0:
        raise ValueError("Maximum consecutive open moves cannot be negative.")
    if turn_penalty_edge_fraction < 0.0:
        raise ValueError("Turn-penalty fraction cannot be negative.")
    if start_id not in graph or goal_id not in graph:
        return [], float("inf")

    start_state = (start_id, 0, normalise_heading(initial_heading))
    # The first tuple value is the search cost (physical distance plus the
    # tiny open-edge tie-break cost); the second is true physical distance.
    best_cost: Dict[Tuple[int, int, int], Tuple[float, float]] = {
        start_state: (0.0, 0.0)
    }
    previous: Dict[
        Tuple[int, int, int], Optional[Tuple[int, int, int]]
    ] = {start_state: None}
    queue: List[Tuple[float, float, int, int, int]] = [
        (0.0, 0.0, start_id, 0, start_state[2])
    ]

    while queue:
        (
            search_cost,
            current_distance,
            current_id,
            open_run,
            current_heading,
        ) = heapq.heappop(queue)
        current_state = (current_id, open_run, current_heading)

        if (search_cost, current_distance) != best_cost.get(current_state):
            continue
        if current_id == goal_id:
            return (
                _reconstruct_state_path(previous, current_state),
                current_distance,
            )

        for neighbour_id, edge_cost in graph.get(current_id, []):
            supported = (
                edge_key(current_id, neighbour_id) in wall_supported_edges
            )
            neighbour_open_run = 0 if supported else open_run + 1

            if neighbour_open_run > max_consecutive_open_moves:
                continue

            required_heading = directed_edge_headings.get(
                (current_id, neighbour_id),
                current_heading,
            )
            neighbour_state = (
                neighbour_id,
                neighbour_open_run,
                required_heading,
            )
            new_distance = current_distance + edge_cost
            turn_quarters = quarter_turn_count(
                current_heading,
                required_heading,
            )
            new_search_cost = (
                search_cost
                + edge_cost
                + (0.0 if supported else OPEN_MOVE_PREFERENCE_PENALTY_PIXELS)
                + turn_quarters * turn_penalty_edge_fraction * edge_cost
            )
            candidate = (new_search_cost, new_distance)

            if candidate >= best_cost.get(
                neighbour_state,
                (float("inf"), float("inf")),
            ):
                continue

            best_cost[neighbour_state] = candidate
            previous[neighbour_state] = current_state
            heapq.heappush(
                queue,
                (
                    new_search_cost,
                    new_distance,
                    neighbour_id,
                    neighbour_open_run,
                    required_heading,
                ),
            )

    return [], float("inf")


def _weighted_wall_guided_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
    wall_supported_edges: Set[EdgeKey],
    open_move_multiplier: float,
    directed_edge_headings: Mapping[Tuple[int, int], int],
    initial_heading: Optional[str],
    turn_penalty_edge_fraction: float,
) -> Tuple[List[int], float]:
    """Find a fallback path that strongly penalises unsupported moves."""
    if open_move_multiplier < 1.0:
        raise ValueError("Open-move multiplier must be at least one.")
    if turn_penalty_edge_fraction < 0.0:
        raise ValueError("Turn-penalty fraction cannot be negative.")
    if start_id not in graph or goal_id not in graph:
        return [], float("inf")

    start_state = (start_id, normalise_heading(initial_heading))
    best_cost: Dict[Tuple[int, int], Tuple[float, float]] = {
        start_state: (0.0, 0.0)
    }
    previous: Dict[
        Tuple[int, int], Optional[Tuple[int, int]]
    ] = {start_state: None}
    queue: List[Tuple[float, float, int, int]] = [
        (0.0, 0.0, start_id, start_state[1])
    ]
    goal_state: Optional[Tuple[int, int]] = None

    while queue:
        (
            weighted_cost,
            physical_distance,
            current_id,
            current_heading,
        ) = heapq.heappop(queue)
        current_state = (current_id, current_heading)
        if (weighted_cost, physical_distance) != best_cost.get(current_state):
            continue
        if current_id == goal_id:
            goal_state = current_state
            break

        for neighbour_id, edge_cost in graph.get(current_id, []):
            supported = (
                edge_key(current_id, neighbour_id) in wall_supported_edges
            )
            required_heading = directed_edge_headings.get(
                (current_id, neighbour_id),
                current_heading,
            )
            turn_quarters = quarter_turn_count(
                current_heading,
                required_heading,
            )
            multiplier = 1.0 if supported else open_move_multiplier
            candidate = (
                weighted_cost
                + edge_cost * multiplier
                + turn_quarters * turn_penalty_edge_fraction * edge_cost,
                physical_distance + edge_cost,
            )
            neighbour_state = (neighbour_id, required_heading)

            if candidate >= best_cost.get(
                neighbour_state,
                (float("inf"), float("inf")),
            ):
                continue

            best_cost[neighbour_state] = candidate
            previous[neighbour_state] = current_state
            heapq.heappush(
                queue,
                (
                    candidate[0],
                    candidate[1],
                    neighbour_id,
                    required_heading,
                ),
            )

    if goal_state is None:
        return [], float("inf")

    path: List[int] = []
    current: Optional[Tuple[int, int]] = goal_state
    while current is not None:
        path.append(current[0])
        current = previous[current]
    path.reverse()

    return path, best_cost[goal_state][1]


def wall_guided_dijkstra_shortest_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
    wall_supported_edges: Set[EdgeKey],
    max_consecutive_open_moves: int = MAX_CONSECUTIVE_OPEN_MOVES,
    fallback_to_wall_biased: bool = True,
    open_move_multiplier: float = OPEN_MOVE_FALLBACK_MULTIPLIER,
    directed_edge_headings: Optional[
        Mapping[Tuple[int, int], int]
    ] = None,
    initial_heading: Optional[str] = None,
    turn_penalty_edge_fraction: float = TURN_PENALTY_EDGE_FRACTION,
) -> Tuple[List[int], float, bool]:
    """Return a shortest wall-supported path and whether fallback was used."""
    path, distance = _strict_wall_guided_path(
        graph,
        start_id,
        goal_id,
        wall_supported_edges,
        max_consecutive_open_moves,
        directed_edge_headings or {},
        initial_heading,
        turn_penalty_edge_fraction,
    )
    if path or not fallback_to_wall_biased:
        return path, distance, False

    path, distance = _weighted_wall_guided_path(
        graph,
        start_id,
        goal_id,
        wall_supported_edges,
        open_move_multiplier,
        directed_edge_headings or {},
        initial_heading,
        turn_penalty_edge_fraction,
    )
    return path, distance, bool(path)


def _plain_dijkstra_shortest_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
) -> Tuple[List[int], float]:
    """Return the unconstrained minimum-cost path and pixel length."""
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


def dijkstra_shortest_path(
    graph: Adjacency,
    start_id: int,
    goal_id: int,
    max_consecutive_open_moves: int = MAX_CONSECUTIVE_OPEN_MOVES,
    fallback_to_wall_biased: bool = True,
    initial_heading: Optional[str] = None,
    turn_penalty_edge_fraction: float = TURN_PENALTY_EDGE_FRACTION,
) -> Tuple[List[int], float]:
    """Return a wall-guided path when graph wall metadata is available.

    ``build_collision_free_graph`` attaches the required side-wall metadata,
    so existing notebook calls automatically use the new behaviour.  Plain
    dictionaries retain the original unconstrained Dijkstra behaviour.
    """
    wall_supported_edges = getattr(graph, "wall_supported_edges", None)
    if wall_supported_edges is None:
        return _plain_dijkstra_shortest_path(graph, start_id, goal_id)

    path, distance, used_fallback = wall_guided_dijkstra_shortest_path(
        graph,
        start_id,
        goal_id,
        wall_supported_edges,
        max_consecutive_open_moves,
        fallback_to_wall_biased,
        directed_edge_headings=getattr(
            graph,
            "directed_edge_headings",
            {},
        ),
        initial_heading=initial_heading,
        turn_penalty_edge_fraction=turn_penalty_edge_fraction,
    )

    if used_fallback:
        warnings.warn(
            "No path satisfies the maximum of {} consecutive open moves; "
            "using the most wall-biased reachable path instead."
            .format(max_consecutive_open_moves),
            RuntimeWarning,
        )

    return path, distance


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
        "--max-open-moves",
        type=int,
        default=MAX_CONSECUTIVE_OPEN_MOVES,
        help=(
            "Maximum consecutive moves with no detected side wall. "
            "Default: {}.".format(MAX_CONSECUTIVE_OPEN_MOVES)
        ),
    )

    parser.add_argument(
        "--heading",
        choices=CARDINAL_HEADINGS,
        default="up",
        help="Initial robot orientation relative to the image. Default: up.",
    )

    parser.add_argument(
        "--turn-penalty",
        type=float,
        default=TURN_PENALTY_EDGE_FRACTION,
        help=(
            "Cost of each 90-degree turn as a fraction of one move. "
            "Default: {:.2f}.".format(TURN_PENALTY_EDGE_FRACTION)
        ),
    )

    parser.add_argument(
        "--strict-wall-rule",
        action="store_true",
        help=(
            "Return no path instead of using a wall-biased fallback when "
            "the consecutive-open-move rule cannot be satisfied."
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

    if args.max_open_moves < 0:
        raise ValueError("--max-open-moves must be zero or greater.")

    if args.turn_penalty < 0.0:
        raise ValueError("--turn-penalty must be zero or greater.")

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
        max_consecutive_open_moves=args.max_open_moves,
        fallback_to_wall_biased=not args.strict_wall_rule,
        initial_heading=args.heading,
        turn_penalty_edge_fraction=args.turn_penalty,
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
    print(
        "Wall guidance: maximum {} consecutive open moves"
        .format(args.max_open_moves)
    )
    print(
        "Turn penalty: {:.2f} moves per 90 degrees"
        .format(args.turn_penalty)
    )
    print(
        "Wall-supported graph edges: {}"
        .format(len(graph.wall_supported_edges))
    )
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
