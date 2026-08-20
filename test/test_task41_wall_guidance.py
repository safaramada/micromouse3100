import math
from pathlib import Path
import sys
import unittest
import warnings

import cv2
import numpy as np


MAZE_CODE = (
    Path(__file__).resolve().parents[1]
    / "computer_vision_code"
    / "mazemapping"
)
sys.path.insert(0, str(MAZE_CODE))

from path_planning_nodes import GridNode  # noqa: E402
from shortest_path_dijkstra import (  # noqa: E402
    CollisionGraph,
    build_collision_free_graph,
    dijkstra_shortest_path,
    edge_key,
    wall_guided_dijkstra_shortest_path,
)


def add_undirected_edge(graph, first, second, cost=1.0):
    graph.setdefault(first, []).append((second, cost))
    graph.setdefault(second, []).append((first, cost))


class WallGuidedDijkstraTests(unittest.TestCase):
    def test_equal_length_route_prefers_fewer_turns(self):
        graph = CollisionGraph()

        add_undirected_edge(graph, 0, 1)
        add_undirected_edge(graph, 1, 3)
        add_undirected_edge(graph, 0, 2)
        add_undirected_edge(graph, 2, 3)
        graph.wall_supported_edges.update({
            edge_key(0, 1),
            edge_key(1, 3),
            edge_key(0, 2),
            edge_key(2, 3),
        })
        graph.directed_edge_headings.update({
            (0, 1): 0,
            (1, 0): 2,
            (1, 3): 1,
            (3, 1): 3,
            (0, 2): 1,
            (2, 0): 3,
            (2, 3): 1,
            (3, 2): 3,
        })

        path, distance = dijkstra_shortest_path(
            graph,
            0,
            3,
            initial_heading="right",
        )

        self.assertEqual(path, [0, 2, 3])
        self.assertEqual(distance, 2.0)

    def test_equal_length_route_prefers_more_wall_supported_moves(self):
        graph = CollisionGraph()

        # Both routes have length 2.  The route through node 2 has a supported
        # first edge, so it should win the wall-support tie-break.
        add_undirected_edge(graph, 0, 1)
        add_undirected_edge(graph, 1, 3)
        add_undirected_edge(graph, 0, 2)
        add_undirected_edge(graph, 2, 3)
        graph.wall_supported_edges.add(edge_key(0, 2))

        path, distance = dijkstra_shortest_path(graph, 0, 3)

        self.assertEqual(path, [0, 2, 3])
        self.assertEqual(distance, 2.0)

    def test_ignores_tiny_perspective_shortcut_without_walls(self):
        graph = CollisionGraph()

        # The open route is one image pixel shorter overall, but both routes
        # represent the same number of logical grid moves.
        add_undirected_edge(graph, 0, 1, 50.0)
        add_undirected_edge(graph, 1, 3, 50.0)
        add_undirected_edge(graph, 0, 2, 50.5)
        add_undirected_edge(graph, 2, 3, 50.5)
        graph.wall_supported_edges.update({
            edge_key(0, 2),
            edge_key(2, 3),
        })

        path, distance = dijkstra_shortest_path(graph, 0, 3)

        self.assertEqual(path, [0, 2, 3])
        self.assertEqual(distance, 101.0)

    def test_rejects_third_open_move_and_uses_supported_route(self):
        graph = CollisionGraph()

        # The ordinary shortest path is 0-1-2-3, but all three moves are open.
        add_undirected_edge(graph, 0, 1)
        add_undirected_edge(graph, 1, 2)
        add_undirected_edge(graph, 2, 3)

        # This route is one move longer, with walls that reset the open count.
        add_undirected_edge(graph, 0, 4)
        add_undirected_edge(graph, 4, 5)
        add_undirected_edge(graph, 5, 6)
        add_undirected_edge(graph, 6, 3)
        graph.wall_supported_edges.update({
            edge_key(4, 5),
            edge_key(6, 3),
        })

        path, distance = dijkstra_shortest_path(graph, 0, 3)

        self.assertEqual(path, [0, 4, 5, 6, 3])
        self.assertEqual(distance, 4.0)

    def test_reports_when_wall_biased_fallback_is_required(self):
        graph = CollisionGraph()
        add_undirected_edge(graph, 0, 1)
        add_undirected_edge(graph, 1, 2)
        add_undirected_edge(graph, 2, 3)

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            path, distance = dijkstra_shortest_path(graph, 0, 3)

        self.assertEqual(path, [0, 1, 2, 3])
        self.assertEqual(distance, 3.0)
        self.assertEqual(len(caught), 1)
        self.assertIn("most wall-biased", str(caught[0].message))

    def test_strict_mode_returns_no_path_instead_of_falling_back(self):
        graph = CollisionGraph()
        add_undirected_edge(graph, 0, 1)
        add_undirected_edge(graph, 1, 2)
        add_undirected_edge(graph, 2, 3)

        path, distance, used_fallback = wall_guided_dijkstra_shortest_path(
            graph,
            0,
            3,
            graph.wall_supported_edges,
            max_consecutive_open_moves=2,
            fallback_to_wall_biased=False,
        )

        self.assertEqual(path, [])
        self.assertTrue(math.isinf(distance))
        self.assertFalse(used_fallback)


class SideWallDetectionTests(unittest.TestCase):
    @staticmethod
    def nodes():
        nodes = []
        node_id = 0
        for row, y in enumerate((10, 30, 50)):
            for column, x in enumerate((10, 30, 50)):
                nodes.append(GridNode(node_id, row, column, x, y, False))
                node_id += 1
        return nodes

    def test_parallel_wall_marks_edge_as_supported(self):
        planning_map = np.full((61, 61), 255, dtype=np.uint8)
        cv2.line(planning_map, (10, 20), (30, 20), 0, thickness=2)

        graph = build_collision_free_graph(
            self.nodes(),
            planning_map,
            rows=3,
            columns=3,
            edge_check_thickness=1,
        )

        self.assertIn(edge_key(3, 4), graph.wall_supported_edges)

    def test_open_corridor_does_not_mark_edge_as_supported(self):
        planning_map = np.full((61, 61), 255, dtype=np.uint8)

        graph = build_collision_free_graph(
            self.nodes(),
            planning_map,
            rows=3,
            columns=3,
            edge_check_thickness=1,
        )

        self.assertNotIn(edge_key(3, 4), graph.wall_supported_edges)


if __name__ == "__main__":
    unittest.main()
