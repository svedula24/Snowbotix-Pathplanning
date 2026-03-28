import os
import sys
import time
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from map import Map, generate_test_map
from rrt_star import RRTStar
from planner import PlanningFailure

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_PATH = os.path.join(HERE, "maps", "test_map.png")


def get_test_map() -> Map:
    if not os.path.exists(MAP_PATH):
        return generate_test_map(MAP_PATH)
    return Map.from_png(MAP_PATH)


def path_length(path: list) -> float:
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += math.hypot(dx, dy)
    return total


class TestRRTStar(unittest.TestCase):

    def test_finds_path_sparse_map(self):
        """RRT* finds a non-empty collision-free path on a 100x100 map with ~20% obstacles."""
        m = get_test_map()
        planner = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=5000)
        path = planner.plan((5, 5), (90, 90))
        self.assertGreater(len(path), 1)
        self.assertEqual(path[0], (5, 5))
        self.assertEqual(path[-1], (90, 90))

    def test_path_improves_with_iterations(self):
        """Path length should decrease (or stay equal) with more iterations — the * property."""
        m = get_test_map()

        planner_low = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=300)
        planner_high = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=5000)

        # Use fixed seed for reproducibility
        import random
        random.seed(0)
        try:
            path_low = planner_low.plan((5, 5), (90, 90))
            len_low = path_length(path_low)
        except PlanningFailure:
            len_low = float("inf")

        random.seed(0)
        path_high = planner_high.plan((5, 5), (90, 90))
        len_high = path_length(path_high)

        # Higher iterations should not be worse
        self.assertLessEqual(len_high, len_low + 1e-6,
                             "More iterations should produce an equal or shorter path.")

    def test_raises_on_impossible_map(self):
        """PlanningFailure is raised when the map has no viable path."""
        # Fully blocked row separating start from goal
        m = Map(20, 20)
        for x in range(20):
            m.set_obstacle(x, 10)

        planner = RRTStar(m, step_size=2, goal_bias=0.1, neighbor_radius=5, max_iterations=500)
        with self.assertRaises(PlanningFailure):
            planner.plan((5, 2), (5, 18))

    def test_path_collision_free(self):
        """Every waypoint in the returned path must be in free space."""
        m = get_test_map()
        planner = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=3000)
        path = planner.plan((5, 5), (90, 90))
        for x, y in path:
            self.assertTrue(m.is_free(x, y),
                            f"Waypoint ({x},{y}) is occupied — path is not collision-free.")

    def test_runs_under_10s(self):
        """RRT* with 5000 iterations must complete in under 10 seconds on a 100x100 grid."""
        m = get_test_map()
        planner = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=5000)
        t0 = time.time()
        planner.plan((5, 5), (90, 90))
        elapsed = time.time() - t0
        self.assertLess(elapsed, 10.0, f"Planning took {elapsed:.2f}s — exceeds 10s budget.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true", help="Show matplotlib visualization after tests")
    args, remaining = parser.parse_known_args()

    if args.visualize:
        m = get_test_map()
        planner = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=5000)
        path = planner.plan((5, 5), (90, 90))
        print(f"Path: {len(path)} waypoints, length={path_length(path):.1f}")
        planner.visualize()
    else:
        unittest.main(argv=[sys.argv[0]] + remaining)
