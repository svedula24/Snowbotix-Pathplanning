import os
import sys
import time
import math
import unittest
import copy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from map import Map, generate_test_map
from d_star_lite import DStarLite
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


def fresh_planner() -> tuple[DStarLite, Map]:
    """Return a fresh (planner, map) pair to avoid state bleed between tests."""
    m = get_test_map()
    return DStarLite(m), m


class TestDStarLite(unittest.TestCase):

    def test_finds_initial_path(self):
        """D* Lite finds a non-empty path on the standard test map."""
        planner, m = fresh_planner()
        path = planner.plan((5, 5), (90, 90))
        self.assertGreater(len(path), 1)
        self.assertEqual(path[0], (5, 5))
        self.assertEqual(path[-1], (90, 90))

    def test_replans_when_obstacle_on_path(self):
        """
        Injecting an obstacle on the current path triggers a valid replan
        that avoids the new obstacle.
        """
        planner, m = fresh_planner()
        original_path = planner.plan((5, 5), (90, 90))

        # Pick a mid-path cell (not start or goal)
        mid_idx = len(original_path) // 2
        obstacle = original_path[mid_idx]

        planner.update_obstacle(*obstacle)
        new_path = planner._path

        self.assertGreater(len(new_path), 1)
        self.assertEqual(new_path[0], (5, 5))
        self.assertEqual(new_path[-1], (90, 90))
        self.assertNotIn(obstacle, new_path,
                         "Replanned path must not pass through the injected obstacle.")

    def test_no_replan_when_obstacle_off_path(self):
        """
        Injecting an obstacle that is NOT on the current path should leave
        the path unchanged.
        """
        planner, m = fresh_planner()
        original_path = planner.plan((5, 5), (90, 90))
        path_set = set(original_path)

        # Find a free cell not on the path
        off_path_cell = None
        for x in range(m.width):
            for y in range(m.height):
                if m.is_free(x, y) and (x, y) not in path_set:
                    off_path_cell = (x, y)
                    break
            if off_path_cell:
                break

        self.assertIsNotNone(off_path_cell, "Could not find a free cell off the path.")
        planner.update_obstacle(*off_path_cell)
        new_path = planner._path

        self.assertEqual(original_path, new_path,
                         "Path should be unchanged when obstacle is not on path.")

    def test_replan_under_500ms(self):
        """update_obstacle + replan must complete in under 500ms."""
        planner, m = fresh_planner()
        original_path = planner.plan((5, 5), (90, 90))
        mid = original_path[len(original_path) // 2]

        t0 = time.time()
        planner.update_obstacle(*mid)
        elapsed = (time.time() - t0) * 1000  # ms
        self.assertLess(elapsed, 500.0,
                        f"Replan took {elapsed:.1f}ms — exceeds 500ms budget.")

    def test_path_collision_free_after_replan(self):
        """Every waypoint in the replanned path must be in free space."""
        planner, m = fresh_planner()
        original_path = planner.plan((5, 5), (90, 90))
        mid = original_path[len(original_path) // 2]
        planner.update_obstacle(*mid)
        new_path = planner._path

        for x, y in new_path:
            self.assertTrue(m.is_free(x, y),
                            f"Waypoint ({x},{y}) is occupied in replanned path.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true", help="Show before/after replan visualization")
    args, remaining = parser.parse_known_args()

    if args.visualize:
        m = get_test_map()
        planner = DStarLite(m)
        start, goal = (5, 5), (90, 90)
        original_path = planner.plan(start, goal)
        mid = original_path[len(original_path) // 2]
        print(f"Injecting obstacle at {mid}")
        planner.update_obstacle(*mid)
        new_path = planner._path
        print(f"Original path: {len(original_path)} waypoints")
        print(f"Replanned path: {len(new_path)} waypoints")
        planner.visualize(original_path=original_path, new_path=new_path, obstacle=mid)
    else:
        unittest.main(argv=[sys.argv[0]] + remaining)
