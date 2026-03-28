import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from map import Map
from planner import Planner, PlanningFailure


class Node:
    __slots__ = ("x", "y", "parent", "cost")

    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


class RRTStar(Planner):
    def __init__(
        self,
        map_obj: Map,
        step_size: float = 5.0,
        goal_bias: float = 0.1,
        neighbor_radius: float = 15.0,
        max_iterations: int = 5000,
    ):
        self.map = map_obj
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.neighbor_radius = neighbor_radius
        self.max_iterations = max_iterations

        self._tree: list[Node] = []
        self._start = None
        self._goal = None
        self._path: list[tuple] = []

    # ------------------------------------------------------------------
    # Planner interface
    # ------------------------------------------------------------------

    def plan(self, start: tuple, goal: tuple) -> list:
        self._start = start
        self._goal = goal
        self._tree = []
        self._path = []

        if not self.map.is_free(*start):
            raise PlanningFailure("Start position is occupied.")
        if not self.map.is_free(*goal):
            raise PlanningFailure("Goal position is occupied.")

        root = Node(*start, parent=None, cost=0.0)
        self._tree.append(root)

        best_goal_node = None

        for _ in range(self.max_iterations):
            sample = self._sample()
            nearest = self._nearest(sample)
            new_node = self._extend(nearest, sample)
            if new_node is None:
                continue

            # Choose best parent among neighbors
            neighbors = self._neighbors(new_node)
            new_node = self._choose_parent(new_node, neighbors)

            self._tree.append(new_node)

            # Rewire neighbors through new_node if cheaper
            self._rewire(new_node, neighbors)

            # Check if we reached the goal
            if self._dist(new_node, Node(*goal)) <= self.step_size:
                if self.map.is_free(*goal) and self._collision_free(new_node.x, new_node.y, *goal):
                    goal_cost = new_node.cost + self._dist(new_node, Node(*goal))
                    if best_goal_node is None or goal_cost < best_goal_node.cost:
                        best_goal_node = Node(*goal, parent=new_node, cost=goal_cost)

        if best_goal_node is None:
            raise PlanningFailure(
                f"No path found within {self.max_iterations} iterations."
            )

        self._path = self._trace_path(best_goal_node)
        return self._path

    def update_obstacle(self, x: int, y: int) -> None:
        self.map.set_obstacle(x, y)

    # ------------------------------------------------------------------
    # Core RRT* helpers
    # ------------------------------------------------------------------

    def _sample(self) -> tuple:
        if random.random() < self.goal_bias:
            return self._goal
        return (
            random.randint(0, self.map.width - 1),
            random.randint(0, self.map.height - 1),
        )

    def _nearest(self, point: tuple) -> Node:
        px, py = point
        return min(self._tree, key=lambda n: (n.x - px) ** 2 + (n.y - py) ** 2)

    def _extend(self, nearest: Node, sample: tuple) -> Node | None:
        dx = sample[0] - nearest.x
        dy = sample[1] - nearest.y
        dist = math.hypot(dx, dy)
        if dist == 0:
            return None
        ratio = min(self.step_size / dist, 1.0)
        nx = int(round(nearest.x + dx * ratio))
        ny = int(round(nearest.y + dy * ratio))
        if not self.map.is_free(nx, ny):
            return None
        if not self._collision_free(nearest.x, nearest.y, nx, ny):
            return None
        cost = nearest.cost + math.hypot(nx - nearest.x, ny - nearest.y)
        return Node(nx, ny, parent=nearest, cost=cost)

    def _neighbors(self, node: Node) -> list[Node]:
        r = self._adaptive_radius()
        return [
            n for n in self._tree
            if math.hypot(n.x - node.x, n.y - node.y) <= r
        ]

    def _adaptive_radius(self) -> float:
        n = max(len(self._tree), 1)
        gamma = 20.0
        r = gamma * math.sqrt(math.log(n) / n)
        return min(r, self.neighbor_radius)

    def _choose_parent(self, node: Node, neighbors: list[Node]) -> Node:
        best_parent = node.parent
        best_cost = node.cost
        for nb in neighbors:
            if nb is node.parent:
                continue
            if not self._collision_free(nb.x, nb.y, node.x, node.y):
                continue
            cost = nb.cost + math.hypot(nb.x - node.x, nb.y - node.y)
            if cost < best_cost:
                best_cost = cost
                best_parent = nb
        node.parent = best_parent
        node.cost = best_cost
        return node

    def _rewire(self, new_node: Node, neighbors: list[Node]) -> None:
        for nb in neighbors:
            if nb is new_node.parent:
                continue
            if not self._collision_free(new_node.x, new_node.y, nb.x, nb.y):
                continue
            new_cost = new_node.cost + math.hypot(new_node.x - nb.x, new_node.y - nb.y)
            if new_cost < nb.cost:
                nb.parent = new_node
                nb.cost = new_cost

    def _collision_free(self, x0, y0, x1, y1) -> bool:
        """Bresenham line check — returns True if segment is fully in free space."""
        points = self._bresenham(int(x0), int(y0), int(x1), int(y1))
        return all(self.map.is_free(px, py) for px, py in points)

    @staticmethod
    def _bresenham(x0, y0, x1, y1) -> list[tuple]:
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    @staticmethod
    def _dist(a: Node, b: Node) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def _trace_path(node: Node) -> list[tuple]:
        path = []
        cur = node
        while cur is not None:
            path.append((cur.x, cur.y))
            cur = cur.parent
        return list(reversed(path))

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def visualize(self, title="RRT* Path Planning"):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.map.to_image(), cmap="gray_r", origin="upper",
                  extent=[0, self.map.width, self.map.height, 0])

        # Draw tree edges
        for node in self._tree:
            if node.parent is not None:
                ax.plot(
                    [node.parent.x, node.x],
                    [node.parent.y, node.y],
                    color="steelblue", linewidth=0.4, alpha=0.6,
                )

        # Draw path
        if self._path:
            xs, ys = zip(*self._path)
            ax.plot(xs, ys, color="yellow", linewidth=2.5, zorder=5, label="Path")

        if self._start:
            ax.plot(*self._start, "ro", markersize=8, zorder=6, label="Start")
        if self._goal:
            ax.plot(*self._goal, "g*", markersize=12, zorder=6, label="Goal")

        ax.set_title(title)
        ax.legend(loc="upper left")
        ax.set_xlim(0, self.map.width)
        ax.set_ylim(self.map.height, 0)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(here, "maps", "test_map.png")

    if not os.path.exists(map_path):
        from map import generate_test_map
        m = generate_test_map(map_path)
    else:
        m = Map.from_png(map_path)

    planner = RRTStar(m, step_size=5, goal_bias=0.1, neighbor_radius=15, max_iterations=5000)
    path = planner.plan((5, 5), (90, 90))
    print(f"Path found with {len(path)} waypoints.")
    planner.visualize()
