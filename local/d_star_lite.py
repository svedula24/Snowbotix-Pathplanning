import math
import heapq
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from map import Map
from planner import Planner, PlanningFailure

INF = float("inf")


class DStarLite(Planner):
    """
    D* Lite path planner.
    Plans backwards from goal → start. Path extraction follows g-value
    gradient from start → goal.
    """

    def __init__(self, map_obj: Map):
        self.map = map_obj
        self._start = None
        self._goal = None
        self._g = defaultdict(lambda: INF)
        self._rhs = defaultdict(lambda: INF)
        self._open: list = []          # heapq of (k1, k2, node)
        self._open_set: set = set()    # fast membership check
        self._km = 0.0
        self._path: list[tuple] = []

    # ------------------------------------------------------------------
    # Planner interface
    # ------------------------------------------------------------------

    def plan(self, start: tuple, goal: tuple) -> list:
        self._start = start
        self._goal = goal
        self._g = defaultdict(lambda: INF)
        self._rhs = defaultdict(lambda: INF)
        self._open = []
        self._open_set = set()
        self._km = 0.0

        if not self.map.is_free(*start):
            raise PlanningFailure("Start position is occupied.")
        if not self.map.is_free(*goal):
            raise PlanningFailure("Goal position is occupied.")

        self._rhs[goal] = 0.0
        self._push(goal, self._key(goal))
        self._compute_shortest_path(full=True)
        self._path = self._extract_path()
        return self._path

    def update_obstacle(self, x: int, y: int) -> None:
        """Mark (x,y) occupied, replan."""
        self.map.set_obstacle(x, y)
        # Update the affected cell and all its neighbors
        affected = [(x, y)] + self._all_neighbors(x, y)
        for cell in affected:
            self._update_vertex(cell)
        self._compute_shortest_path()
        self._path = self._extract_path()

    # ------------------------------------------------------------------
    # D* Lite internals
    # ------------------------------------------------------------------

    def _key(self, u: tuple) -> tuple:
        g_u = self._g[u]
        rhs_u = self._rhs[u]
        min_val = min(g_u, rhs_u)
        k1 = min_val + self._heuristic(u, self._start) + self._km
        k2 = min_val
        return (k1, k2)

    def _heuristic(self, a: tuple, b: tuple) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _push(self, u: tuple, key: tuple):
        heapq.heappush(self._open, (key[0], key[1], u))
        self._open_set.add(u)

    def _top_key(self) -> tuple:
        while self._open:
            k1, k2, u = self._open[0]
            if u not in self._open_set:
                heapq.heappop(self._open)
                continue
            return (k1, k2)
        return (INF, INF)

    def _pop(self):
        while self._open:
            k1, k2, u = heapq.heappop(self._open)
            if u in self._open_set:
                self._open_set.discard(u)
                return (k1, k2), u
        return (INF, INF), None

    def _remove(self, u: tuple):
        self._open_set.discard(u)

    def _edge_cost(self, u: tuple, v: tuple) -> float:
        """Cost of moving from u to v (diagonal = sqrt(2), cardinal = 1)."""
        if not self.map.is_free(*u) or not self.map.is_free(*v):
            return INF
        dx = abs(u[0] - v[0])
        dy = abs(u[1] - v[1])
        return math.sqrt(2) if dx == 1 and dy == 1 else 1.0

    def _all_neighbors(self, x: int, y: int) -> list[tuple]:
        """8-connected neighbors regardless of occupancy (for cost updates)."""
        result = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map.width and 0 <= ny < self.map.height:
                    result.append((nx, ny))
        return result

    def _update_vertex(self, u: tuple) -> None:
        if u != self._goal:
            # rhs = min cost over successors (neighbors) — planning backwards,
            # so successors in the reverse graph are the same neighbors
            neighbors = self._all_neighbors(*u)
            if not self.map.is_free(*u):
                self._rhs[u] = INF
            else:
                self._rhs[u] = min(
                    (self._edge_cost(u, s) + self._g[s] for s in neighbors),
                    default=INF,
                )
        self._remove(u)
        if self._g[u] != self._rhs[u]:
            self._push(u, self._key(u))

    def _compute_shortest_path(self, full=False):
        while True:
            top_key = self._top_key()
            if top_key == (INF, INF):
                break
            if not full:
                start_key = self._key(self._start)
                if top_key >= start_key and self._rhs[self._start] == self._g[self._start]:
                    break
            k_old, u = self._pop()
            if u is None:
                break
            k_new = self._key(u)
            if k_old < k_new:
                # Key outdated — reinsert with current key
                self._push(u, k_new)
            elif self._g[u] > self._rhs[u]:
                # Overconsistent → make consistent
                self._g[u] = self._rhs[u]
                for nb in self._all_neighbors(*u):
                    self._update_vertex(nb)
            else:
                # Underconsistent → raise g, update neighbors
                self._g[u] = INF
                self._update_vertex(u)
                for nb in self._all_neighbors(*u):
                    self._update_vertex(nb)

    def _extract_path(self) -> list[tuple]:
        if self._g[self._start] == INF:
            raise PlanningFailure("No path exists from start to goal.")

        path = [self._start]
        current = self._start

        for _ in range(self.map.width * self.map.height):
            if current == self._goal:
                break
            neighbors = self._all_neighbors(*current)
            best = None
            best_cost = INF
            for nb in neighbors:
                cost = self._edge_cost(current, nb) + self._g[nb]
                if cost < best_cost:
                    best_cost = cost
                    best = nb
            # g-values must strictly decrease toward goal — if not, we're stuck
            if best is None or self._g[best] >= self._g[current] - 1e-9:
                raise PlanningFailure("Path extraction failed — no forward progress.")
            path.append(best)
            current = best

        if current != self._goal:
            raise PlanningFailure("Path extraction did not reach goal.")
        return path

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def visualize(self, original_path=None, new_path=None, obstacle=None, title="D* Lite Replanning"):
        fig, axes = plt.subplots(1, 2 if new_path else 1, figsize=(14 if new_path else 7, 7))
        if new_path is None:
            axes = [axes]

        def draw_on(ax, path, extra_obstacle=None, subtitle=""):
            ax.imshow(self.map.to_image(), cmap="gray_r", origin="upper",
                      extent=[0, self.map.width, self.map.height, 0])
            if path:
                xs, ys = zip(*path)
                ax.plot(xs, ys, color="dodgerblue", linewidth=2.5, zorder=5, label="Path")
            if extra_obstacle:
                ox, oy = extra_obstacle
                ax.plot(ox, oy, "rx", markersize=12, markeredgewidth=2.5, zorder=7, label="Obstacle")
            if self._start:
                ax.plot(*self._start, "ro", markersize=8, zorder=6, label="Start")
            if self._goal:
                ax.plot(*self._goal, "g*", markersize=12, zorder=6, label="Goal")
            ax.set_title(subtitle)
            ax.legend(loc="upper left", fontsize=8)
            ax.set_xlim(0, self.map.width)
            ax.set_ylim(self.map.height, 0)

        draw_on(axes[0], original_path, subtitle="Original Path")
        if new_path:
            draw_on(axes[1], new_path, extra_obstacle=obstacle, subtitle="Replanned Path")

        fig.suptitle(title)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(here, "maps", "hard_map.png")

    if not os.path.exists(map_path):
        from map import generate_test_map
        m = generate_test_map(map_path)
    else:
        m = Map.from_png(map_path)

    planner = DStarLite(m)
    start, goal = (5, 5), (90, 90)
    original_path = planner.plan(start, goal)
    print(f"Initial path: {len(original_path)} waypoints")

    # Inject an obstacle mid-path
    mid = original_path[len(original_path) // 2]
    print(f"Injecting obstacle at {mid}")
    planner.update_obstacle(*mid)
    new_path = planner._path
    print(f"Replanned path: {len(new_path)} waypoints")

    planner.visualize(original_path=original_path, new_path=new_path, obstacle=mid)
