from abc import ABC, abstractmethod


class PlanningFailure(Exception):
    pass


class Planner(ABC):
    @abstractmethod
    def plan(self, start: tuple, goal: tuple) -> list:
        """Compute a path from start to goal. Returns list of (x, y) tuples."""
        ...

    @abstractmethod
    def update_obstacle(self, x: int, y: int) -> None:
        """Mark cell (x, y) as occupied and update internal state."""
        ...
