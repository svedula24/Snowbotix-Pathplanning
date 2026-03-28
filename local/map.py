import numpy as np
from PIL import Image
import os


class Map:
    """2D occupancy grid. 0 = free, 1 = occupied, -1 = unknown."""

    def __init__(self, width: int, height: int, grid=None):
        self.width = width
        self.height = height
        if grid is not None:
            self.grid = np.array(grid, dtype=np.int8)
        else:
            self.grid = np.zeros((height, width), dtype=np.int8)

    def is_free(self, x: int, y: int) -> bool:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return self.grid[y, x] == 0

    def set_obstacle(self, x: int, y: int) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y, x] = 1

    def get_neighbors(self, x: int, y: int) -> list:
        """Return 8-connected free neighbors."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_free(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors

    def to_image(self) -> np.ndarray:
        """Return a float array suitable for matplotlib imshow (0=white/free, 1=black/occupied)."""
        img = np.zeros((self.height, self.width), dtype=np.float32)
        img[self.grid == 1] = 1.0
        img[self.grid == -1] = 0.5
        return img

    @classmethod
    def from_png(cls, path: str) -> "Map":
        """Load a PNG. White pixels = free, dark pixels = occupied."""
        img = Image.open(path).convert("L")
        arr = np.array(img)
        height, width = arr.shape
        grid = np.zeros((height, width), dtype=np.int8)
        grid[arr < 128] = 1  # dark = occupied
        return cls(width, height, grid)


def generate_test_map(path: str, width=100, height=100, obstacle_density=0.20, seed=42):
    """
    Generate a 100x100 test map with ~20% random obstacles and a guaranteed
    clear diagonal corridor from (5,5) to (90,90).
    """
    rng = np.random.default_rng(seed)
    grid = np.zeros((height, width), dtype=np.int8)

    # Random obstacles
    mask = rng.random((height, width)) < obstacle_density
    grid[mask] = 1

    # Clear a corridor along the diagonal (5,5) -> (90,90)
    steps = 120
    for i in range(steps + 1):
        t = i / steps
        cx = int(5 + t * 85)
        cy = int(5 + t * 85)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    grid[ny, nx] = 0

    # Clear start and goal zones
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            for sx, sy in [(5, 5), (90, 90)]:
                nx, ny = sx + dx, sy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    grid[ny, nx] = 0

    # Convert to image (white=free, black=occupied)
    img_arr = np.ones((height, width), dtype=np.uint8) * 255
    img_arr[grid == 1] = 0
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(img_arr).save(path)
    print(f"Test map saved to {path}")
    return Map(width, height, grid)


def generate_hard_map(path: str, width=100, height=100, seed=42):
    """
    Generate a harder 100x100 map with:
    - 35% random obstacles
    - Several dense wall clusters
    - No guaranteed corridor; only start and goal zones are cleared
    """
    rng = np.random.default_rng(seed)
    grid = np.zeros((height, width), dtype=np.int8)

    # Random obstacles at higher density
    mask = rng.random((height, width)) < 0.35
    grid[mask] = 1

    # Dense wall clusters (horizontal and vertical barriers with narrow gaps)
    walls = [
        ((20, 10), (20, 75)),   # vertical wall, gap at bottom
        ((50, 25), (50, 90)),   # vertical wall, gap at top
        ((75, 10), (75, 65)),   # vertical wall, gap at bottom
        ((10, 40), (85, 40)),   # horizontal wall, gap on right
        ((15, 65), (70, 65)),   # horizontal wall, gap on right
    ]
    for (x0, y0), (x1, y1) in walls:
        for x in range(min(x0, x1), max(x0, x1) + 1):
            for y in range(min(y0, y1), max(y0, y1) + 1):
                if 0 <= x < width and 0 <= y < height:
                    grid[y, x] = 1

    # Clear start and goal zones only
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            for sx, sy in [(5, 5), (90, 90)]:
                nx, ny = sx + dx, sy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    grid[ny, nx] = 0

    img_arr = np.ones((height, width), dtype=np.uint8) * 255
    img_arr[grid == 1] = 0
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(img_arr).save(path)
    print(f"Hard map saved to {path}")
    return Map(width, height, grid)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    generate_test_map(os.path.join(here, "maps", "test_map.png"))
    generate_hard_map(os.path.join(here, "maps", "hard_map.png"))
