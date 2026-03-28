# Snowbotix Path Planning

Path planning system for an autonomous snow removal robot. Implements RRT* for initial global path planning and D* Lite for real-time replanning when obstacles appear.

## Structure

```
local/          # Standalone Python implementations
├── map.py      # 2D occupancy grid
├── planner.py  # Abstract base class
├── rrt_star.py # RRT* planner
├── d_star_lite.py  # D* Lite dynamic replanner
├── test_rrt.py
└── test_dstar.py
```

## Setup

```bash
python3 -m venv snow-pplan
source snow-pplan/bin/activate
pip install numpy matplotlib Pillow
```

## Usage

```bash
cd local

# Generate test maps
python map.py

# Run RRT*
python rrt_star.py

# Run D* Lite
python d_star_lite.py

# Run tests
python test_rrt.py
python test_dstar.py
```
