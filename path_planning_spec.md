# Feature Spec: Path Planning & Trajectory Optimization
## Autonomous Snow Removal Robot — RRT* + D* Lite

**Status:** Draft
**Author:** Saumit
**Date:** 2026-03-28
**Stack:** Python / C++, ROS2 Humble, Gazebo
**Reference:** [arXiv 2510.12169v2](https://arxiv.org/html/2510.12169v2)

---

## What Is a Spec Doc (and Why You're Writing One)

A spec doc (also called a PRD — Product Requirements Document) is a written contract you make with yourself and your client before you write a single line of code. It answers three questions:

1. **What** are we building, exactly?
2. **Why** are we building it this way?
3. **How** do we know when we're done?

Writing this upfront forces you to surface assumptions before they become bugs. For a robotics project, it's especially useful because the line between "algorithm works" and "system works" is blurry — the spec keeps you from confusing the two. You'll refer back to this when you're lost at 2am wondering why your robot is driving into a wall.

---

## Problem Statement

Autonomous snow removal robots must reliably navigate back to a wireless charging station after completing a work cycle — or when battery levels drop below a safe threshold. In dynamic outdoor environments (shifting snow, people, new obstacles), a static planner cannot guarantee the robot reaches the charger without getting stuck or taking a dangerously long route. The client needs a working, demonstrable implementation of two complementary algorithms — **RRT*** for computing an optimal initial path to the charging station and **D* Lite** for real-time replanning when obstacles appear en route — integrated into a ROS2/Gazebo simulation pipeline. Failure to solve this means the robot either drains its battery mid-operation or requires manual retrieval, defeating the purpose of full autonomy.

---

## Goals

1. **RRT\* produces a collision-free, asymptotically optimal path** from the robot's current pose to the wireless charging station on a 2D occupancy grid with static obstacles, verifiable in under 5 seconds on a standard map size (100x100 cells).
2. **D\* Lite successfully replans the return-to-charger route** when new obstacles appear mid-execution, updating the path in under 500ms without restarting from scratch.
3. **Both algorithms run as standalone Python scripts locally** (Mac) before any ROS2 integration, with matplotlib visualization of the planned path and tree/graph structure.
4. **A ROS2 Gazebo sim demo** shows the 4-wheeled robot autonomously navigating back to a designated charging station, replanning around a dynamically spawned obstacle en route.
5. **Code is cleanly modularized** so RRT* and D* Lite can be swapped out or run in combination without refactoring the surrounding system, and the charging station goal is a configurable parameter (not hardcoded).

---

## Non-Goals

- **Full coverage path planning** (mowing-pattern, boustrophedon, etc.) — that's a separate problem from return-to-charger navigation. Out of scope for v1; algorithms should be designed so coverage logic can be layered on top later.
- **3D path planning** — we're operating on a 2D occupancy grid. Real terrain elevation is a v2 concern.
- **Hardware deployment** — the deliverable is a sim demo, not hardware-ready code. No CAN bus, motor controllers, or real sensor drivers.
- **SLAM / autonomous mapping** — the map is assumed to be known (static or with injected dynamic obstacles). Building the map is not part of this feature.
- **Custom ROS2 Nav2 plugin integration** — we're building standalone ROS2 nodes first. Plugging into Nav2's planner server is a future concern.

---

## Algorithm Primer

### RRT* — Rapidly-exploring Random Tree Star

RRT* is a **global path planner**. You use it once at the start (or when the goal changes) to find the best path through a known map.

**How it works:**
1. Start with a tree rooted at the robot's current position.
2. Sample a random point in free space.
3. Find the nearest node in the tree, extend toward the sample.
4. **Rewire**: check if any nearby nodes would have a shorter path if rerouted through the new node — if yes, reconnect them. This is the `*` part that makes it optimal.
5. Repeat until the goal is reached or a max iteration count is hit.

**Key parameters:**
- `step_size`: how far to extend each branch (meters or grid cells)
- `goal_bias`: probability of sampling the goal directly (speeds up convergence, typically 0.05–0.15)
- `neighbor_radius`: search radius for the rewiring step (critical for optimality)
- `max_iterations`: when to stop trying

**Why it fits return-to-charger:** The robot needs to find the shortest viable path back to a fixed charging station from an arbitrary field position. Snow environments have large open areas with sparse obstacles, which RRT* handles efficiently. The charger's location is known and fixed, making this an ideal single-query planning problem — compute once, follow the path.

---

### D* Lite — Dynamic A*

D* Lite is a **local replanner**. It's designed to handle maps that change while the robot is moving — exactly what happens when sensors detect a new snow drift or obstacle mid-route.

**How it works:**
1. Plans a path from goal → start (backwards) using a priority queue, similar to A*.
2. As the robot moves and sensors update the map, only the **affected portion** of the path is recalculated — not the whole thing.
3. Uses a two-key priority system (`k1 = min(g, rhs) + h`, `k2 = min(g, rhs)`) to track which nodes are inconsistent and need updating.

**Key parameters:**
- `heuristic`: typically Euclidean or Manhattan distance
- `obstacle_cost`: cost assigned to occupied cells (usually infinity)
- `update_threshold`: how much a cell cost must change before triggering a replan

**Why it fits return-to-charger:** The path back to the charging station can get blocked mid-route — a snow drift closes off a corridor, a person steps into the path, a gate closes. D* Lite handles this without throwing away the whole plan. It pairs perfectly with RRT*: use RRT* once to compute the initial optimal return path, then hand off to D* Lite to keep that path valid as the world changes. This is critical for battery safety — a failed replan means the robot can't charge.

---

## User Stories

**As the engineer (you),** I want a standalone Python RRT* implementation that I can run on my Mac so that I can validate the algorithm produces an optimal path to the charger location before introducing ROS2 complexity.

**As the engineer,** I want a standalone Python D* Lite implementation with simulated "obstacle injection" mid-route so that I can verify the return-to-charger path correctly updates without a full replan.

**As the engineer,** I want both algorithms to share a common `Map` interface and accept a configurable `goal` pose so that swapping planners or changing the charger's location requires no structural code changes.

**As the client,** I want to see the robot autonomously navigate back to its wireless charging station in a Gazebo snow world — replanning around a dynamically spawned obstacle en route — so that I can validate the autonomous charging behavior end-to-end.

**As the robot's operator,** I want the charging station's position to be a configurable parameter (not hardcoded) so that the system works if the charger is relocated on the property.

**As a future engineer,** I want the planner nodes cleanly separated from the robot controller so that the return-to-charger logic can be triggered by a battery monitor node without coupling it to the planner implementation.

---

## Requirements

### P0 — Must Have (MVP)

**1. 2D Occupancy Grid Map Abstraction**
- A `Map` class (Python) that wraps a 2D array of cells (`0` = free, `1` = occupied, `-1` = unknown).
- Methods: `is_free(x, y)`, `set_obstacle(x, y)`, `get_neighbors(x, y)`, `to_image()` (for visualization).
- Acceptance criteria:
  - [ ] Map loads from a hardcoded array and from a PNG file (white = free, black = occupied).
  - [ ] `set_obstacle()` updates the grid in place and downstream planners see the change.

**2. RRT* — Standalone Python Implementation**
- File: `rrt_star.py`
- Inputs: `Map`, `start: (x, y)`, `goal: (x, y)`, planning params.
- Output: `list[tuple[x, y]]` representing the planned path.
- Acceptance criteria:
  - [ ] Finds a collision-free path on a 100x100 grid with 20% obstacle density.
  - [ ] Path improves (gets shorter) as `max_iterations` increases — demonstrating the `*` property.
  - [ ] Raises `PlanningFailure` if no path exists within `max_iterations`.
  - [ ] `matplotlib` visualization shows the explored tree + final path.
  - [ ] Runs in under 10 seconds locally for `max_iterations=5000`.

**3. D* Lite — Standalone Python Implementation**
- File: `d_star_lite.py`
- Inputs: `Map`, `start: (x, y)`, `goal: (x, y)`.
- Output: Current best path as `list[tuple[x, y]]`, updated in-place when `update_obstacle()` is called.
- Acceptance criteria:
  - [ ] Finds initial path on same 100x100 test map as RRT*.
  - [ ] When `update_obstacle(x, y)` is called with a cell on the current path, produces a new valid path within 500ms.
  - [ ] When `update_obstacle(x, y)` is called with a cell NOT on the current path, path is unchanged.
  - [ ] `matplotlib` visualization shows original path, injected obstacle, and replanned path.

**4. ROS2 Planner Node**
- Package: `snow_planner`
- Nodes: `rrt_star_node.py` and `d_star_lite_node.py`
- Subscribes: `/map` (`nav_msgs/OccupancyGrid`), `/goal_pose` (`geometry_msgs/PoseStamped`), `/obstacle_update` (custom or `std_msgs/String` for MVP)
- Publishes: `/plan` (`nav_msgs/Path`)
- Acceptance criteria:
  - [ ] `ros2 run snow_planner rrt_star_node` starts without errors.
  - [ ] Publishing a goal to `/goal_pose` causes a path to appear on `/plan`.
  - [ ] Path is visible in RViz2 as a green line.
  - [ ] Injecting an obstacle via `/obstacle_update` causes D* Lite node to publish an updated path.

**5. Gazebo Sim Demo**
- A Gazebo world (`snow_world.sdf`) with flat terrain, a few static box obstacles, a 4-wheeled robot model, and a **wireless charging station model** at a fixed location.
- Robot follows the planned path using a simple path-following controller node.
- Acceptance criteria:
  - [ ] Robot navigates from spawn point to the charging station without hitting static obstacles.
  - [ ] When a dynamic obstacle is spawned mid-run, robot replans and still reaches the charger.
  - [ ] Charging station goal pose is read from a config file / ROS2 param, not hardcoded.
  - [ ] Robot stops (velocity = 0) upon reaching the charger's docking pose (within tolerance).
  - [ ] Demo is launchable with a single command: `ros2 launch snow_planner demo.launch.py`

---

### P1 — Nice to Have

**6. Path Smoothing Post-Processing**
- After RRT* produces a path, run a shortcutting or spline-fitting pass to remove jagged waypoints.
- Not critical for the demo but makes the robot's motion look less erratic.

**7. Trajectory Velocity Profile**
- Assign speeds to waypoints based on curvature (slow down at turns).
- Required for realistic robot behavior but the path-following controller can fake this with a constant speed for v1.

**8. Non-holonomic Constraint Handling**
- RRT* currently plans in (x, y). Extending to (x, y, θ) with Dubins or Reeds-Shepp curves respects the 4-wheeled robot's turning radius.
- Important for hardware, less critical for sim where we can cheat with a differential-drive controller.

---

### P2 — Future Considerations

**9. Nav2 Planner Plugin Interface**
- Wrap RRT* as a proper `nav2_core::GlobalPlanner` plugin so it integrates with the full Nav2 stack.

**10. Coverage Path Planning**
- Use RRT* or a grid-based approach to generate a boustrophedon (lawnmower) coverage path for full-yard snow clearing.

**11. 3D Terrain Awareness**
- Extend the map to an elevation map (2.5D) so the planner avoids steep inclines.

---

## Implementation Steps

This is your build order. Do each step locally first. Don't touch ROS2 until Step 4.

### Step 1: Local Map + RRT* (Day 1 AM)

```
project/
├── map.py          # Map class
├── rrt_star.py     # RRT* implementation
├── test_rrt.py     # Test + visualization script
└── maps/
    └── test_map.png
```

1. Write `Map` class with a hardcoded 2D test array.
2. Implement the core RRT* loop: sample → nearest → extend → collision check → rewire.
3. Add goal-reaching check.
4. Add `matplotlib` visualization: draw the grid, tree edges, and final path.
5. Tune `step_size`, `goal_bias`, `neighbor_radius` until you consistently get good paths.

**The rewiring step is where most bugs hide.** When you extend to a new node, you must check all nodes within `neighbor_radius` — not just the nearest one — and reconnect any whose path through the new node is cheaper. If you skip this, you have RRT, not RRT*.

---

### Step 2: Local D* Lite (Day 1 PM)

```
project/
├── d_star_lite.py
└── test_dstar.py
```

1. Implement `rhs` and `g` value tables (both initialized to ∞ except `rhs[goal] = 0`).
2. Implement the priority queue using Python's `heapq` with the two-key comparator.
3. Implement `compute_shortest_path()` — the main planning loop.
4. Implement `update_vertex(u)` — called whenever an edge cost changes.
5. Write a test that: plans a path, injects an obstacle on that path, calls `update_vertex` on affected cells, replans, and visualizes both paths.

**D* Lite is trickier to debug than RRT\*.** The key insight: it plans backwards (goal → start). When you call the path, you follow the `g` value gradient from start → goal. If your path looks wrong, check that you're initializing from the goal, not the start.

---

### Step 3: Shared Interfaces + Test Suite (Day 1 EOD)

1. Define a `Planner` abstract base class with `plan(start, goal) -> List[Tuple]` and `update_obstacle(x, y)`.
2. Make both `RRTStar` and `DStarLite` implement it.
3. Write a head-to-head comparison script: same map, same start/goal, run both, visualize side by side.
4. Verify: RRT* path is smoother/more optimal, D* Lite handles dynamic updates faster.

---

### Step 4: ROS2 Package Setup (Day 2 AM)

```
ros2_ws/
└── src/
    └── snow_planner/
        ├── package.xml
        ├── setup.py
        ├── snow_planner/
        │   ├── __init__.py
        │   ├── rrt_star_node.py   # Wraps Step 1 code
        │   ├── d_star_lite_node.py
        │   └── map_utils.py       # OccupancyGrid ↔ Map conversion
        └── launch/
            └── demo.launch.py
```

1. Create the package: `ros2 pkg create --build-type ament_python snow_planner`
2. Copy your local `rrt_star.py` and `d_star_lite.py` into the package (don't rewrite them — wrap them).
3. Write `map_utils.py` with `occupancy_grid_to_map(msg) -> Map` and `path_to_nav_path(path, frame) -> nav_msgs/Path`.
4. Write `rrt_star_node.py`:
   - Subscribe to `/map` and cache the map.
   - Subscribe to `/goal_pose` — on receipt, run RRT* and publish to `/plan`.
5. Write `d_star_lite_node.py`:
   - Same as above, plus subscribe to obstacle updates and republish updated plan.
6. Test in RViz2 without Gazebo: publish a fake map and goal, confirm path appears.

---

### Step 5: Gazebo Integration (Day 2 PM – Day 3)

1. Create a minimal `snow_world.sdf` — flat plane, 3–5 box obstacles, ambient lighting.
2. Create a 4-wheeled robot URDF (`snow_bot.urdf.xacro`):
   - 4 wheel joints, a chassis, a generic laser scan sensor (for future use).
   - Use `diff_drive` controller or `ackermann_drive` — diff drive is simpler.
3. Write a `path_follower_node.py`:
   - Subscribe to `/plan`.
   - For each waypoint, compute heading error and distance.
   - Publish `geometry_msgs/Twist` to `/cmd_vel` to drive toward the next waypoint.
   - Simple proportional controller is fine for the demo.
4. Wire everything in `demo.launch.py`:
   - Launch Gazebo with `snow_world.sdf`
   - Spawn robot
   - Launch RRT* node
   - Launch D* Lite node
   - Launch path follower node
   - Launch RViz2 with a premade config showing map + path + robot
5. Run the demo end-to-end. Record a short video or screenshot sequence for the client.

---

## Success Metrics

### Local (Steps 1–3)
- RRT* finds a valid path on 10/10 test maps with < 20% obstacle density.
- D* Lite replans correctly on 10/10 injected obstacle scenarios.
- Both run in < 10 seconds on a 100x100 grid locally.

### ROS2 (Steps 4–5)
- A goal published to `/goal_pose` produces a `/plan` response within 2 seconds.
- The Gazebo demo robot reaches its goal without collision in a static world.
- When a Gazebo obstacle is spawned on the path, the robot replans and reaches goal.

---

## Open Questions

| Question | Owner | Blocking? |
|---|---|---|
| Does the client have a specific map format / environment size they want simulated? | Client | No — use a generic world for now |
| Should RRT* or D* Lite be the "primary" planner, or are they expected to run in tandem? | Client/Engineering | No — design for composability, clarify later |
| Does the 4-wheeled robot use Ackermann steering or differential drive? This affects non-holonomic constraints | Client | No for sim (use diff drive), Yes for hardware |
| What's the expected outdoor operating area size? Matters for map resolution vs. planning time tradeoff | Client | No for v1 |
| **What triggers the return-to-charger behavior?** Battery threshold, end of work cycle, manual command, or all three? | Client | No for sim (manually trigger for demo), Yes for production |
| **Does the wireless charging require precise docking alignment** (e.g. within 5cm, specific heading)? Or is proximity enough? | Client | No for sim, Yes for hardware |
| Reference paper (arXiv 2510.12169v2) — any specific algorithm variant or evaluation metric from the paper that must be reproduced? | You | Yes — review paper before finalizing algorithm params |

---

## Technical Gotchas

Things that will bite you if you don't know about them:

**RRT\* rewiring radius:** If `neighbor_radius` is too small, you get RRT (not optimal). If too large, it's slow. A good heuristic is `neighbor_radius = min(γ * (log(n)/n)^(1/d), step_size)` where `n` = current tree size, `d` = dimensions (2), `γ` = tuning constant.

**D\* Lite key comparison:** The priority queue comparator must use the full two-key tuple `(k1, k2)` for tie-breaking, not just `k1`. Get this wrong and you'll get infinite loops.

**OccupancyGrid coordinate frame:** ROS2 `OccupancyGrid` uses `(row, col)` = `(y, x)` in grid space, but your planner likely uses `(x, y)`. Define this conversion once in `map_utils.py` and use it everywhere or you'll get mirrored paths.

**Gazebo robot spawning:** The robot URDF needs `<ros2_control>` tags and a controller config `.yaml` for `diff_drive` to work. Without it, `cmd_vel` does nothing.

**Mac + ROS2:** Use Docker (ROS2 Humble image) or a VM if you're not already on Ubuntu. ROS2 Humble on macOS via Homebrew is finicky. Docker is faster to set up.

---

## File Structure (Final)

```
snow_planner_ws/
├── local/                      # Phase 1 — Mac standalone
│   ├── map.py
│   ├── rrt_star.py
│   ├── d_star_lite.py
│   ├── planner.py              # Abstract base class
│   ├── test_rrt.py
│   ├── test_dstar.py
│   └── maps/
│       └── test_map.png
└── ros2_ws/                    # Phase 2 — ROS2 + Gazebo
    └── src/
        └── snow_planner/
            ├── package.xml
            ├── setup.py
            ├── resource/
            ├── snow_planner/
            │   ├── rrt_star_node.py
            │   ├── d_star_lite_node.py
            │   ├── path_follower_node.py
            │   └── map_utils.py
            ├── models/
            │   └── snow_bot.urdf.xacro
            ├── worlds/
            │   └── snow_world.sdf
            └── launch/
                └── demo.launch.py
```

---

*Spec version 1.0 — Ready for implementation. Update open questions after client review.*
