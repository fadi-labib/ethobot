# Manual Test Plan — PSO Waypoint Demo

Covers the behaviour that cannot be exercised by `colcon test` or headless node
runs: robot motion, Gazebo/RViz coordinate alignment, and the waypoint
state-machine. The unit tests cover the `PsoSolver` and `RobotController`
classes; running the PSO node and visualizer headless confirms parameter loading
and PSO convergence. Everything below requires a machine with a display/GPU
(Gazebo + RViz).

## Prerequisites

```bash
# One-time deps (if not already installed)
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-nav2-bringup ros-jazzy-ros-gz

# Every shell
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
cd ~/projects/biorobot
colcon build --packages-select ethobot_core ethobot_interfaces ethobot_algorithms ethobot_robots ethobot_simulation
source install/setup.bash
```

## T1 — Baseline obstacle-avoidance run (golden path)

```bash
ros2 launch ethobot_simulation gazebo_pso_demo.launch.py
```

Watch the terminal for, in order: `Obstacles: 4` → PSO `Iteration 10/50 ... 50/50`
→ `OPTIMIZATION COMPLETE` → `PSO COMPLETE` → `>>> PHASE 1: Navigating to
waypoint ...` → `Waypoint reached` → `>>> PHASE 2: Navigating to goal ...` →
`NAVIGATION COMPLETE`.

**Pass:** robot drives start → waypoint → goal, ends within 0.2 m of (3,3), and
never touches a cylinder. The iteration log must advance 10 → 50 (regression
guard for the iteration-counter bug).

## T2 — Gazebo/RViz coordinate alignment

With T1 still running, inspect RViz beside Gazebo.

**Pass:** the four gray cylinder markers in RViz sit at the same world positions
as the Gazebo obstacles; the red goal marker overlays the green Gazebo goal
sphere at (3,3); the converged green "global best" marker sits where the robot
actually heads. **Fail = the alignment bug is not fully resolved** — record any
constant XY offset (that is the map↔odom shift).

## T3 — Off-origin spawn (exercises the map→odom fix)

```bash
ros2 launch ethobot_simulation gazebo_pso_demo.launch.py robot_x:=1.0 robot_y:=1.0
```

**Pass:** robot spawns at (1,1) in Gazebo, PSO logs `Start: (1.0, 1.0)`, and the
robot still reaches (3,3). Before the fix, the static `map→odom` transform was
hardcoded to identity and the robot would stop ~(1,1) short of the goal. This is
the most important case — it is the bug that "works by accident" at the origin.

## T4 — Degenerate-waypoint early-out (clear straight path)

```bash
ros2 launch ethobot_simulation gazebo_pso_demo.launch.py goal_x:=3.0 goal_y:=0.0
```

The straight line y=0 is clear of all obstacles (they are at y ≥ 1).

**Pass:** the terminal shows `>>> Waypoint adds no value (direct path) — skipping
to goal <<<` and the robot goes straight to (3,0) with no Phase-1 detour. (The
Gazebo goal sphere stays at (3,3) here — ignore it; the navigation goal is (3,0).)

## T5 — Obstacle single-source-of-truth

```bash
# Move obstacle 1 in the YAML, e.g. change obstacle_x: [1.0, ...] -> [0.5, ...]
nano src/ethobot_algorithms/config/obstacles.yaml
colcon build --packages-select ethobot_algorithms && source install/setup.bash
ros2 launch ethobot_simulation gazebo_pso_demo.launch.py
```

**Pass:** the RViz marker and the PSO planned path both reflect the moved
obstacle. (Gazebo's physical cylinder will not move — it lives in the SDF by
design; edit `worlds/ethobot_world.sdf` too if you want the physics to match.)

## T6 — load_obstacles() validation guard

```bash
# Temporarily give the arrays mismatched lengths (e.g. 4 x-values, 3 y-values)
ros2 run ethobot_algorithms pso_path_planning_node --ros-args --params-file src/ethobot_algorithms/config/obstacles.yaml
```

**Pass:** logs `obstacle_x/y/radius length mismatch (...) — ignoring obstacles`
and PSO still runs (0 obstacles) rather than crashing. Revert the YAML afterward.

## T7 — Regression

```bash
colcon test && colcon test-result --verbose
```

**Pass:** `123 tests, 0 errors, 0 failures, 12 skipped`.

## Evidence to capture

- A Gazebo + RViz screenshot from T2 (alignment).
- Terminal logs from T3 (off-origin spawn reaching goal) and T4 (early-out skip).
