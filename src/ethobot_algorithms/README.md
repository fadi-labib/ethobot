# ethobot_algorithms

Bio-inspired optimization algorithms for robotics applications in ROS2.

## Overview

This package implements bio-inspired optimization algorithms that can be used for path planning, parameter tuning, and swarm coordination. Currently implements Particle Swarm Optimization (PSO).

## Algorithms

### Particle Swarm Optimization (PSO)

PSO is inspired by the social behavior of bird flocking and fish schooling. A swarm of particles explores the search space, sharing information about good solutions.

**How it works:**
1. Initialize particles at random positions
2. Each particle remembers its personal best position
3. All particles know the global best position
4. Particles update velocity toward personal and global bests
5. Add some randomness for exploration
6. Repeat until convergence

**Velocity Update Equation:**
```
v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)
```

Where:
- `w` = inertia weight (momentum)
- `c1` = cognitive coefficient (personal best attraction)
- `c2` = social coefficient (global best attraction)
- `r1, r2` = random values in [0,1]
- `pbest` = particle's personal best position
- `gbest` = swarm's global best position

**Position Update:**
```
x = x + v
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `population_size` | 30 | Number of particles in the swarm |
| `inertia_weight` | 0.7 | Controls momentum (w). Higher = more exploration |
| `cognitive_coeff` | 1.5 | Personal best attraction (c1) |
| `social_coeff` | 1.5 | Global best attraction (c2) |
| `max_velocity` | 1.0 | Maximum velocity per dimension |

**Tuning Tips:**
- High `inertia_weight` (>0.8): More exploration, slower convergence
- Low `inertia_weight` (<0.4): Faster convergence, may get stuck
- Balanced c1=c2: Good general performance
- High c2 > c1: Faster convergence to global best
- High c1 > c2: More diverse exploration

## Nodes

### pso_path_planning_node

Demo node that finds an optimal position avoiding obstacles.

**Published Topics:**
- `/ethobot/swarm_state` (SwarmState): Current particle positions
- `/ethobot/algorithm_status` (AlgorithmStatus): Optimization progress

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x` | 10.0 | Goal X position |
| `goal_y` | 10.0 | Goal Y position |
| `population_size` | 30 | Number of particles |
| `max_iterations` | 100 | Maximum iterations |
| `publish_rate_hz` | 10.0 | State publish rate |

### swarm_visualizer_node

Visualizes the PSO swarm in RViz using MarkerArray.

**Published Topics:**
- `/ethobot/visualization` (MarkerArray): RViz markers

**Subscribed Topics:**
- `/ethobot/swarm_state` (SwarmState): Swarm state to visualize

**Visualization:**
- Blue/Red spheres: Particles (color = fitness quality)
- Green sphere: Global best position
- Red sphere: Goal position
- Gray cylinders: Obstacles

## Launch Files

### pso_demo.launch.py

Launches the complete PSO visualization demo.

```bash
ros2 launch ethobot_algorithms pso_demo.launch.py
```

**Arguments:**
```bash
ros2 launch ethobot_algorithms pso_demo.launch.py \
    goal_x:=8.0 \
    goal_y:=8.0 \
    population_size:=50 \
    max_iterations:=200
```

**What it launches:**
1. `pso_path_planning_node` - Runs PSO optimization
2. `swarm_visualizer_node` - Publishes RViz markers
3. `static_transform_publisher` - Broadcasts `map` TF frame
4. `rviz2` - Visualization with pre-configured display

## Usage

### As a Library

```cpp
#include "ethobot_algorithms/pso_solver.hpp"

// Configure PSO
ethobot_algorithms::PsoParams params;
params.population_size = 50;
params.inertia_weight = 0.7;
params.cognitive_coeff = 1.5;
params.social_coeff = 1.5;

// Create solver
auto pso = std::make_unique<ethobot_algorithms::PsoSolver>(params);

// Define problem
ethobot_core::Problem problem;
problem.dimensions = 2;
problem.lower_bounds = {0.0, 0.0};
problem.upper_bounds = {10.0, 10.0};
problem.minimize = true;
problem.fitness_function = [](const std::vector<double>& pos) {
  // Your fitness function here
  return compute_fitness(pos);
};

// Initialize and run
pso->initialize(problem);

// Option 1: Run to completion
auto result = pso->solve(100);

// Option 2: Step-by-step (for ROS integration)
for (int i = 0; i < 100; ++i) {
  pso->step();
  auto state = pso->get_swarm_state();  // For visualization
}
```

## Demo: 2D Path Planning

The demo finds an optimal (x, y) position that:
1. Minimizes distance to goal (10, 10)
2. Avoids 4 circular obstacles

**Search Space:** 12x12 grid (0,0) to (12,12)

**Obstacles:**
| Center | Radius |
|--------|--------|
| (3, 3) | 1.5 |
| (7, 4) | 1.0 |
| (5, 7) | 1.2 |
| (2, 8) | 0.8 |

**Fitness Function:**
```
fitness = distance_to_goal + obstacle_penalty
```

Where:
- `distance_to_goal = sqrt((x-10)^2 + (y-10)^2)`
- `obstacle_penalty = 1000` if inside obstacle
- `obstacle_penalty = 50 * clearance` if within 1.0 unit of edge

## Dependencies

- rclcpp
- ethobot_core
- ethobot_interfaces
- geometry_msgs
- std_msgs
- visualization_msgs
- tf2_ros

## Build

```bash
colcon build --packages-select ethobot_algorithms
```

## Future Algorithms

Planned implementations:
- **ACO** (Ant Colony Optimization): Multi-robot routing
- **GA** (Genetic Algorithm): Controller evolution
- **DE** (Differential Evolution): Parameter tuning
