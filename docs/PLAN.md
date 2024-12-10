# Ethobot Implementation Plan

This document outlines the implementation roadmap for the Ethobot project.

## Overview

Ethobot is being developed incrementally, starting with a minimal viable implementation and expanding functionality over time.

```
Phase 1: Foundation      → Interfaces and core structure    ✅ COMPLETE
Phase 2: First Algorithm → PSO implementation               ✅ COMPLETE
Phase 3: Simulation      → Gazebo integration with ground robot
Phase 4: Expansion       → Additional algorithms and robot types
```

---

## Phase 1: Foundation ✅

**Goal**: Establish project structure and ROS2 interfaces.

### Completed

- [x] Project structure and workspace layout
- [x] Git repository initialization
- [x] License (MIT)
- [x] README.md with project overview
- [x] CLAUDE.md with development guidelines
- [x] Documentation structure (docs/)
- [x] `ethobot_interfaces` package
  - [x] Message definitions (ParticleState, SwarmState, RobotState, etc.)
  - [x] Service definitions (SpawnRobot, SetAlgorithmParams, GetMetrics)
  - [x] Action definitions (RunOptimization)
  - [x] Build verification with colcon

---

## Phase 2: PSO Algorithm ✅

**Goal**: Implement Particle Swarm Optimization as the first bio-inspired algorithm.

### Completed

- [x] Create `ethobot_core` package
  - [x] `AlgorithmBase` abstract class
  - [x] `Problem` struct (fitness function interface)
  - [x] `OptimizationResult` struct

- [x] Create `ethobot_algorithms` package
  - [x] `PsoSolver` class (custom PSO implementation)
  - [x] `pso_path_planning_node` ROS2 node
  - [x] `swarm_visualizer_node` for RViz visualization
  - [x] Launch file with RViz configuration

### PSO Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `population_size` | 30 | Number of particles |
| `max_iterations` | 100 | Maximum iterations |
| `inertia_weight` | 0.7 | Velocity inertia (w) |
| `cognitive_coeff` | 1.5 | Personal best weight (c1) |
| `social_coeff` | 1.5 | Global best weight (c2) |
| `max_velocity` | 0.5 | Maximum particle velocity |

### PSO Demo: 2D Path Planning

The demo finds an optimal position in a 2D space (12x12) that:
- Minimizes distance to goal at (10, 10)
- Avoids 4 circular obstacles

**Fitness Function:**
```
fitness = distance_to_goal + obstacle_penalty
```

Where `obstacle_penalty`:
- 1000 if inside obstacle
- 50 × clearance_distance if within 1.0 unit of obstacle edge

### Deliverables

- [x] Working PSO algorithm node
- [x] Configurable via ROS2 parameters
- [x] Publishes SwarmState for visualization
- [x] RViz visualization with markers
- [ ] Unit tests (future)

---

## Phase 3: Ground Robot Simulation

**Goal**: Integrate TurtleBot3 in Gazebo for path planning demonstration.

### Tasks

- [ ] Create `ethobot_robots` package
  - [ ] `RobotInterface` abstract class
  - [ ] `GroundRobot` implementation (TurtleBot3)
  - [ ] Velocity command translation
  - [ ] Odometry subscription

- [ ] Create `ethobot_simulation` package
  - [ ] Gazebo world (simple obstacles)
  - [ ] Robot spawning service
  - [ ] Simulation manager node

- [ ] Path planning scenario
  - [ ] Define start/goal positions
  - [ ] Obstacle representation
  - [ ] Fitness function (path length + obstacles)
  - [ ] Launch file

### Dependencies for This Phase

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz
```

### Deliverables

1. TurtleBot3 controlled by PSO path planning
2. Gazebo simulation environment
3. Launch files for easy startup
4. Demo video/documentation

---

## Phase 4: Algorithm Expansion

**Goal**: Add additional bio-inspired algorithms.

### Algorithms to Implement

| Algorithm | Priority | Use Case |
|-----------|----------|----------|
| ACO (Ant Colony) | High | Multi-robot routing |
| GA (Genetic Algorithm) | Medium | Controller optimization |
| DE (Differential Evolution) | Medium | Parameter tuning |
| Firefly Algorithm | Low | Multi-objective optimization |

### Tasks per Algorithm

- [ ] Algorithm implementation (Pagmo2 wrapper)
- [ ] ROS2 node
- [ ] Parameter configuration
- [ ] Unit tests
- [ ] Documentation
- [ ] Example scenario

---

## Phase 5: Multi-Robot and Swarm

**Goal**: Enable multiple robots working together.

### Tasks

- [ ] Multi-robot spawning
- [ ] Swarm coordination
- [ ] Communication between robots
- [ ] Foraging scenario
- [ ] Formation control scenario

---

## Phase 6: Aerial Robots

**Goal**: Add quadrotor support for 3D scenarios.

### Tasks

- [ ] `AerialRobot` implementation
- [ ] PX4/Gazebo integration
- [ ] 3D path planning
- [ ] Formation flying scenario

---

## Success Metrics

| Metric | Target |
|--------|--------|
| Build success | 100% packages build |
| Test coverage | >80% |
| Documentation | All public APIs documented |
| Performance | PSO converges in <100 iterations for test problems |

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Pagmo2 API changes | Medium | Pin version, wrap API |
| Gazebo-ROS2 bridge issues | High | Test early, use stable versions |
| Performance bottlenecks | Medium | Profile regularly, optimize critical paths |

## Resources

### References
- [Pagmo2 Documentation](https://esa.github.io/pagmo2/)
- [ROS2 Documentation](https://docs.ros.org/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Gazebo Sim Tutorials](https://gazebosim.org/docs)

### Papers
- Kennedy & Eberhart (1995) - Particle Swarm Optimization
- Dorigo & Stützle (2004) - Ant Colony Optimization
- Holland (1975) - Genetic Algorithms
