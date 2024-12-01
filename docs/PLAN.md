# Ethobot Implementation Plan

This document outlines the implementation roadmap for the Ethobot project.

## Overview

Ethobot is being developed incrementally, starting with a minimal viable implementation and expanding functionality over time.

```
Phase 1: Foundation     → Interfaces and core structure
Phase 2: First Algorithm → PSO implementation
Phase 3: Simulation     → Gazebo integration with ground robot
Phase 4: Expansion      → Additional algorithms and robot types
```

---

## Phase 1: Foundation (Current)

**Goal**: Establish project structure and ROS2 interfaces.

### Completed

- [x] Project structure and workspace layout
- [x] Git repository initialization
- [x] License (MIT)
- [x] README.md with project overview
- [x] CLAUDE.md with development guidelines
- [x] Documentation structure (docs/)

### In Progress

- [ ] `ethobot_interfaces` package
  - [x] Message definitions (ParticleState, SwarmState, RobotState, etc.)
  - [x] Service definitions (SpawnRobot, SetAlgorithmParams, GetMetrics)
  - [x] Action definitions (RunOptimization)
  - [ ] Build verification with colcon

### Dependencies for This Phase

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-desktop ros-dev-tools
```

### Deliverables

1. Compilable `ethobot_interfaces` package
2. Complete documentation structure
3. Ready for algorithm implementation

---

## Phase 2: PSO Algorithm

**Goal**: Implement Particle Swarm Optimization as the first bio-inspired algorithm.

### Tasks

- [ ] Create `ethobot_core` package
  - [ ] `AlgorithmBase` abstract class
  - [ ] `Problem` struct (fitness function interface)
  - [ ] `OptimizationResult` struct
  - [ ] Utility functions (coordinate transforms, etc.)

- [ ] Create `ethobot_algorithms` package
  - [ ] `PsoAlgorithm` class (wraps Pagmo2 PSO)
  - [ ] `PsoNode` ROS2 node
  - [ ] Parameter configuration (YAML)
  - [ ] Unit tests

### PSO Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `population_size` | 50 | Number of particles |
| `max_iterations` | 100 | Maximum iterations |
| `inertia_weight` | 0.7 | Velocity inertia |
| `cognitive_coeff` | 1.5 | Personal best weight |
| `social_coeff` | 1.5 | Global best weight |

### Dependencies for This Phase

```bash
sudo apt install libpagmo-dev libeigen3-dev
```

### Deliverables

1. Working PSO algorithm node
2. Configurable via ROS2 parameters
3. Publishes SwarmState for visualization
4. Unit tests with >80% coverage

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
