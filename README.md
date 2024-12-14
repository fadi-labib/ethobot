# Ethobot

A modular C++ framework for bio-inspired robotics algorithms in ROS2.

> **Ethobot** (from *ethology* — the study of animal behavior) brings nature-inspired optimization algorithms to robotics, enabling intelligent path planning, swarm coordination, and adaptive behaviors.

## Why Bio-Inspired Algorithms?

Nature has solved optimization problems over millions of years. Bio-inspired algorithms mimic these solutions:

| Algorithm | Inspired By | Robotics Application |
|-----------|-------------|---------------------|
| **PSO** | Bird flocking, fish schooling | Path planning, parameter tuning |
| **ACO** | Ant pheromone trails | Multi-robot routing, coverage |
| **GA** | Natural selection | Controller evolution, morphology |

## Current Status

- [x] Project structure and interfaces
- [x] PSO algorithm implementation
- [x] RViz visualization
- [x] Ground robot integration (TurtleBot3)
- [ ] Gazebo simulation

See [docs/PLAN.md](docs/PLAN.md) for the full roadmap.

## Project Structure

```
ethobot/
├── src/
│   ├── ethobot_interfaces/    # ROS2 messages, services, actions
│   ├── ethobot_core/          # Base classes and utilities
│   ├── ethobot_algorithms/    # Algorithm implementations (PSO, etc.)
│   └── ethobot_robots/        # Robot controllers (TurtleBot3)
├── config/                    # Parameter files (YAML)
├── launch/                    # Launch files
├── worlds/                    # Gazebo simulation worlds
├── models/                    # Robot URDF/SDF models
├── docs/                      # Documentation
│   ├── PLAN.md               # Implementation roadmap
│   ├── ARCHITECTURE.md       # System design
│   └── CONTRIBUTING.md       # Contribution guidelines
└── scripts/                   # Utility scripts
```

## Requirements

| Dependency | Version | Purpose |
|------------|---------|---------|
| Ubuntu | 22.04 / 24.04 | Operating system |
| ROS2 | Humble / Jazzy | Robotics middleware |
| Gazebo Sim | Harmonic | Simulation (via ros-gz) |
| Pagmo2 | 2.18+ | Optimization algorithms |
| Eigen3 | 3.4+ | Linear algebra |

## Installation

### 1. Install ROS2 (if not already installed)

```bash
# Ubuntu 24.04 (Jazzy)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools
```

### 2. Install Dependencies

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install dependencies
sudo apt install \
    libpagmo-dev \
    libeigen3-dev \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-fake-node \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz
```

### 3. Clone and Build

```bash
git clone https://github.com/yourusername/ethobot.git
cd ethobot

# Source ROS2 (add to ~/.bashrc for convenience)
source /opt/ros/jazzy/setup.bash

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage

### Build

```bash
cd ~/projects/biorobot
source /opt/ros/jazzy/setup.zsh  # or setup.bash
colcon build --packages-select ethobot_interfaces ethobot_core ethobot_algorithms ethobot_robots
source install/setup.zsh
```

### Run PSO Demo

```bash
ros2 launch ethobot_algorithms pso_demo.launch.py
```

This launches:
- **pso_path_planning_node**: Runs PSO optimization to find goal position
- **swarm_visualizer_node**: Publishes RViz markers for visualization
- **static_transform_publisher**: Broadcasts the `map` TF frame
- **rviz2**: Visualization with pre-configured display

### Custom Parameters

```bash
ros2 launch ethobot_algorithms pso_demo.launch.py \
    goal_x:=8.0 \
    goal_y:=8.0 \
    population_size:=50 \
    max_iterations:=200
```

### Test with TurtleBot3 (Fake Node)

Test the PSO-to-robot integration without Gazebo:

**Terminal 1 - TurtleBot3 Fake Node:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
```

**Terminal 2 - PSO Optimization:**
```bash
source install/setup.zsh
ros2 run ethobot_algorithms pso_path_planning_node --ros-args \
    -p goal_x:=2.0 -p goal_y:=2.0 -p max_iterations:=50
```

**Terminal 3 - Waypoint Follower:**
```bash
source install/setup.zsh
ros2 run ethobot_robots waypoint_follower_node
```

**Terminal 4 - Monitor:**
```bash
ros2 topic echo /odom --field pose.pose.position
```

The robot will navigate to the PSO's optimal solution.

## Documentation

| Document | Description |
|----------|-------------|
| [PLAN.md](docs/PLAN.md) | Implementation roadmap and milestones |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | System design and component overview |
| [CONTRIBUTING.md](docs/CONTRIBUTING.md) | How to contribute to the project |
| [CLAUDE.md](CLAUDE.md) | Development guide and conventions |

## Custom Interfaces

### Messages

| Message | Purpose |
|---------|---------|
| `ParticleState` | Single particle in PSO swarm |
| `SwarmState` | Entire swarm state for visualization |
| `RobotState` | Unified robot state (ground/aerial) |
| `AlgorithmStatus` | Optimization progress and status |

### Services

| Service | Purpose |
|---------|---------|
| `SpawnRobot` | Spawn robot in simulation |
| `SetAlgorithmParams` | Modify algorithm parameters at runtime |
| `GetMetrics` | Retrieve performance metrics |

### Actions

| Action | Purpose |
|--------|---------|
| `RunOptimization` | Execute optimization with progress feedback |

## References

### Bio-Inspired Algorithms
- Kennedy, J., & Eberhart, R. (1995). Particle Swarm Optimization. *IEEE International Conference on Neural Networks*.
- Dorigo, M., & Stützle, T. (2004). *Ant Colony Optimization*. MIT Press.
- Holland, J. H. (1975). *Adaptation in Natural and Artificial Systems*. University of Michigan Press.

### Libraries
- [Pagmo2](https://esa.github.io/pagmo2/) — ESA's parallel optimization library
- [ROS2](https://docs.ros.org/) — Robot Operating System
- [Gazebo](https://gazebosim.org/) — Robot simulation

## License

MIT License — see [LICENSE](LICENSE) for details.

## Author

Fadi Labib <github@fadilabib.com>
