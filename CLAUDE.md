# Ethobot Development Guide

Development conventions and quick reference for the Ethobot project.

## Quick Commands

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
cd /home/fadi/projects/biorobot
colcon build --symlink-install
source install/setup.bash

# Build single package
colcon build --packages-select ethobot_interfaces

# Clean build
rm -rf build/ install/ log/

# Test
colcon test
colcon test-result --verbose

# Lint
ament_cpplint src/
ament_uncrustify src/
```

## Current Project Structure

```
ethobot/
├── src/
│   └── ethobot_interfaces/    # Messages, services, actions (ACTIVE)
├── config/                    # YAML parameters
├── launch/                    # Launch files
├── worlds/                    # Gazebo worlds
├── models/                    # Robot URDF/SDF
├── docs/                      # Documentation
├── scripts/                   # Utilities
├── CLAUDE.md                  # This file
├── README.md                  # Project overview
└── LICENSE                    # MIT
```

## Coding Standards

### C++ Style

- **Standard**: C++17
- **Build**: ament_cmake
- **Formatting**: ROS2 style (ament_uncrustify)
- **Linting**: ament_cpplint

```cpp
// Header guard style
#ifndef ETHOBOT_PACKAGE__CLASS_NAME_HPP_
#define ETHOBOT_PACKAGE__CLASS_NAME_HPP_

namespace ethobot_package
{

class ClassName
{
public:
  explicit ClassName(int param);

  void public_method();

private:
  int member_variable_;

  void private_method();
};

}  // namespace ethobot_package

#endif  // ETHOBOT_PACKAGE__CLASS_NAME_HPP_
```

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Package | snake_case | `ethobot_algorithms` |
| Class | PascalCase | `PsoOptimizer` |
| Function | snake_case | `calculate_fitness()` |
| Variable | snake_case | `particle_count` |
| Member | trailing underscore | `velocity_` |
| Constant | UPPER_SNAKE | `MAX_ITERATIONS` |
| Topic | snake_case | `/ethobot/swarm_state` |
| Parameter | snake_case | `max_iterations` |

### File Naming

| Type | Convention | Example |
|------|------------|---------|
| Header | snake_case.hpp | `algorithm_base.hpp` |
| Source | snake_case.cpp | `algorithm_base.cpp` |
| Message | PascalCase.msg | `SwarmState.msg` |
| Service | PascalCase.srv | `SpawnRobot.srv` |
| Action | PascalCase.action | `RunOptimization.action` |

## Package Template

When creating a new package:

```
ethobot_<name>/
├── include/ethobot_<name>/
│   └── *.hpp
├── src/
│   └── *.cpp
├── test/
│   └── *_test.cpp
├── CMakeLists.txt
└── package.xml
```

### CMakeLists.txt Template

```cmake
cmake_minimum_required(VERSION 3.8)
project(ethobot_<name>)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethobot_interfaces REQUIRED)

# Add library/executable here

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ethobot_<name></name>
  <version>0.1.0</version>
  <description>Description here</description>
  <maintainer email="github@fadilabib.com">Fadi Labib</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>ethobot_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## ROS2 Interfaces

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/ethobot/swarm_state` | SwarmState | Algorithm node | Swarm visualization |
| `/ethobot/algorithm_status` | AlgorithmStatus | Algorithm node | Progress updates |
| `/ethobot/<robot>/cmd_vel` | geometry_msgs/Twist | Algorithm node | Robot velocity |
| `/ethobot/<robot>/odom` | nav_msgs/Odometry | Robot/Sim | Robot odometry |

### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/ethobot/spawn_robot` | SpawnRobot | Simulation manager | Spawn robot |
| `/ethobot/set_params` | SetAlgorithmParams | Algorithm node | Runtime tuning |

### Actions

| Action | Type | Server | Description |
|--------|------|--------|-------------|
| `/ethobot/optimize` | RunOptimization | Algorithm node | Run optimization |

## Git Workflow

### Branch Naming

```
feat/<description>    # New feature
fix/<description>     # Bug fix
docs/<description>    # Documentation
refactor/<description> # Code refactoring
```

### Commit Messages

Use conventional commits:

```
feat: add PSO algorithm implementation
fix: correct particle velocity update
docs: update installation instructions
refactor: extract fitness function interface
test: add PSO convergence tests
chore: update dependencies
```

### Commit Author

All commits must use:
```
Fadi Labib <github@fadilabib.com>
```

## Testing

### Unit Tests

```cpp
#include <gtest/gtest.h>
#include "ethobot_algorithms/pso.hpp"

TEST(PsoTest, InitializesCorrectly)
{
  // Test implementation
}
```

### Running Tests

```bash
# All tests
colcon test

# Specific package
colcon test --packages-select ethobot_algorithms

# With output
colcon test --event-handlers console_direct+

# View results
colcon test-result --verbose
```

## Debugging

### Launch with Debug

```bash
ros2 launch ethobot path_planning.launch.py --debug
```

### GDB

```bash
ros2 run --prefix 'gdb -ex run --args' ethobot_algorithms pso_node
```

### Logging

```cpp
RCLCPP_DEBUG(this->get_logger(), "Debug message");
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error message");
```

## Dependencies

```bash
# Source ROS2 first
source /opt/ros/jazzy/setup.bash

# Install all dependencies
sudo apt install \
    libpagmo-dev \
    libeigen3-dev \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz
```

| Package | Purpose |
|---------|---------|
| libpagmo-dev | Optimization algorithms (PSO, ACO, GA) |
| libeigen3-dev | Linear algebra |
| ros-jazzy-turtlebot3 | Ground robot simulation |
| ros-jazzy-turtlebot3-msgs | TurtleBot3 message types |
| ros-jazzy-nav2-bringup | Navigation stack |
| ros-jazzy-ros-gz | Gazebo Sim integration |

## Documentation

- [PLAN.md](docs/PLAN.md) — Implementation roadmap
- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — System design
- [CONTRIBUTING.md](docs/CONTRIBUTING.md) — Contribution guidelines
