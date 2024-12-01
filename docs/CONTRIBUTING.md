# Contributing to Ethobot

Thank you for your interest in contributing to Ethobot! This document provides guidelines for contributing to the project.

## Code of Conduct

Be respectful and constructive. Focus on the code and ideas, not the person.

## Getting Started

### Prerequisites

```bash
# Ubuntu 24.04 with ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

sudo apt install \
    ros-jazzy-desktop \
    ros-dev-tools \
    libpagmo-dev \
    libeigen3-dev \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz
```

### Fork and Clone

```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/ethobot.git
cd ethobot
git remote add upstream https://github.com/ORIGINAL_OWNER/ethobot.git
```

### Build and Test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## How to Contribute

### Reporting Bugs

1. Check existing issues first
2. Use the bug report template
3. Include:
   - ROS2 version
   - Ubuntu version
   - Steps to reproduce
   - Expected vs actual behavior
   - Error messages/logs

### Suggesting Features

1. Check existing issues/discussions
2. Describe the use case
3. Explain why it benefits the project
4. Consider implementation approach

### Submitting Code

1. Create a feature branch
2. Make changes
3. Write/update tests
4. Update documentation
5. Submit pull request

## Development Workflow

### Branch Strategy

```
main              # Stable, release-ready
├── feat/xxx      # New features
├── fix/xxx       # Bug fixes
├── docs/xxx      # Documentation
└── refactor/xxx  # Code improvements
```

### Creating a Branch

```bash
# Sync with upstream
git fetch upstream
git checkout main
git merge upstream/main

# Create feature branch
git checkout -b feat/my-feature
```

### Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>: <description>

[optional body]

[optional footer]
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Formatting, no code change
- `refactor`: Code restructuring
- `test`: Adding tests
- `chore`: Maintenance

**Examples**:
```
feat: add PSO algorithm implementation

Implements Particle Swarm Optimization using Pagmo2.
Includes ROS2 node wrapper and parameter configuration.

Closes #123
```

```
fix: correct particle velocity bounds checking

Velocity was not being clamped to max_velocity parameter.
```

### Pull Request Process

1. **Before submitting**:
   ```bash
   # Ensure tests pass
   colcon test
   colcon test-result --verbose

   # Check formatting
   ament_uncrustify src/
   ament_cpplint src/
   ```

2. **PR title**: Use conventional commit format

3. **PR description**:
   - What changes were made
   - Why the changes were made
   - How to test
   - Related issues

4. **Review process**:
   - Address feedback promptly
   - Keep commits atomic
   - Rebase if needed

## Coding Standards

### C++ Style

Follow the [ROS2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

**Key points**:
- C++17 standard
- 2-space indentation
- 100 character line limit
- Use `snake_case` for functions/variables
- Use `PascalCase` for classes
- Use `UPPER_SNAKE_CASE` for constants

**Header guards**:
```cpp
#ifndef ETHOBOT_PACKAGE__FILE_NAME_HPP_
#define ETHOBOT_PACKAGE__FILE_NAME_HPP_

// ...

#endif  // ETHOBOT_PACKAGE__FILE_NAME_HPP_
```

**Includes order**:
```cpp
// 1. Related header
#include "ethobot_algorithms/pso.hpp"

// 2. C system headers
#include <cmath>

// 3. C++ standard library
#include <memory>
#include <vector>

// 4. Other libraries
#include <Eigen/Dense>

// 5. ROS2 headers
#include "rclcpp/rclcpp.hpp"

// 6. Project headers
#include "ethobot_core/algorithm_base.hpp"
```

### Documentation

**Header files**: Document all public APIs

```cpp
/**
 * @brief Particle Swarm Optimization algorithm
 *
 * Implements the standard PSO algorithm with configurable
 * inertia weight and acceleration coefficients.
 */
class PsoAlgorithm : public AlgorithmBase
{
public:
  /**
   * @brief Construct PSO algorithm
   * @param node ROS2 node for logging and parameters
   * @param config Algorithm configuration
   */
  PsoAlgorithm(rclcpp::Node::SharedPtr node, const PsoConfig& config);

  /**
   * @brief Execute one iteration of the algorithm
   * @return true if converged, false otherwise
   */
  bool step() override;
};
```

**Source files**: Comment complex logic only

```cpp
// Use weighted sum of personal and global best
// v = w*v + c1*r1*(pbest-x) + c2*r2*(gbest-x)
velocity = inertia_weight_ * velocity
         + cognitive_coeff_ * r1 * (personal_best - position)
         + social_coeff_ * r2 * (global_best - position);
```

### Testing

**Unit tests required for**:
- All public class methods
- Algorithm correctness
- Edge cases

**Test file naming**: `<source_file>_test.cpp`

**Test structure**:
```cpp
#include <gtest/gtest.h>
#include "ethobot_algorithms/pso.hpp"

class PsoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup code
  }
};

TEST_F(PsoTest, ConvergesOnSimpleProblem)
{
  // Test implementation
  EXPECT_LT(result.best_fitness, 0.01);
}

TEST_F(PsoTest, RespectsMaxIterations)
{
  // Test implementation
  EXPECT_LE(result.iterations, max_iterations);
}
```

## Package Guidelines

### Creating a New Package

1. Use the templates in CLAUDE.md
2. Follow naming convention: `ethobot_<name>`
3. Include:
   - `package.xml` with all dependencies
   - `CMakeLists.txt` with proper exports
   - `README.md` with package description
   - Unit tests

### Dependencies

- Minimize external dependencies
- Use ROS2 packages when available
- Document all dependencies in package.xml

## Review Checklist

Before requesting review, ensure:

- [ ] Code compiles without warnings
- [ ] All tests pass
- [ ] New code has tests
- [ ] Documentation updated
- [ ] Follows coding standards
- [ ] Commit messages are clear
- [ ] No unrelated changes

## Getting Help

- Check existing documentation
- Search closed issues
- Open a discussion for questions
- Tag maintainers if blocked

## Recognition

Contributors will be acknowledged in:
- Release notes
- README.md (for significant contributions)

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
