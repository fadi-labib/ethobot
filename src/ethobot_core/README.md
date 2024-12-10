# ethobot_core

Core library providing base classes and utilities for bio-inspired optimization algorithms in ROS2.

## Overview

This package provides the abstract foundation for implementing optimization algorithms like PSO, ACO, and GA. It defines:

- **Problem**: Search space definition with bounds and fitness function
- **AlgorithmBase**: Abstract class that all algorithms inherit from
- **OptimizationResult**: Standard result format

## Classes

### Problem

Defines the optimization problem to solve.

```cpp
struct Problem {
  std::size_t dimensions;       // Number of dimensions (e.g., 2 for x,y)
  std::vector<double> lower_bounds;  // Min value per dimension
  std::vector<double> upper_bounds;  // Max value per dimension
  std::function<double(const std::vector<double>&)> fitness_function;
  bool minimize = true;         // true = lower fitness is better
};
```

**Example:**
```cpp
ethobot_core::Problem problem;
problem.dimensions = 2;
problem.lower_bounds = {0.0, 0.0};
problem.upper_bounds = {10.0, 10.0};
problem.minimize = true;

// Fitness: distance to goal (5, 5)
problem.fitness_function = [](const std::vector<double>& pos) {
  double dx = pos[0] - 5.0;
  double dy = pos[1] - 5.0;
  return std::sqrt(dx*dx + dy*dy);
};
```

### AlgorithmBase

Abstract base class for optimization algorithms.

**Pure Virtual Methods (must implement):**
```cpp
void initialize(const Problem& problem);  // Setup with problem definition
void step();                              // Execute one iteration
void reset();                             // Reset to initial state
SwarmState get_swarm_state() const;       // Get current state for visualization
```

**Provided Methods:**
```cpp
OptimizationResult solve(max_iterations, target_fitness);  // Run to completion
AlgorithmStatus get_status() const;   // Get status message
std::string get_name() const;
std::size_t get_iteration() const;
double get_best_fitness() const;
std::vector<double> get_best_solution() const;
void pause();
void resume();
void stop();
```

### OptimizationResult

Result of running an optimization.

```cpp
struct OptimizationResult {
  std::vector<double> best_solution;  // Optimal position found
  double best_fitness;                // Fitness at best solution
  std::size_t iterations;             // Iterations completed
  double elapsed_time_sec;            // Time taken
  bool converged;                     // True if target reached
};
```

## Usage

### Implementing a New Algorithm

```cpp
#include "ethobot_core/algorithm_base.hpp"

class MyAlgorithm : public ethobot_core::AlgorithmBase
{
public:
  MyAlgorithm() : AlgorithmBase("MyAlgorithm") {}

  void initialize(const ethobot_core::Problem& problem) override {
    problem_ = problem;
    // Initialize your algorithm's state
  }

  void step() override {
    iteration_++;
    // Execute one iteration of your algorithm
    // Update best_fitness_ and best_solution_ if improved
  }

  void reset() override {
    iteration_ = 0;
    best_fitness_ = std::numeric_limits<double>::infinity();
    // Reset your algorithm's state
  }

  ethobot_interfaces::msg::SwarmState get_swarm_state() const override {
    ethobot_interfaces::msg::SwarmState state;
    // Populate with your algorithm's particles/agents
    return state;
  }
};
```

### Using an Algorithm

```cpp
#include "ethobot_algorithms/pso_solver.hpp"

// Create algorithm
ethobot_algorithms::PsoParams params;
params.population_size = 30;
auto pso = std::make_unique<ethobot_algorithms::PsoSolver>(params);

// Define problem
ethobot_core::Problem problem;
problem.dimensions = 2;
problem.lower_bounds = {0.0, 0.0};
problem.upper_bounds = {10.0, 10.0};
problem.fitness_function = my_fitness_function;

// Initialize and run
pso->initialize(problem);
auto result = pso->solve(100);  // 100 iterations max

std::cout << "Best: " << result.best_fitness << std::endl;
```

## Dependencies

- rclcpp
- ethobot_interfaces

## Build

```bash
colcon build --packages-select ethobot_core
```
