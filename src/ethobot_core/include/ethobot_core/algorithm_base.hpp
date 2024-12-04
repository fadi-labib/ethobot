#ifndef ETHOBOT_CORE__ALGORITHM_BASE_HPP_
#define ETHOBOT_CORE__ALGORITHM_BASE_HPP_

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ethobot_interfaces/msg/swarm_state.hpp"
#include "ethobot_interfaces/msg/algorithm_status.hpp"

namespace ethobot_core
{

/**
 * @brief Optimization problem definition
 *
 * Defines the search space bounds and fitness function for optimization.
 */
struct Problem
{
  std::size_t dimensions;                    // Number of dimensions (2 for 2D path)
  std::vector<double> lower_bounds;          // Min value per dimension
  std::vector<double> upper_bounds;          // Max value per dimension
  std::function<double(const std::vector<double>&)> fitness_function;
  bool minimize = true;                      // true = lower fitness is better
};

/**
 * @brief Result of optimization
 */
struct OptimizationResult
{
  std::vector<double> best_solution;
  double best_fitness;
  std::size_t iterations;
  double elapsed_time_sec;
  bool converged;
};

/**
 * @brief Abstract base class for bio-inspired optimization algorithms
 *
 * Provides common interface for PSO, ACO, GA, etc.
 */
class AlgorithmBase
{
public:
  explicit AlgorithmBase(const std::string & name);
  virtual ~AlgorithmBase() = default;

  // Prevent copying
  AlgorithmBase(const AlgorithmBase &) = delete;
  AlgorithmBase & operator=(const AlgorithmBase &) = delete;

  /**
   * @brief Initialize the algorithm with a problem
   * @param problem Problem definition with bounds and fitness function
   */
  virtual void initialize(const Problem & problem) = 0;

  /**
   * @brief Execute one iteration of the algorithm
   */
  virtual void step() = 0;

  /**
   * @brief Run optimization until convergence or max iterations
   * @param max_iterations Maximum number of iterations
   * @param target_fitness Stop if this fitness is reached
   * @return Optimization result
   */
  virtual OptimizationResult solve(
    std::size_t max_iterations,
    double target_fitness = -std::numeric_limits<double>::infinity());

  /**
   * @brief Reset algorithm state
   */
  virtual void reset() = 0;

  /**
   * @brief Get current swarm state for visualization
   * @return SwarmState message
   */
  virtual ethobot_interfaces::msg::SwarmState get_swarm_state() const = 0;

  /**
   * @brief Get algorithm status
   * @return AlgorithmStatus message
   */
  ethobot_interfaces::msg::AlgorithmStatus get_status() const;

  // Getters
  std::string get_name() const { return name_; }
  std::size_t get_iteration() const { return iteration_; }
  double get_best_fitness() const { return best_fitness_; }
  std::vector<double> get_best_solution() const { return best_solution_; }
  bool is_running() const { return running_; }

  // Control
  void pause() { paused_ = true; }
  void resume() { paused_ = false; }
  void stop() { running_ = false; }

protected:
  std::string name_;
  Problem problem_;

  std::size_t iteration_ = 0;
  double best_fitness_ = std::numeric_limits<double>::infinity();
  std::vector<double> best_solution_;

  bool running_ = false;
  bool paused_ = false;

  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace ethobot_core

#endif  // ETHOBOT_CORE__ALGORITHM_BASE_HPP_
