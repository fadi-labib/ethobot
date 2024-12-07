#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ethobot_algorithms/pso_solver.hpp"
#include "ethobot_interfaces/msg/swarm_state.hpp"
#include "ethobot_interfaces/msg/algorithm_status.hpp"

using namespace std::chrono_literals;

/**
 * @brief 2D Path Planning Demo using PSO
 *
 * Problem: Find the optimal (x, y) position that:
 *   1. Minimizes distance to goal
 *   2. Avoids circular obstacles
 *
 * This is a simplified demo. In real path planning, you'd optimize
 * a sequence of waypoints, not just a single point.
 */
class PsoPathPlanningNode : public rclcpp::Node
{
public:
  PsoPathPlanningNode()
  : Node("pso_path_planning")
  {
    // Declare parameters
    this->declare_parameter("goal_x", 10.0);
    this->declare_parameter("goal_y", 10.0);
    this->declare_parameter("population_size", 30);
    this->declare_parameter("max_iterations", 100);
    this->declare_parameter("publish_rate_hz", 10.0);

    // Get parameters
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    int pop_size = this->get_parameter("population_size").as_int();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    double rate = this->get_parameter("publish_rate_hz").as_double();

    // Create publishers
    swarm_state_pub_ = this->create_publisher<ethobot_interfaces::msg::SwarmState>(
      "ethobot/swarm_state", 10);
    algorithm_status_pub_ = this->create_publisher<ethobot_interfaces::msg::AlgorithmStatus>(
      "ethobot/algorithm_status", 10);

    // Setup obstacles (circular obstacles)
    // Format: {x, y, radius}
    obstacles_ = {
      {3.0, 3.0, 1.5},
      {7.0, 4.0, 1.0},
      {5.0, 7.0, 1.2},
      {2.0, 8.0, 0.8}
    };

    // Setup PSO
    ethobot_algorithms::PsoParams params;
    params.population_size = static_cast<std::size_t>(pop_size);
    params.inertia_weight = 0.7;
    params.cognitive_coeff = 1.5;
    params.social_coeff = 1.5;
    params.max_velocity = 0.5;

    pso_ = std::make_unique<ethobot_algorithms::PsoSolver>(params);

    // Define the optimization problem
    ethobot_core::Problem problem;
    problem.dimensions = 2;  // x, y
    problem.lower_bounds = {0.0, 0.0};
    problem.upper_bounds = {12.0, 12.0};
    problem.minimize = true;

    // Fitness function: distance to goal + penalty for obstacles
    problem.fitness_function = [this](const std::vector<double> & pos) {
        return this->compute_fitness(pos);
      };

    pso_->initialize(problem);

    RCLCPP_INFO(this->get_logger(), "PSO Path Planning initialized");
    RCLCPP_INFO(this->get_logger(), "Goal: (%.1f, %.1f)", goal_x_, goal_y_);
    RCLCPP_INFO(this->get_logger(), "Obstacles: %zu", obstacles_.size());
    RCLCPP_INFO(this->get_logger(), "Population: %d, Max iterations: %d", pop_size, max_iterations_);

    // Create timer for PSO steps
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PsoPathPlanningNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (iteration_ >= static_cast<std::size_t>(max_iterations_)) {
      // Optimization complete
      if (!completed_) {
        completed_ = true;
        auto result = pso_->get_best_solution();
        RCLCPP_INFO(this->get_logger(),
          "Optimization complete! Best position: (%.3f, %.3f), Fitness: %.4f",
          result[0], result[1], pso_->get_best_fitness());
      }
      return;
    }

    // Execute one PSO step
    pso_->step();
    iteration_++;

    // Publish swarm state
    auto swarm_state = pso_->get_swarm_state();
    swarm_state.header.frame_id = "map";
    swarm_state.iteration = static_cast<uint32_t>(iteration_);
    swarm_state.max_iterations = static_cast<uint32_t>(max_iterations_);
    swarm_state_pub_->publish(swarm_state);

    // Publish algorithm status
    auto status = pso_->get_status();
    status.header.stamp = this->now();
    status.current_iteration = static_cast<uint32_t>(iteration_);
    status.max_iterations = static_cast<uint32_t>(max_iterations_);
    algorithm_status_pub_->publish(status);

    // Log progress every 10 iterations
    if (iteration_ % 10 == 0) {
      auto best = pso_->get_best_solution();
      RCLCPP_INFO(this->get_logger(),
        "Iteration %zu/%d: Best (%.2f, %.2f), Fitness: %.4f",
        iteration_, max_iterations_, best[0], best[1], pso_->get_best_fitness());
    }
  }

  /**
   * @brief Compute fitness for a position
   *
   * Lower is better:
   *   - Distance to goal
   *   - High penalty if inside obstacle
   */
  double compute_fitness(const std::vector<double> & pos)
  {
    double x = pos[0];
    double y = pos[1];

    // Distance to goal
    double dist_to_goal = std::sqrt(
      std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));

    // Obstacle penalty
    double obstacle_penalty = 0.0;
    for (const auto & obs : obstacles_) {
      double ox = obs[0];
      double oy = obs[1];
      double radius = obs[2];

      double dist_to_obs = std::sqrt(std::pow(x - ox, 2) + std::pow(y - oy, 2));

      if (dist_to_obs < radius) {
        // Inside obstacle - very heavy penalty
        obstacle_penalty += 1000.0;
      } else if (dist_to_obs < radius + 1.0) {
        // Close to obstacle - penalty (encourages clearance)
        obstacle_penalty += 50.0 * (radius + 1.0 - dist_to_obs);
      }
    }

    return dist_to_goal + obstacle_penalty;
  }

  // Publishers
  rclcpp::Publisher<ethobot_interfaces::msg::SwarmState>::SharedPtr swarm_state_pub_;
  rclcpp::Publisher<ethobot_interfaces::msg::AlgorithmStatus>::SharedPtr algorithm_status_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // PSO solver
  std::unique_ptr<ethobot_algorithms::PsoSolver> pso_;

  // Problem definition
  double goal_x_;
  double goal_y_;
  int max_iterations_;
  std::vector<std::vector<double>> obstacles_;  // {x, y, radius}

  bool completed_ = false;
  std::size_t iteration_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PsoPathPlanningNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
