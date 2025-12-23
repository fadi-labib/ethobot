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
 * @brief PSO Waypoint Optimization for Path Planning
 *
 * PSO finds the OPTIMAL WAYPOINT that minimizes:
 *   total_path_length = dist(start, waypoint) + dist(waypoint, goal)
 *   + obstacle penalties for waypoint proximity
 *
 * This is useful when obstacles block the direct path.
 * The robot will: start → waypoint → goal
 */
class PsoPathPlanningNode : public rclcpp::Node
{
public:
  PsoPathPlanningNode()
  : Node("pso_path_planning")
  {
    // Declare configurable parameters
    this->declare_parameter("start_x", 0.0);
    this->declare_parameter("start_y", 0.0);
    this->declare_parameter("goal_x", 3.0);
    this->declare_parameter("goal_y", 3.0);
    this->declare_parameter("population_size", 30);
    this->declare_parameter("max_iterations", 50);
    this->declare_parameter("publish_rate_hz", 5.0);
    this->declare_parameter("search_space_min", 0.0);
    this->declare_parameter("search_space_max", 4.0);

    // Get parameters
    start_x_ = this->get_parameter("start_x").as_double();
    start_y_ = this->get_parameter("start_y").as_double();
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    int pop_size = this->get_parameter("population_size").as_int();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    double rate = this->get_parameter("publish_rate_hz").as_double();
    double space_min = this->get_parameter("search_space_min").as_double();
    double space_max = this->get_parameter("search_space_max").as_double();

    // Create publishers
    swarm_state_pub_ = this->create_publisher<ethobot_interfaces::msg::SwarmState>(
      "ethobot/swarm_state", 10);
    algorithm_status_pub_ = this->create_publisher<ethobot_interfaces::msg::AlgorithmStatus>(
      "ethobot/algorithm_status", 10);

    // Setup obstacles (circular obstacles)
    // Format: {x, y, radius}
    obstacles_ = {
      {1.0, 1.0, 0.3},
      {2.0, 1.2, 0.25},
      {1.5, 2.2, 0.28},
      {0.6, 2.5, 0.2}
    };

    // Check if direct path is blocked
    direct_path_length_ = std::sqrt(
      std::pow(goal_x_ - start_x_, 2) + std::pow(goal_y_ - start_y_, 2));

    // Setup PSO
    ethobot_algorithms::PsoParams params;
    params.population_size = static_cast<std::size_t>(pop_size);
    params.inertia_weight = 0.7;
    params.cognitive_coeff = 1.5;
    params.social_coeff = 1.5;
    params.max_velocity = 0.3;

    pso_ = std::make_unique<ethobot_algorithms::PsoSolver>(params);

    // Define the optimization problem - find optimal WAYPOINT
    ethobot_core::Problem problem;
    problem.dimensions = 2;  // waypoint (x, y)
    problem.lower_bounds = {space_min, space_min};
    problem.upper_bounds = {space_max, space_max};
    problem.minimize = true;

    // Fitness function: optimize waypoint for shortest safe path
    problem.fitness_function = [this](const std::vector<double> & waypoint) {
        return this->compute_waypoint_fitness(waypoint);
      };

    pso_->initialize(problem);

    RCLCPP_INFO(this->get_logger(), "=== PSO Waypoint Optimization ===");
    RCLCPP_INFO(this->get_logger(), "Start: (%.1f, %.1f)", start_x_, start_y_);
    RCLCPP_INFO(this->get_logger(), "Goal: (%.1f, %.1f)", goal_x_, goal_y_);
    RCLCPP_INFO(this->get_logger(), "Direct path length: %.2f", direct_path_length_);
    RCLCPP_INFO(this->get_logger(), "Obstacles: %zu", obstacles_.size());
    RCLCPP_INFO(this->get_logger(), "PSO will find optimal waypoint to avoid obstacles");

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
      // Optimization complete - publish final state and stop
      if (!completed_) {
        completed_ = true;
        auto waypoint = pso_->get_best_solution();
        double fitness = pso_->get_best_fitness();

        RCLCPP_INFO(this->get_logger(), "=== OPTIMIZATION COMPLETE ===");
        RCLCPP_INFO(this->get_logger(), "Optimal waypoint: (%.3f, %.3f)", waypoint[0], waypoint[1]);
        RCLCPP_INFO(this->get_logger(), "Path fitness: %.4f", fitness);
        RCLCPP_INFO(this->get_logger(), "Route: (%.1f,%.1f) → (%.2f,%.2f) → (%.1f,%.1f)",
          start_x_, start_y_, waypoint[0], waypoint[1], goal_x_, goal_y_);

        // Publish FINAL swarm state
        auto swarm_state = pso_->get_swarm_state();
        swarm_state.header.frame_id = "map";
        swarm_state.header.stamp = this->now();
        swarm_state.iteration = static_cast<uint32_t>(max_iterations_);
        swarm_state.max_iterations = static_cast<uint32_t>(max_iterations_);
        swarm_state_pub_->publish(swarm_state);
      }
      return;
    }

    // Execute one PSO step
    pso_->step();
    iteration_++;

    // Publish swarm state
    auto swarm_state = pso_->get_swarm_state();
    swarm_state.header.frame_id = "map";
    swarm_state.header.stamp = this->now();
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
        "Iteration %zu/%d: Waypoint (%.2f, %.2f), Fitness: %.4f",
        iteration_, max_iterations_, best[0], best[1], pso_->get_best_fitness());
    }
  }

  /**
   * @brief Compute fitness for a waypoint
   *
   * Fitness = path_length + obstacle_penalty
   *
   * path_length = dist(start, waypoint) + dist(waypoint, goal)
   * obstacle_penalty = high if waypoint is near/inside obstacles
   *
   * Lower fitness = better waypoint
   */
  double compute_waypoint_fitness(const std::vector<double> & waypoint)
  {
    double wx = waypoint[0];
    double wy = waypoint[1];

    // Path length through waypoint
    double dist_start_to_wp = std::sqrt(
      std::pow(wx - start_x_, 2) + std::pow(wy - start_y_, 2));
    double dist_wp_to_goal = std::sqrt(
      std::pow(goal_x_ - wx, 2) + std::pow(goal_y_ - wy, 2));
    double total_path_length = dist_start_to_wp + dist_wp_to_goal;

    // Obstacle penalty for waypoint
    double obstacle_penalty = 0.0;
    for (const auto & obs : obstacles_) {
      double ox = obs[0];
      double oy = obs[1];
      double radius = obs[2];

      double dist_to_obs = std::sqrt(std::pow(wx - ox, 2) + std::pow(wy - oy, 2));

      if (dist_to_obs < radius) {
        // Inside obstacle - very heavy penalty
        obstacle_penalty += 100.0;
      } else if (dist_to_obs < radius + 0.3) {
        // Close to obstacle - penalty (robot needs clearance)
        obstacle_penalty += 30.0 * (radius + 0.3 - dist_to_obs);
      }

      // Also penalize if path segments cross near obstacles
      obstacle_penalty += check_path_segment_penalty(start_x_, start_y_, wx, wy, obs);
      obstacle_penalty += check_path_segment_penalty(wx, wy, goal_x_, goal_y_, obs);
    }

    return total_path_length + obstacle_penalty;
  }

  /**
   * @brief Check if a path segment passes too close to an obstacle
   */
  double check_path_segment_penalty(double x1, double y1, double x2, double y2,
                                     const std::vector<double> & obs)
  {
    double ox = obs[0];
    double oy = obs[1];
    double radius = obs[2];

    // Calculate closest point on line segment to obstacle center
    double dx = x2 - x1;
    double dy = y2 - y1;
    double seg_len_sq = dx * dx + dy * dy;

    if (seg_len_sq < 0.0001) return 0.0;  // Degenerate segment

    // Parameter t for closest point on line
    double t = std::max(0.0, std::min(1.0,
      ((ox - x1) * dx + (oy - y1) * dy) / seg_len_sq));

    // Closest point on segment
    double closest_x = x1 + t * dx;
    double closest_y = y1 + t * dy;

    // Distance from obstacle center to closest point
    double dist = std::sqrt(std::pow(closest_x - ox, 2) + std::pow(closest_y - oy, 2));

    // Penalty if path passes through or near obstacle
    double clearance = 0.2;  // Robot radius + safety margin
    if (dist < radius + clearance) {
      return 50.0 * (radius + clearance - dist);
    }

    return 0.0;
  }

  // Publishers
  rclcpp::Publisher<ethobot_interfaces::msg::SwarmState>::SharedPtr swarm_state_pub_;
  rclcpp::Publisher<ethobot_interfaces::msg::AlgorithmStatus>::SharedPtr algorithm_status_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // PSO solver
  std::unique_ptr<ethobot_algorithms::PsoSolver> pso_;

  // Configurable parameters
  double start_x_;
  double start_y_;
  double goal_x_;
  double goal_y_;
  int max_iterations_;
  double direct_path_length_;

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
