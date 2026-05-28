// Copyright 2026 Fadi Labib
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <algorithm>
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

    // Obstacles come from parameters (config/obstacles.yaml) as parallel arrays.
    this->declare_parameter("obstacle_x", std::vector<double>{});
    this->declare_parameter("obstacle_y", std::vector<double>{});
    this->declare_parameter("obstacle_radius", std::vector<double>{});
    obstacles_ = load_obstacles();

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
    RCLCPP_INFO(this->get_logger(), "Obstacles: %zu", obstacles_.size());
    RCLCPP_INFO(this->get_logger(), "PSO will find optimal waypoint to avoid obstacles");

    // Create timer for PSO steps
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PsoPathPlanningNode::timer_callback, this));
  }

private:
  /**
   * @brief Build the obstacle list from the obstacle_x/y/radius parameters.
   *
   * Returns {x, y, radius} triples. Returns empty (with a log message) when no
   * obstacles are configured or the parallel arrays disagree in length.
   */
  std::vector<std::vector<double>> load_obstacles()
  {
    auto xs = this->get_parameter("obstacle_x").as_double_array();
    auto ys = this->get_parameter("obstacle_y").as_double_array();
    auto rs = this->get_parameter("obstacle_radius").as_double_array();

    if (xs.size() != ys.size() || xs.size() != rs.size()) {
      RCLCPP_ERROR(this->get_logger(),
        "obstacle_x/y/radius length mismatch (%zu/%zu/%zu) — ignoring obstacles",
        xs.size(), ys.size(), rs.size());
      return {};
    }
    if (xs.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "No obstacles configured (load config/obstacles.yaml to set them)");
    }

    std::vector<std::vector<double>> result;
    result.reserve(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) {
      result.push_back({xs[i], ys[i], rs[i]});
    }
    return result;
  }

  void timer_callback()
  {
    const std::size_t current = pso_->get_iteration();

    if (current >= static_cast<std::size_t>(max_iterations_)) {
      // Optimization complete - publish final state once and stop
      if (!completed_) {
        completed_ = true;
        auto waypoint = pso_->get_best_solution();
        double fitness = pso_->get_best_fitness();

        RCLCPP_INFO(this->get_logger(), "=== OPTIMIZATION COMPLETE ===");
        RCLCPP_INFO(this->get_logger(), "Optimal waypoint: (%.3f, %.3f)", waypoint[0], waypoint[1]);
        RCLCPP_INFO(this->get_logger(), "Path fitness: %.4f", fitness);
        RCLCPP_INFO(this->get_logger(), "Route: (%.1f,%.1f) → (%.2f,%.2f) → (%.1f,%.1f)",
          start_x_, start_y_, waypoint[0], waypoint[1], goal_x_, goal_y_);

        publish_swarm_state(current);
        timer_->cancel();  // optimization done; stop the periodic callback
      }
      return;
    }

    // Execute one PSO step (solver increments its own iteration)
    pso_->step();
    const std::size_t after_step = pso_->get_iteration();

    publish_swarm_state(after_step);

    // Log progress every 10 iterations
    if (after_step % 10 == 0) {
      auto best = pso_->get_best_solution();
      RCLCPP_INFO(this->get_logger(),
        "Iteration %zu/%d: Waypoint (%.2f, %.2f), Fitness: %.4f",
        after_step, max_iterations_, best[0], best[1], pso_->get_best_fitness());
    }
  }

  void publish_swarm_state(std::size_t iteration)
  {
    auto swarm_state = pso_->get_swarm_state();
    swarm_state.header.frame_id = "map";
    swarm_state.header.stamp = this->now();
    swarm_state.iteration = static_cast<uint32_t>(iteration);
    swarm_state.max_iterations = static_cast<uint32_t>(max_iterations_);
    swarm_state_pub_->publish(swarm_state);

    auto status = pso_->get_status();
    status.header.stamp = this->now();
    status.current_iteration = static_cast<uint32_t>(iteration);
    status.max_iterations = static_cast<uint32_t>(max_iterations_);
    algorithm_status_pub_->publish(status);
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

    // Obstacle penalty. The waypoint is an endpoint of both segments, so the
    // segment checks already account for waypoint-on-obstacle — there is no
    // separate waypoint-proximity term (which would double-count it).
    double obstacle_penalty = 0.0;
    for (const auto & obs : obstacles_) {
      obstacle_penalty += check_path_segment_penalty(start_x_, start_y_, wx, wy, obs);
      obstacle_penalty += check_path_segment_penalty(wx, wy, goal_x_, goal_y_, obs);
    }

    return total_path_length + obstacle_penalty;
  }

  /**
   * @brief Penalty for a path segment passing through or near an obstacle.
   *
   * Finds the closest point on the segment to the obstacle centre, then:
   *   - dist < radius           → segment intersects obstacle (hard penalty)
   *   - dist < radius+clearance → too close, graduated penalty
   *   - otherwise               → no penalty
   *
   * A degenerate (zero-length) segment collapses to a point check via t = 0,
   * so a waypoint sitting on an obstacle is still penalised.
   */
  double check_path_segment_penalty(
    double x1, double y1, double x2, double y2,
    const std::vector<double> & obs)
  {
    double ox = obs[0];
    double oy = obs[1];
    double radius = obs[2];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double seg_len_sq = dx * dx + dy * dy;

    // Parameter t for closest point on the segment, clamped to [0, 1].
    // A degenerate (zero-length) segment collapses to the point check at t = 0.
    double t = 0.0;
    if (seg_len_sq >= 1e-9) {
      t = std::clamp(((ox - x1) * dx + (oy - y1) * dy) / seg_len_sq, 0.0, 1.0);
    }

    double closest_x = x1 + t * dx;
    double closest_y = y1 + t * dy;
    double dist = std::hypot(closest_x - ox, closest_y - oy);

    const double clearance = 0.3;  // robot radius + safety margin
    if (dist < radius) {
      return 100.0;  // segment passes through the obstacle
    }
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

  std::vector<std::vector<double>> obstacles_;  // {x, y, radius}

  bool completed_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PsoPathPlanningNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
