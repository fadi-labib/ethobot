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


#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ethobot_robots/robot_controller.hpp"
#include "ethobot_interfaces/msg/swarm_state.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

/**
 * @brief Waypoint Follower Node
 *
 * Navigation sequence:
 *   1. Wait for PSO to complete optimization
 *   2. Navigate to PSO waypoint (global_best)
 *   3. Navigate to final goal
 *
 * This demonstrates PSO-guided path planning with obstacle avoidance.
 */
class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
  : Node("waypoint_follower")
  {
    // Parameters
    this->declare_parameter("robot_namespace", "");
    this->declare_parameter("control_rate_hz", 10.0);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("max_linear_velocity", 0.22);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("final_goal_x", 3.0);
    this->declare_parameter("final_goal_y", 3.0);
    // Robot spawn pose in the map frame. The controller works in odom (whose
    // origin coincides with the spawn pose for TurtleBot3), so we subtract
    // this from map-frame waypoints before handing them to the controller.
    // TODO(fadi): replace with a proper tf2 lookup if rotated spawns are needed.
    this->declare_parameter("spawn_x", 0.0);
    this->declare_parameter("spawn_y", 0.0);

    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    control_rate_ = this->get_parameter("control_rate_hz").as_double();
    final_goal_x_ = this->get_parameter("final_goal_x").as_double();
    final_goal_y_ = this->get_parameter("final_goal_y").as_double();
    spawn_x_ = this->get_parameter("spawn_x").as_double();
    spawn_y_ = this->get_parameter("spawn_y").as_double();

    // Store params for later initialization
    params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params_.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    params_.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
  }

  void initialize()
  {
    controller_ = std::make_unique<ethobot_robots::RobotController>(
      shared_from_this(), robot_namespace_);
    controller_->set_params(params_);

    swarm_sub_ = this->create_subscription<ethobot_interfaces::msg::SwarmState>(
      "ethobot/swarm_state", 10,
      std::bind(&WaypointFollowerNode::swarm_callback, this, _1));

    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&WaypointFollowerNode::control_callback, this));

    RCLCPP_INFO(this->get_logger(), "=== Waypoint Follower ===");
    RCLCPP_INFO(this->get_logger(), "Final goal: (%.2f, %.2f)", final_goal_x_, final_goal_y_);
    RCLCPP_INFO(this->get_logger(), "Waiting for PSO to find optimal waypoint...");
  }

private:
  void swarm_callback(const ethobot_interfaces::msg::SwarmState::SharedPtr msg)
  {
    current_iteration_ = msg->iteration;
    max_iterations_ = msg->max_iterations;

    // Store PSO waypoint
    pso_waypoint_x_ = msg->global_best.x;
    pso_waypoint_y_ = msg->global_best.y;
    pso_fitness_ = msg->global_best_fitness;

    // Log progress
    if (current_iteration_ % 10 == 0 || current_iteration_ == 1) {
      RCLCPP_INFO(this->get_logger(),
        "PSO %u/%u: waypoint=(%.2f, %.2f), fitness=%.3f",
        current_iteration_, max_iterations_,
        pso_waypoint_x_, pso_waypoint_y_, pso_fitness_);
    }

    // Check if PSO is complete
    if (current_iteration_ >= max_iterations_ && !pso_complete_) {
      pso_complete_ = true;
      RCLCPP_INFO(this->get_logger(), " ");
      RCLCPP_INFO(this->get_logger(), "=== PSO COMPLETE ===");
      RCLCPP_INFO(this->get_logger(), "Optimal waypoint: (%.2f, %.2f)",
        pso_waypoint_x_, pso_waypoint_y_);
      RCLCPP_INFO(this->get_logger(), "Route: START → (%.2f, %.2f) → GOAL (%.2f, %.2f)",
        pso_waypoint_x_, pso_waypoint_y_, final_goal_x_, final_goal_y_);
    }
  }

  void control_callback()
  {
    if (!pso_complete_) {
      return;  // Wait for PSO
    }

    // State machine for navigation
    switch (nav_state_) {
      case NavState::WAITING:
        if (waypoint_is_redundant(pso_waypoint_x_, pso_waypoint_y_,
            spawn_x_, spawn_y_, final_goal_x_, final_goal_y_,
            params_.goal_tolerance))
        {
          // Direct path is clear — skip the detour and head straight to goal.
          nav_state_ = NavState::TO_GOAL;
          controller_->set_goal(final_goal_x_ - spawn_x_, final_goal_y_ - spawn_y_);
          RCLCPP_INFO(this->get_logger(), " ");
          RCLCPP_INFO(this->get_logger(),
            ">>> Waypoint adds no value (direct path) — skipping to goal <<<");
          RCLCPP_INFO(this->get_logger(), ">>> PHASE 2: Navigating to goal (%.2f, %.2f) [map] <<<",
            final_goal_x_, final_goal_y_);
        } else {
          // Navigate to waypoint first (convert map-frame waypoint to odom).
          nav_state_ = NavState::TO_WAYPOINT;
          controller_->set_goal(pso_waypoint_x_ - spawn_x_, pso_waypoint_y_ - spawn_y_);
          RCLCPP_INFO(this->get_logger(), " ");
          RCLCPP_INFO(this->get_logger(),
            ">>> PHASE 1: Navigating to waypoint (%.2f, %.2f) [map] <<<",
            pso_waypoint_x_, pso_waypoint_y_);
        }
        break;

      case NavState::TO_WAYPOINT:
        controller_->update();
        if (controller_->goal_reached()) {
          auto pose = controller_->get_pose();
          RCLCPP_INFO(this->get_logger(), "Waypoint reached at odom (%.2f, %.2f)", pose.x, pose.y);

          // Now go to final goal (convert map-frame goal to odom).
          nav_state_ = NavState::TO_GOAL;
          controller_->set_goal(final_goal_x_ - spawn_x_, final_goal_y_ - spawn_y_);
          RCLCPP_INFO(this->get_logger(), " ");
          RCLCPP_INFO(this->get_logger(), ">>> PHASE 2: Navigating to goal (%.2f, %.2f) [map] <<<",
            final_goal_x_, final_goal_y_);
        }
        break;

      case NavState::TO_GOAL:
        controller_->update();
        if (controller_->goal_reached()) {
          nav_state_ = NavState::COMPLETE;
          auto pose = controller_->get_pose();
          RCLCPP_INFO(this->get_logger(), " ");
          RCLCPP_INFO(this->get_logger(), "=== NAVIGATION COMPLETE ===");
          RCLCPP_INFO(this->get_logger(), "Final position: (%.2f, %.2f)", pose.x, pose.y);
        }
        break;

      case NavState::COMPLETE:
        // Done - do nothing
        break;
    }
  }

  /**
   * @brief Decide whether the PSO waypoint actually adds value to the path.
   *
   * When the direct start→goal line is obstacle-free, the PSO fitness
   * (dist(start,wp) + dist(wp,goal) + penalties) has a FLAT minimum along the
   * entire line — every collinear point ties for "best". PSO then converges to
   * some arbitrary point on that line, and routing through it just adds a
   * pointless dogleg. This predicate detects that case so we can skip Phase 1.
   *
   * All coordinates are in the map frame.
   *
   * @param wx,wy  PSO waypoint
   * @param sx,sy  start (spawn) pose
   * @param gx,gy  final goal
   * @param tol    distance tolerance in meters (reuse goal_tolerance)
   * @return true if the waypoint should be skipped (navigate straight to goal)
   */
  bool waypoint_is_redundant(
    double wx, double wy, double sx, double sy,
    double gx, double gy, double tol) const
  {
    // Path-length test: a waypoint only earns its place if routing through it
    // is meaningfully longer than the straight shot. On the direct line the
    // two distances are equal (difference ~0), so we skip it.
    const double through = std::hypot(wx - sx, wy - sy) + std::hypot(gx - wx, gy - wy);
    const double direct = std::hypot(gx - sx, gy - sy);
    return (through - direct) < tol;
  }

  enum class NavState
  {
    WAITING,
    TO_WAYPOINT,
    TO_GOAL,
    COMPLETE
  };

  std::unique_ptr<ethobot_robots::RobotController> controller_;
  rclcpp::Subscription<ethobot_interfaces::msg::SwarmState>::SharedPtr swarm_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  std::string robot_namespace_;
  double control_rate_;
  ethobot_robots::ControlParams params_;
  double final_goal_x_;
  double final_goal_y_;
  double spawn_x_;
  double spawn_y_;

  // PSO state
  uint32_t current_iteration_ = 0;
  uint32_t max_iterations_ = 0;
  double pso_waypoint_x_ = 0.0;
  double pso_waypoint_y_ = 0.0;
  double pso_fitness_ = 0.0;
  bool pso_complete_ = false;

  // Navigation state
  NavState nav_state_ = NavState::WAITING;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollowerNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
