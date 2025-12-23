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
    this->declare_parameter("wait_for_completion", true);
    this->declare_parameter("final_goal_x", 3.0);
    this->declare_parameter("final_goal_y", 3.0);

    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    control_rate_ = this->get_parameter("control_rate_hz").as_double();
    wait_for_completion_ = this->get_parameter("wait_for_completion").as_bool();
    final_goal_x_ = this->get_parameter("final_goal_x").as_double();
    final_goal_y_ = this->get_parameter("final_goal_y").as_double();

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
      RCLCPP_INFO(this->get_logger(), "");
      RCLCPP_INFO(this->get_logger(), "=== PSO COMPLETE ===");
      RCLCPP_INFO(this->get_logger(), "Optimal waypoint: (%.2f, %.2f)", pso_waypoint_x_, pso_waypoint_y_);
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
        // Start navigation to waypoint
        nav_state_ = NavState::TO_WAYPOINT;
        controller_->set_goal(pso_waypoint_x_, pso_waypoint_y_);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), ">>> PHASE 1: Navigating to waypoint (%.2f, %.2f) <<<",
          pso_waypoint_x_, pso_waypoint_y_);
        break;

      case NavState::TO_WAYPOINT:
        controller_->update();
        if (controller_->goal_reached()) {
          auto pose = controller_->get_pose();
          RCLCPP_INFO(this->get_logger(), "Waypoint reached at (%.2f, %.2f)", pose.x, pose.y);

          // Now go to final goal
          nav_state_ = NavState::TO_GOAL;
          controller_->set_goal(final_goal_x_, final_goal_y_);
          RCLCPP_INFO(this->get_logger(), "");
          RCLCPP_INFO(this->get_logger(), ">>> PHASE 2: Navigating to goal (%.2f, %.2f) <<<",
            final_goal_x_, final_goal_y_);
        }
        break;

      case NavState::TO_GOAL:
        controller_->update();
        if (controller_->goal_reached()) {
          nav_state_ = NavState::COMPLETE;
          auto pose = controller_->get_pose();
          RCLCPP_INFO(this->get_logger(), "");
          RCLCPP_INFO(this->get_logger(), "=== NAVIGATION COMPLETE ===");
          RCLCPP_INFO(this->get_logger(), "Final position: (%.2f, %.2f)", pose.x, pose.y);
        }
        break;

      case NavState::COMPLETE:
        // Done - do nothing
        break;
    }
  }

  enum class NavState {
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
  bool wait_for_completion_;
  double final_goal_x_;
  double final_goal_y_;

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
