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
 * Receives the global best position from PSO swarm state
 * and navigates the robot to that position.
 *
 * This demonstrates connecting bio-inspired optimization
 * to real robot control.
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
    this->declare_parameter("goal_tolerance", 0.15);
    this->declare_parameter("max_linear_velocity", 0.22);
    this->declare_parameter("max_angular_velocity", 2.0);

    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    control_rate_ = this->get_parameter("control_rate_hz").as_double();

    // Store params for later initialization
    params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params_.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    params_.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
  }

  /**
   * @brief Initialize after node is fully constructed
   * Must be called after make_shared to use shared_from_this
   */
  void initialize()
  {
    // Create robot controller (needs shared_from_this)
    controller_ = std::make_unique<ethobot_robots::RobotController>(
      shared_from_this(), robot_namespace_);
    controller_->set_params(params_);

    // Subscribe to swarm state to get global best
    swarm_sub_ = this->create_subscription<ethobot_interfaces::msg::SwarmState>(
      "ethobot/swarm_state", 10,
      std::bind(&WaypointFollowerNode::swarm_callback, this, _1));

    // Control timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&WaypointFollowerNode::control_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Follower started");
    RCLCPP_INFO(this->get_logger(), "Listening for swarm_state on 'ethobot/swarm_state'");
  }

private:
  void swarm_callback(const ethobot_interfaces::msg::SwarmState::SharedPtr msg)
  {
    // Update goal from global best position
    double new_x = msg->global_best.x;
    double new_y = msg->global_best.y;

    // Only update if goal changed significantly (avoid jitter)
    double dx = new_x - last_goal_x_;
    double dy = new_y - last_goal_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 0.1 || !goal_set_) {
      controller_->set_goal(new_x, new_y);
      last_goal_x_ = new_x;
      last_goal_y_ = new_y;
      goal_set_ = true;

      RCLCPP_INFO(this->get_logger(),
        "Updated goal from PSO: (%.2f, %.2f), iteration %u/%u",
        new_x, new_y, msg->iteration, msg->max_iterations);
    }

    // Check if optimization is complete
    if (msg->iteration >= msg->max_iterations && !optimization_complete_) {
      optimization_complete_ = true;
      RCLCPP_INFO(this->get_logger(),
        "PSO optimization complete! Final goal: (%.2f, %.2f)",
        new_x, new_y);
    }
  }

  void control_callback()
  {
    controller_->update();

    // Log when goal is reached
    if (controller_->goal_reached() && !reported_goal_reached_) {
      reported_goal_reached_ = true;
      auto pose = controller_->get_pose();
      RCLCPP_INFO(this->get_logger(),
        "Robot reached goal! Position: (%.2f, %.2f)",
        pose.x, pose.y);
    }

    // Reset reported flag when new goal is set
    if (!controller_->goal_reached()) {
      reported_goal_reached_ = false;
    }
  }

  std::unique_ptr<ethobot_robots::RobotController> controller_;
  rclcpp::Subscription<ethobot_interfaces::msg::SwarmState>::SharedPtr swarm_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Initialization parameters
  std::string robot_namespace_;
  double control_rate_;
  ethobot_robots::ControlParams params_;

  double last_goal_x_ = 0.0;
  double last_goal_y_ = 0.0;
  bool goal_set_ = false;
  bool optimization_complete_ = false;
  bool reported_goal_reached_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointFollowerNode>();
  node->initialize();  // Call after make_shared for shared_from_this

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
