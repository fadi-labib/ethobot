#include "ethobot_robots/robot_controller.hpp"

namespace ethobot_robots
{

RobotController::RobotController(
  rclcpp::Node::SharedPtr node,
  const std::string & robot_namespace)
: node_(node)
{
  std::string ns = robot_namespace.empty() ? "" : "/" + robot_namespace;

  // Create publisher for velocity commands
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    ns + "/cmd_vel", 10);

  // Subscribe to odometry
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    ns + "/odom", 10,
    std::bind(&RobotController::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "RobotController initialized (namespace: '%s')",
    robot_namespace.empty() ? "default" : robot_namespace.c_str());
}

void RobotController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
    msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  current_pose_.theta = std::atan2(siny_cosp, cosy_cosp);

  odom_received_ = true;
}

void RobotController::set_goal(double x, double y)
{
  goal_pose_.x = x;
  goal_pose_.y = y;
  has_goal_ = true;
  goal_reached_ = false;

  RCLCPP_INFO(node_->get_logger(), "New goal set: (%.2f, %.2f)", x, y);
}

void RobotController::clear_goal()
{
  has_goal_ = false;
  goal_reached_ = false;
  stop();
}

double RobotController::get_distance_to_goal() const
{
  if (!has_goal_) {
    return 0.0;
  }

  double dx = goal_pose_.x - current_pose_.x;
  double dy = goal_pose_.y - current_pose_.y;
  return std::sqrt(dx * dx + dy * dy);
}

void RobotController::update()
{
  if (!odom_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Waiting for odometry...");
    return;
  }

  if (!has_goal_ || goal_reached_) {
    return;
  }

  // Calculate distance and angle to goal
  double dx = goal_pose_.x - current_pose_.x;
  double dy = goal_pose_.y - current_pose_.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double target_angle = std::atan2(dy, dx);
  double angle_error = normalize_angle(target_angle - current_pose_.theta);

  // Check if goal reached
  if (distance < params_.goal_tolerance) {
    goal_reached_ = true;
    stop();
    RCLCPP_INFO(node_->get_logger(), "Goal reached! Distance: %.3f m", distance);
    return;
  }

  // Compute velocity commands using proportional control
  geometry_msgs::msg::Twist cmd;

  // If angle error is large, rotate in place first
  if (std::abs(angle_error) > params_.angle_tolerance) {
    cmd.linear.x = 0.0;
    cmd.angular.z = params_.kp_angular * angle_error;
  } else {
    // Move toward goal
    cmd.linear.x = params_.kp_linear * distance;
    cmd.angular.z = params_.kp_angular * angle_error;
  }

  // Clamp velocities to limits
  cmd.linear.x = std::clamp(cmd.linear.x, -params_.max_linear_velocity, params_.max_linear_velocity);
  cmd.angular.z = std::clamp(cmd.angular.z, -params_.max_angular_velocity, params_.max_angular_velocity);

  cmd_vel_pub_->publish(cmd);
}

void RobotController::stop()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

double RobotController::normalize_angle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace ethobot_robots
