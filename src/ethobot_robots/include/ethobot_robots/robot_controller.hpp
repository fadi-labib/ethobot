#ifndef ETHOBOT_ROBOTS__ROBOT_CONTROLLER_HPP_
#define ETHOBOT_ROBOTS__ROBOT_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"

namespace ethobot_robots
{

/**
 * @brief Robot control parameters
 */
struct ControlParams
{
  double max_linear_velocity = 0.22;   // m/s (TurtleBot3 max)
  double max_angular_velocity = 2.84;  // rad/s (TurtleBot3 max)
  double goal_tolerance = 0.1;         // meters
  double angle_tolerance = 0.1;        // radians
  double kp_linear = 0.5;              // P gain for linear velocity
  double kp_angular = 1.0;             // P gain for angular velocity
};

/**
 * @brief Robot controller for differential drive robots (TurtleBot3)
 *
 * Provides simple waypoint navigation using proportional control.
 * Subscribes to odometry and publishes velocity commands.
 */
class RobotController
{
public:
  /**
   * @brief Construct controller attached to a ROS node
   * @param node The ROS node to use for pub/sub
   * @param robot_namespace Optional namespace for multi-robot
   */
  explicit RobotController(
    rclcpp::Node::SharedPtr node,
    const std::string & robot_namespace = "");

  /**
   * @brief Set control parameters
   */
  void set_params(const ControlParams & params) { params_ = params; }

  /**
   * @brief Get current control parameters
   */
  const ControlParams & get_params() const { return params_; }

  /**
   * @brief Set goal position
   * @param x Goal X coordinate
   * @param y Goal Y coordinate
   */
  void set_goal(double x, double y);

  /**
   * @brief Clear current goal and stop robot
   */
  void clear_goal();

  /**
   * @brief Check if robot has reached the goal
   */
  bool goal_reached() const { return goal_reached_; }

  /**
   * @brief Check if a goal is currently set
   */
  bool has_goal() const { return has_goal_; }

  /**
   * @brief Get current robot pose
   */
  geometry_msgs::msg::Pose2D get_pose() const { return current_pose_; }

  /**
   * @brief Get distance to current goal
   */
  double get_distance_to_goal() const;

  /**
   * @brief Compute and publish velocity command
   *
   * Call this in a timer callback to continuously control the robot.
   */
  void update();

  /**
   * @brief Stop the robot immediately
   */
  void stop();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalize_angle(double angle) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  ControlParams params_;

  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::Pose2D goal_pose_;

  bool has_goal_ = false;
  bool goal_reached_ = false;
  bool odom_received_ = false;
};

}  // namespace ethobot_robots

#endif  // ETHOBOT_ROBOTS__ROBOT_CONTROLLER_HPP_
