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


#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ethobot_robots/robot_controller.hpp"

namespace ethobot_robots
{

// =============================================================================
// Test Fixtures
// =============================================================================

class RobotControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    controller_ = std::make_unique<RobotController>(node_);
  }

  void TearDown() override
  {
    controller_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  // Helper to simulate odometry message
  void publish_odom(double x, double y, double theta)
  {
    auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;

    // Convert theta to quaternion (yaw only)
    odom.pose.pose.orientation.z = std::sin(theta / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta / 2.0);

    odom_pub->publish(odom);

    // Spin to process the message
    rclcpp::spin_some(node_);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RobotController> controller_;
};

// =============================================================================
// ControlParams Tests
// =============================================================================

TEST(ControlParamsTest, DefaultValues)
{
  ControlParams params;

  EXPECT_DOUBLE_EQ(params.max_linear_velocity, 0.22);
  EXPECT_DOUBLE_EQ(params.max_angular_velocity, 2.84);
  EXPECT_DOUBLE_EQ(params.goal_tolerance, 0.1);
  EXPECT_DOUBLE_EQ(params.angle_tolerance, 0.1);
  EXPECT_DOUBLE_EQ(params.kp_linear, 0.5);
  EXPECT_DOUBLE_EQ(params.kp_angular, 1.0);
}

TEST(ControlParamsTest, CustomValues)
{
  ControlParams params;
  params.max_linear_velocity = 0.5;
  params.kp_linear = 1.0;
  params.goal_tolerance = 0.05;

  EXPECT_DOUBLE_EQ(params.max_linear_velocity, 0.5);
  EXPECT_DOUBLE_EQ(params.kp_linear, 1.0);
  EXPECT_DOUBLE_EQ(params.goal_tolerance, 0.05);
}

// =============================================================================
// RobotController Construction Tests
// =============================================================================

TEST_F(RobotControllerTest, Construction)
{
  // Controller should initialize without errors
  EXPECT_FALSE(controller_->has_goal());
  EXPECT_FALSE(controller_->goal_reached());
}

TEST_F(RobotControllerTest, ConstructionWithNamespace)
{
  auto namespaced_controller = std::make_unique<RobotController>(node_, "robot1");

  EXPECT_FALSE(namespaced_controller->has_goal());
}

// =============================================================================
// RobotController Parameter Tests
// =============================================================================

TEST_F(RobotControllerTest, SetParams)
{
  ControlParams params;
  params.max_linear_velocity = 0.5;
  params.kp_linear = 2.0;

  controller_->set_params(params);

  EXPECT_DOUBLE_EQ(controller_->get_params().max_linear_velocity, 0.5);
  EXPECT_DOUBLE_EQ(controller_->get_params().kp_linear, 2.0);
}

// =============================================================================
// RobotController Goal Tests
// =============================================================================

TEST_F(RobotControllerTest, SetGoal)
{
  EXPECT_FALSE(controller_->has_goal());

  controller_->set_goal(5.0, 3.0);

  EXPECT_TRUE(controller_->has_goal());
  EXPECT_FALSE(controller_->goal_reached());
}

TEST_F(RobotControllerTest, ClearGoal)
{
  controller_->set_goal(5.0, 3.0);
  EXPECT_TRUE(controller_->has_goal());

  controller_->clear_goal();

  EXPECT_FALSE(controller_->has_goal());
}

TEST_F(RobotControllerTest, MultipleGoalSets)
{
  controller_->set_goal(1.0, 1.0);
  EXPECT_TRUE(controller_->has_goal());

  controller_->set_goal(2.0, 2.0);
  EXPECT_TRUE(controller_->has_goal());

  controller_->clear_goal();
  EXPECT_FALSE(controller_->has_goal());
}

// =============================================================================
// RobotController Pose Tests
// =============================================================================

TEST_F(RobotControllerTest, InitialPose)
{
  auto pose = controller_->get_pose();

  // Initial pose should be zero
  EXPECT_DOUBLE_EQ(pose.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

// =============================================================================
// RobotController Distance Tests
// =============================================================================

TEST_F(RobotControllerTest, DistanceToGoalWithoutGoal)
{
  // Without a goal set, distance should be 0 or handled gracefully
  double dist = controller_->get_distance_to_goal();
  EXPECT_GE(dist, 0.0);
}

TEST_F(RobotControllerTest, DistanceToGoal)
{
  // Set goal at (3, 4) from origin
  controller_->set_goal(3.0, 4.0);

  double dist = controller_->get_distance_to_goal();

  // Distance from (0,0) to (3,4) should be 5.0
  EXPECT_NEAR(dist, 5.0, 0.001);
}

// =============================================================================
// RobotController Stop Tests
// =============================================================================

TEST_F(RobotControllerTest, Stop)
{
  controller_->set_goal(10.0, 10.0);

  // stop() only halts motion; it does not abandon the goal (that is
  // clear_goal()'s job). update() relies on the goal surviving a stop()
  // when it halts the robot at the goal.
  controller_->stop();

  EXPECT_TRUE(controller_->has_goal());
}

// =============================================================================
// RobotController Update Tests
// =============================================================================

TEST_F(RobotControllerTest, UpdateWithoutGoal)
{
  // Update without goal should not crash
  controller_->update();

  EXPECT_FALSE(controller_->has_goal());
}

TEST_F(RobotControllerTest, UpdateWithGoal)
{
  controller_->set_goal(1.0, 0.0);

  // Update should not crash
  controller_->update();

  EXPECT_TRUE(controller_->has_goal());
}

// =============================================================================
// TODO(fadi): Students should add more tests:
// - Test goal_reached detection when robot is at goal position
// - Test velocity clamping (max linear/angular)
// - Test normalize_angle function edge cases
// - Test with simulated odometry updates
// - Test multi-robot scenarios with namespaces
// - Test proportional control gains effect on velocity
// - Integration test with mock publishers/subscribers
// =============================================================================

}  // namespace ethobot_robots

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
