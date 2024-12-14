# ethobot_robots

Robot interfaces for the Ethobot bio-inspired robotics framework. Currently supports TurtleBot3 differential drive robots.

## Overview

This package provides:
- **RobotController**: Low-level velocity control with waypoint navigation
- **WaypointFollowerNode**: Subscribes to PSO swarm state and navigates to global best

## Classes

### RobotController

Proportional controller for differential drive robots.

```cpp
#include "ethobot_robots/robot_controller.hpp"

// Create controller attached to a ROS node
auto controller = std::make_unique<ethobot_robots::RobotController>(node, "");

// Configure parameters
ethobot_robots::ControlParams params;
params.max_linear_velocity = 0.22;   // m/s
params.max_angular_velocity = 2.0;   // rad/s
params.goal_tolerance = 0.15;        // meters
controller->set_params(params);

// Set goal and update in timer callback
controller->set_goal(2.0, 2.0);
controller->update();  // Call repeatedly

// Check status
if (controller->goal_reached()) {
  // Arrived at destination
}
```

### Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_velocity` | 0.22 | Maximum forward speed (m/s) |
| `max_angular_velocity` | 2.84 | Maximum rotation speed (rad/s) |
| `goal_tolerance` | 0.1 | Distance to consider goal reached (m) |
| `angle_tolerance` | 0.1 | Angle error before moving forward (rad) |
| `kp_linear` | 0.5 | Proportional gain for linear velocity |
| `kp_angular` | 1.0 | Proportional gain for angular velocity |

## Nodes

### waypoint_follower_node

Connects PSO optimization to robot navigation.

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/ethobot/swarm_state` | SwarmState | PSO swarm state with global best |
| `/odom` | Odometry | Robot odometry |

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_namespace` | "" | Namespace for multi-robot |
| `control_rate_hz` | 10.0 | Control loop frequency |
| `goal_tolerance` | 0.15 | Goal reached threshold (m) |
| `max_linear_velocity` | 0.22 | Max forward speed (m/s) |
| `max_angular_velocity` | 2.0 | Max rotation speed (rad/s) |

## Testing with TurtleBot3 Fake Node

The fake node simulates TurtleBot3 without Gazebo - useful for algorithm testing.

### Install Dependencies

```bash
sudo apt install ros-jazzy-turtlebot3-fake-node ros-jazzy-turtlebot3-gazebo
```

### Test Step by Step

**Terminal 1 - Build:**
```bash
cd ~/projects/biorobot
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select ethobot_interfaces ethobot_core ethobot_algorithms ethobot_robots
source install/setup.zsh
```

**Terminal 2 - Launch TurtleBot3 Fake Node:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
```

**Terminal 3 - Launch PSO:**
```bash
cd ~/projects/biorobot
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 run ethobot_algorithms pso_path_planning_node --ros-args \
    -p goal_x:=2.0 -p goal_y:=2.0 -p max_iterations:=50
```

**Terminal 4 - Launch Waypoint Follower:**
```bash
cd ~/projects/biorobot
source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 run ethobot_robots waypoint_follower_node
```

**Terminal 5 - Monitor:**
```bash
# Watch robot position
ros2 topic echo /odom --field pose.pose.position

# Or see velocity commands
ros2 topic echo /cmd_vel
```

### What Happens

1. PSO optimizes to find goal position (2.0, 2.0)
2. PSO publishes `swarm_state` with `global_best`
3. Waypoint follower receives `global_best` as navigation goal
4. Robot controller computes velocity commands
5. Fake node updates odometry based on commands
6. Robot converges to PSO's optimal solution

## Fake Node vs Gazebo

| Feature | Fake Node | Gazebo |
|---------|-----------|--------|
| Odometry | Simulated from cmd_vel | Physics-based |
| Collision | No | Yes |
| Sensors (LIDAR) | No | Yes |
| Speed | Fast | Slower |
| Use case | Algorithm testing | Full simulation |

## Dependencies

- rclcpp
- ethobot_interfaces
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2, tf2_ros, tf2_geometry_msgs

## Build

```bash
colcon build --packages-select ethobot_robots
```
