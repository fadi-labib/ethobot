# ethobot_simulation

Gazebo simulation environment for the Ethobot bio-inspired robotics framework.

## Overview

This package provides:
- Custom Gazebo world with obstacles matching the PSO demo
- Launch files for integrated PSO + TurtleBot3 simulation
- ROS-Gazebo bridge configuration

## World Description

The `ethobot_world.sdf` contains:

| Object | Position | Size | Color |
|--------|----------|------|-------|
| Ground | (0, 0) | 20x20 | Gray |
| Obstacle 1 | (3, 3) | r=1.5 | Red |
| Obstacle 2 | (7, 4) | r=1.0 | Green |
| Obstacle 3 | (5, 7) | r=1.2 | Blue |
| Obstacle 4 | (2, 8) | r=0.8 | Yellow |
| Goal | (10, 10) | r=0.3 | Green (emissive) |

These obstacles match the PSO path planning demo.

## Launch Files

### gazebo_pso_demo.launch.py

Full simulation with custom world:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch ethobot_simulation gazebo_pso_demo.launch.py
```

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `goal_x` | 10.0 | Goal X position |
| `goal_y` | 10.0 | Goal Y position |
| `robot_x` | 0.0 | Robot start X |
| `robot_y` | 0.0 | Robot start Y |
| `use_sim_time` | true | Use simulation time |

**What it launches:**
- Gazebo with custom world
- TurtleBot3 robot
- ROS-Gazebo bridge (cmd_vel, odom, scan, clock)
- PSO path planning node
- Swarm visualizer node
- Waypoint follower node

### turtlebot3_pso.launch.py

Simpler launch using TurtleBot3's empty world:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch ethobot_simulation turtlebot3_pso.launch.py goal_x:=2.0 goal_y:=2.0
```

Use this if the custom world has issues.

## How It Works

```
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo Simulation                       │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────┐ │
│  │  TurtleBot3 │  │  Obstacles   │  │   Goal Marker       │ │
│  └──────┬──────┘  └──────────────┘  └─────────────────────┘ │
│         │                                                    │
└─────────┼────────────────────────────────────────────────────┘
          │ ROS-Gazebo Bridge
          ▼
┌─────────────────────────────────────────────────────────────┐
│                         ROS2                                 │
│                                                              │
│  ┌──────────────────┐    ┌────────────────────────────────┐ │
│  │  PSO Path        │───▶│  /ethobot/swarm_state          │ │
│  │  Planning Node   │    │  (global_best position)         │ │
│  └──────────────────┘    └───────────────┬────────────────┘ │
│                                          │                   │
│                                          ▼                   │
│  ┌──────────────────┐    ┌────────────────────────────────┐ │
│  │  Waypoint        │◀───│  Subscribes to swarm_state     │ │
│  │  Follower Node   │    │  Sets goal from global_best    │ │
│  └────────┬─────────┘    └────────────────────────────────┘ │
│           │                                                  │
│           ▼                                                  │
│  ┌────────────────────┐                                     │
│  │  /cmd_vel          │───▶ Robot moves toward goal         │
│  └────────────────────┘                                     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## Topics

### Published by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | Odometry | Robot odometry from Gazebo |
| `/scan` | LaserScan | LIDAR data |
| `/clock` | Clock | Simulation time |

### Subscribed by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |

## Dependencies

- ros_gz_sim
- ros_gz_bridge
- turtlebot3_gazebo
- turtlebot3_description
- ethobot_algorithms
- ethobot_robots

## Installation

```bash
sudo apt install \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-description
```

## Build

```bash
colcon build --packages-select ethobot_simulation
```

## Troubleshooting

### Gazebo doesn't start
Check if Gazebo Sim is installed:
```bash
gz sim --version
```

### Robot doesn't move
1. Check if topics are bridged: `ros2 topic list`
2. Verify cmd_vel is published: `ros2 topic echo /cmd_vel`
3. Check simulation time: `ros2 param get /waypoint_follower use_sim_time`

### PSO goal doesn't match world obstacles
The obstacles in `ethobot_world.sdf` match those in `pso_path_planning_node.cpp`. If you modify one, update the other.
