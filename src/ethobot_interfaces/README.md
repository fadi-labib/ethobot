# ethobot_interfaces

ROS2 interface definitions for the Ethobot bio-inspired robotics framework.

## Messages

### ParticleState.msg
Represents a single particle in a swarm optimization algorithm.

| Field | Type | Description |
|-------|------|-------------|
| `position` | geometry_msgs/Point | Current position in search space |
| `velocity` | geometry_msgs/Vector3 | Current velocity |
| `fitness` | float64 | Fitness value (lower is better for minimization) |
| `personal_best` | geometry_msgs/Point | Best position found by this particle |

### SwarmState.msg
Complete state of a particle swarm for visualization.

| Field | Type | Description |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp and frame |
| `particles` | ParticleState[] | All particles in the swarm |
| `global_best` | geometry_msgs/Point | Best position found by any particle |
| `global_best_fitness` | float64 | Fitness at global best |
| `iteration` | uint32 | Current iteration |
| `max_iterations` | uint32 | Maximum iterations |

### AlgorithmStatus.msg
Status and progress of an optimization algorithm.

| Field | Type | Description |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp |
| `algorithm_name` | string | Name of the algorithm (e.g., "PSO") |
| `status` | string | Current status (RUNNING, CONVERGED, etc.) |
| `current_iteration` | uint32 | Current iteration number |
| `max_iterations` | uint32 | Maximum iterations |
| `best_fitness` | float64 | Current best fitness value |
| `convergence_rate` | float64 | Rate of fitness improvement |

### RobotState.msg
Unified robot state for ground and aerial robots.

| Field | Type | Description |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp and frame |
| `robot_id` | string | Unique robot identifier |
| `pose` | geometry_msgs/Pose | Position and orientation |
| `twist` | geometry_msgs/Twist | Linear and angular velocity |
| `robot_type` | string | Type: "ground" or "aerial" |
| `battery_level` | float32 | Battery percentage (0-100) |

### SwarmCommand.msg
Command to control a robot swarm.

| Field | Type | Description |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp |
| `command_type` | string | START, STOP, PAUSE, RESUME |
| `target_position` | geometry_msgs/Point | Goal position |
| `formation_type` | string | Formation: "none", "line", "circle" |
| `parameters` | float64[] | Additional parameters |

## Services

### SpawnRobot.srv
Spawn a robot in simulation.

**Request:**
- `robot_namespace` (string): Unique namespace for the robot
- `robot_type` (string): "turtlebot3", "quadrotor", etc.
- `initial_pose` (geometry_msgs/Pose): Starting pose

**Response:**
- `success` (bool): Whether spawn succeeded
- `message` (string): Status or error message

### SetAlgorithmParams.srv
Modify algorithm parameters at runtime.

**Request:**
- `algorithm_name` (string): Target algorithm
- `param_names` (string[]): Parameter names
- `param_values` (float64[]): New values

**Response:**
- `success` (bool): Whether update succeeded
- `message` (string): Status message

### GetMetrics.srv
Retrieve performance metrics.

**Request:**
- `metric_names` (string[]): Requested metrics

**Response:**
- `metric_names` (string[]): Returned metric names
- `metric_values` (float64[]): Metric values
- `timestamp` (builtin_interfaces/Time): When metrics were collected

## Actions

### RunOptimization.action
Execute an optimization with progress feedback.

**Goal:**
- `problem_id` (string): Problem identifier
- `max_iterations` (uint32): Maximum iterations
- `target_fitness` (float64): Stop when fitness reaches this

**Result:**
- `success` (bool): Whether optimization succeeded
- `final_fitness` (float64): Best fitness achieved
- `best_solution` (float64[]): Optimal solution vector
- `iterations_completed` (uint32): Actual iterations run

**Feedback:**
- `current_iteration` (uint32): Current iteration
- `current_fitness` (float64): Current best fitness
- `progress_percentage` (float32): Completion percentage

## Usage

```cpp
#include "ethobot_interfaces/msg/swarm_state.hpp"
#include "ethobot_interfaces/msg/algorithm_status.hpp"
#include "ethobot_interfaces/srv/spawn_robot.hpp"
#include "ethobot_interfaces/action/run_optimization.hpp"
```

## Build

```bash
colcon build --packages-select ethobot_interfaces
```
