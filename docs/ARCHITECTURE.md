# Ethobot Architecture

System design and component overview for the Ethobot framework.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         User Interface                               │
│                    (RViz, CLI, Launch files)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐     │
│  │   Algorithms    │  │     Robots      │  │   Simulation    │     │
│  │  ┌───────────┐  │  │  ┌───────────┐  │  │  ┌───────────┐  │     │
│  │  │    PSO    │  │  │  │  Ground   │  │  │  │  Gazebo   │  │     │
│  │  ├───────────┤  │  │  ├───────────┤  │  │  │  Manager  │  │     │
│  │  │    ACO    │  │  │  │  Aerial   │  │  │  └───────────┘  │     │
│  │  ├───────────┤  │  │  └───────────┘  │  │                 │     │
│  │  │    GA     │  │  │                 │  │                 │     │
│  │  └───────────┘  │  │                 │  │                 │     │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘     │
│           │                    │                    │               │
│  ┌────────┴────────────────────┴────────────────────┴────────┐     │
│  │                      ethobot_core                          │     │
│  │         (AlgorithmBase, RobotInterface, Utilities)         │     │
│  └────────────────────────────┬───────────────────────────────┘     │
│                               │                                      │
│  ┌────────────────────────────┴───────────────────────────────┐     │
│  │                    ethobot_interfaces                       │     │
│  │              (Messages, Services, Actions)                  │     │
│  └─────────────────────────────────────────────────────────────┘     │
│                                                                      │
├─────────────────────────────────────────────────────────────────────┤
│                         ROS2 Middleware                              │
│                    (Topics, Services, Actions)                       │
├─────────────────────────────────────────────────────────────────────┤
│                      Gazebo Simulation                               │
└─────────────────────────────────────────────────────────────────────┘
```

## Package Structure

### ethobot_interfaces

**Purpose**: Define all custom ROS2 message, service, and action types.

```
ethobot_interfaces/
├── msg/
│   ├── ParticleState.msg      # Single PSO particle
│   ├── SwarmState.msg         # Entire swarm state
│   ├── RobotState.msg         # Unified robot state
│   ├── SwarmCommand.msg       # Control commands
│   └── AlgorithmStatus.msg    # Progress updates
├── srv/
│   ├── SpawnRobot.srv         # Spawn robot in simulation
│   ├── SetAlgorithmParams.srv # Runtime parameter tuning
│   └── GetMetrics.srv         # Performance metrics
├── action/
│   └── RunOptimization.action # Long-running optimization
├── CMakeLists.txt
└── package.xml
```

**Dependencies**: std_msgs, geometry_msgs, builtin_interfaces

---

### ethobot_core (Planned)

**Purpose**: Base classes and utilities shared across packages.

```
ethobot_core/
├── include/ethobot_core/
│   ├── algorithm_base.hpp     # Abstract algorithm interface
│   ├── robot_interface.hpp    # Abstract robot interface
│   ├── problem.hpp            # Optimization problem definition
│   └── utils.hpp              # Utility functions
├── src/
│   ├── algorithm_base.cpp
│   └── utils.cpp
├── CMakeLists.txt
└── package.xml
```

**Key Classes**:

```cpp
namespace ethobot_core
{

// Optimization problem definition
struct Problem
{
  size_t dimensions;
  Eigen::VectorXd lower_bounds;
  Eigen::VectorXd upper_bounds;
  std::function<double(const Eigen::VectorXd&)> fitness_function;
  bool minimize = true;
};

// Abstract base for all algorithms
class AlgorithmBase
{
public:
  virtual void initialize(const Problem& problem) = 0;
  virtual void step() = 0;
  virtual OptimizationResult solve(size_t max_iterations) = 0;
  virtual SwarmState get_swarm_state() const = 0;
};

// Abstract base for all robots
class RobotInterface
{
public:
  virtual Pose get_pose() const = 0;
  virtual void set_velocity(const Twist& cmd) = 0;
  virtual SensorData get_sensors() const = 0;
};

}  // namespace ethobot_core
```

---

### ethobot_algorithms (Planned)

**Purpose**: Bio-inspired algorithm implementations.

```
ethobot_algorithms/
├── include/ethobot_algorithms/
│   ├── pso.hpp                # PSO implementation
│   ├── aco.hpp                # ACO implementation
│   └── ga.hpp                 # GA implementation
├── src/
│   ├── pso.cpp
│   ├── pso_node.cpp           # ROS2 node wrapper
│   └── ...
├── config/
│   └── pso_params.yaml        # Default parameters
├── test/
│   └── pso_test.cpp
├── CMakeLists.txt
└── package.xml
```

---

### ethobot_robots (Planned)

**Purpose**: Robot-specific implementations.

```
ethobot_robots/
├── include/ethobot_robots/
│   ├── ground_robot.hpp       # TurtleBot3 wrapper
│   └── aerial_robot.hpp       # Quadrotor wrapper
├── src/
│   ├── ground_robot.cpp
│   └── ground_robot_node.cpp
├── CMakeLists.txt
└── package.xml
```

---

## Data Flow

### Optimization Loop

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Problem    │────▶│  Algorithm   │────▶│   Solution   │
│  Definition  │     │  (PSO/ACO)   │     │   Output     │
└──────────────┘     └──────┬───────┘     └──────────────┘
                            │
                            ▼
                     ┌──────────────┐
                     │ SwarmState   │───▶ Visualization
                     │   Topic      │
                     └──────────────┘
```

### Robot Control Loop

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Algorithm   │────▶│    Robot     │────▶│  Simulation  │
│  Decision    │     │  Interface   │     │   (Gazebo)   │
└──────────────┘     └──────────────┘     └──────┬───────┘
       ▲                                         │
       │              ┌──────────────┐           │
       └──────────────│   Sensors    │◀──────────┘
                      │   (Odom)     │
                      └──────────────┘
```

---

## ROS2 Communication

### Topic Graph

```
/ethobot/swarm_state          [SwarmState]
    └── Published by: algorithm node
    └── Subscribed by: visualization, metrics

/ethobot/algorithm_status     [AlgorithmStatus]
    └── Published by: algorithm node
    └── Subscribed by: CLI, monitoring

/ethobot/<robot_id>/cmd_vel   [geometry_msgs/Twist]
    └── Published by: algorithm node
    └── Subscribed by: robot/simulation

/ethobot/<robot_id>/odom      [nav_msgs/Odometry]
    └── Published by: robot/simulation
    └── Subscribed by: algorithm node
```

### Service Graph

```
/ethobot/spawn_robot          [SpawnRobot]
    └── Server: simulation manager
    └── Client: launch system, CLI

/ethobot/set_algorithm_params [SetAlgorithmParams]
    └── Server: algorithm node
    └── Client: CLI, GUI
```

### Action Graph

```
/ethobot/run_optimization     [RunOptimization]
    └── Server: algorithm node
    └── Client: scenario manager, CLI
```

---

## Threading Model

```
┌─────────────────────────────────────────────────────────┐
│                    Algorithm Node                        │
├─────────────────────────────────────────────────────────┤
│  Main Thread (rclcpp spin)                              │
│  ├── Service callbacks                                  │
│  ├── Topic subscriptions                                │
│  └── Timer callbacks                                    │
│                                                         │
│  Action Server Thread                                   │
│  └── Optimization execution                             │
│      └── Algorithm::step() loop                         │
│                                                         │
│  Publisher Thread (async)                               │
│  └── SwarmState publishing                              │
└─────────────────────────────────────────────────────────┘
```

---

## Configuration

### Parameter Hierarchy

```yaml
# config/pso_params.yaml
ethobot:
  algorithm:
    type: "pso"
    pso:
      population_size: 50
      max_iterations: 100
      inertia_weight: 0.7
      cognitive_coeff: 1.5
      social_coeff: 1.5

  robot:
    type: "ground"
    max_linear_velocity: 0.5
    max_angular_velocity: 1.0

  simulation:
    world: "maze_simple"
    update_rate: 100.0
```

---

## Extension Points

### Adding a New Algorithm

1. Create class inheriting from `AlgorithmBase`
2. Implement required methods: `initialize()`, `step()`, `solve()`
3. Create ROS2 node wrapper
4. Add parameter configuration
5. Register in launch system

### Adding a New Robot Type

1. Create class inheriting from `RobotInterface`
2. Implement required methods: `get_pose()`, `set_velocity()`
3. Create ROS2 node wrapper
4. Add URDF/SDF model
5. Configure Gazebo plugins

---

## Performance Considerations

### Optimization
- Use Eigen for vectorized math operations
- Minimize copies with move semantics
- Batch publish operations
- Use async spinners for high-frequency loops

### Memory
- Pre-allocate particle vectors
- Use object pools for message creation
- Avoid dynamic allocation in hot paths

### Real-time
- Separate optimization from control loops
- Use deadline QoS for time-critical topics
- Profile and optimize critical paths
