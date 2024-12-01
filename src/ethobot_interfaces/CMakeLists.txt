cmake_minimum_required(VERSION 3.8)
project(ethobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ParticleState.msg"
  "msg/SwarmState.msg"
  "msg/RobotState.msg"
  "msg/SwarmCommand.msg"
  "msg/AlgorithmStatus.msg"
)

set(srv_files
  "srv/SpawnRobot.srv"
  "srv/SetAlgorithmParams.srv"
  "srv/GetMetrics.srv"
)

set(action_files
  "action/RunOptimization.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
