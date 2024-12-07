cmake_minimum_required(VERSION 3.8)
project(ethobot_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethobot_core REQUIRED)
find_package(ethobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# PSO library
add_library(pso_solver SHARED
  src/pso_solver.cpp
)

target_include_directories(pso_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pso_solver
  rclcpp
  ethobot_core
  ethobot_interfaces
  geometry_msgs
)

# PSO test node (2D path planning demo)
add_executable(pso_path_planning_node
  src/pso_path_planning_node.cpp
)

target_include_directories(pso_path_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(pso_path_planning_node
  pso_solver
)

ament_target_dependencies(pso_path_planning_node
  rclcpp
  ethobot_core
  ethobot_interfaces
  geometry_msgs
  std_msgs
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS pso_solver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Swarm visualizer node
add_executable(swarm_visualizer_node
  src/swarm_visualizer_node.cpp
)

ament_target_dependencies(swarm_visualizer_node
  rclcpp
  ethobot_interfaces
  visualization_msgs
)

# Install executables
install(
  TARGETS pso_path_planning_node swarm_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz config
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Export
ament_export_include_directories(include)
ament_export_libraries(pso_solver)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp ethobot_core ethobot_interfaces geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
