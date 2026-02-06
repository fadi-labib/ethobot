cmake_minimum_required(VERSION 3.8)
project(ethobot_robots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Robot controller library
add_library(robot_controller SHARED
  src/robot_controller.cpp
)

ament_target_dependencies(robot_controller
  rclcpp
  ethobot_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Waypoint follower node
add_executable(waypoint_follower_node
  src/waypoint_follower_node.cpp
)

target_link_libraries(waypoint_follower_node
  robot_controller
)

ament_target_dependencies(waypoint_follower_node
  rclcpp
  ethobot_interfaces
  geometry_msgs
  nav_msgs
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS robot_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(robot_controller)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  ethobot_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(robot_controller_test
    test/robot_controller_test.cpp
  )
  target_include_directories(robot_controller_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(robot_controller_test robot_controller)
  ament_target_dependencies(robot_controller_test
    rclcpp
    ethobot_interfaces
    geometry_msgs
    nav_msgs
  )
endif()

ament_package()
