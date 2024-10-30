cmake_minimum_required(VERSION 3.5)
project(navigation_handler)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# Declare the executable
add_executable(navigation_handler src/navigation_handler.cpp)

# Link libraries
ament_target_dependencies(navigation_handler
  rclcpp
  sensor_msgs
  nav_msgs
  rclcpp_action
  nav2_msgs
  geometry_msgs
  example_interfaces
)

# Install
install(TARGETS navigation_handler
  DESTINATION lib/${PROJECT_NAME})

ament_package()
