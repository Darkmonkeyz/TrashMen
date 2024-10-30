cmake_minimum_required(VERSION 3.5)
project(data_processor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)

# Declare the executable and link dependencies
add_executable(data_processor src/data_processor.cpp)
add_executable(sprint1 src/sprint1.cpp)

ament_target_dependencies(data_processor rclcpp sensor_msgs)
ament_target_dependencies(sprint1 rclcpp sensor_msgs)

install(TARGETS
  data_processor
  sprint1
  DESTINATION lib/${PROJECT_NAME})

#
ament_package()
