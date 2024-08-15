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

ament_target_dependencies(data_processor rclcpp sensor_msgs)

install(TARGETS
  data_processor
  DESTINATION lib/${PROJECT_NAME})

# Install launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()