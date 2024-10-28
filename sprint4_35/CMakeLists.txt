cmake_minimum_required(VERSION 3.5)
project(sprint4_35)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)



# Declare the executable and link dependencies
add_executable(sprint4_35 src/sprint4_35.cpp)

ament_target_dependencies(sprint4_35 rclcpp sensor_msgs geometry_msgs OpenCV yaml-cpp nav_msgs nav2_msgs tf2 tf2_geometry_msgs) 

target_link_libraries(sprint4_35
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp  # Ensure this is here
)


install(TARGETS
  sprint4_35
  DESTINATION lib/${PROJECT_NAME})

#
ament_package()
