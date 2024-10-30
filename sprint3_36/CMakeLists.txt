cmake_minimum_required(VERSION 3.5)
project(sprint3_36)

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


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)



# Declare the executable and link dependencies
add_executable(sprint3_36 src/sprint3_36.cpp)

ament_target_dependencies(sprint3_36 rclcpp sensor_msgs geometry_msgs OpenCV yaml-cpp nav_msgs tf2 tf2_geometry_msgs) 

target_link_libraries(sprint3_36
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp  # Ensure this is here
)


install(TARGETS
  sprint3_36
  DESTINATION lib/${PROJECT_NAME})

#
ament_package()
