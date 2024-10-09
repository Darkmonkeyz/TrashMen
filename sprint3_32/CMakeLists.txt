cmake_minimum_required(VERSION 3.10)

# Project name
project(ImageOverlay)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(image_overlay main.cpp)

# Link OpenCV libraries to the executable
target_link_libraries(image_overlay ${OpenCV_LIBS})
