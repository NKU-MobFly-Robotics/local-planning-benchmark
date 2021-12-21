cmake_minimum_required(VERSION 2.8.3)
project(metric_evaluation)

SET(CMAKE_BUILD_TYPE Release)  # Debug, Release
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME} metrics.cpp)
