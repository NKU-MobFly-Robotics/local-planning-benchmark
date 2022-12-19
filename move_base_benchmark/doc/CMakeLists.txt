cmake_minimum_required(VERSION 2.8.3)
project(metric_evaluation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

add_executable(${PROJECT_NAME} metrics.cpp)
