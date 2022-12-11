cmake_minimum_required(VERSION 3.0.2)
project(move_base_benchmark)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    base_local_planner
    clear_costmap_recovery
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    message_generation
    move_base_msgs
    nav_core
    nav_msgs
    navfn
    pluginlib
    roscpp
    rospy
    rotate_recovery
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# Ceres
find_package(Ceres REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MoveBase.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# move_base_benchmark
add_library(move_base_benchmark
  src/move_base_benchmark.cpp
  src/obs_dist_calculator.cpp
)
target_link_libraries(move_base_benchmark
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(move_base_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_benchmark_node
  src/move_base_benchmark_node.cpp
)
add_dependencies(move_base_benchmark_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_benchmark_node move_base_benchmark)
set_target_properties(move_base_benchmark_node PROPERTIES OUTPUT_NAME move_base_benchmark)

# simple_navigation_goals
add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
