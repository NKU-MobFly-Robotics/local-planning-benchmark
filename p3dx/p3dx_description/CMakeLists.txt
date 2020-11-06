cmake_minimum_required(VERSION 2.8.3)
project(p3dx_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)