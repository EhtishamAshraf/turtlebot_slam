cmake_minimum_required(VERSION 3.0.2)
project(mrob_highlight_controller)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
