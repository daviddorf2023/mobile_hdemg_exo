cmake_minimum_required(VERSION 3.0.2)
project(h3_control_client)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  controller_manager
  controller_manager_msgs
  roscpp
  std_msgs
)






