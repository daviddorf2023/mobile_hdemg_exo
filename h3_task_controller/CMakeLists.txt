cmake_minimum_required(VERSION 3.0.2)
project(h3_task_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  h3_hardware_interface
  pluginlib
  roscpp
  std_msgs
  h3_msgs
)

catkin_package(
  CATKIN_DEPENDS controller_interface h3_hardware_interface pluginlib roscpp std_msgs h3_msgs
)


include_directories( include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/h3_task_controller.cpp)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

install(FILES
  task_controller_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

