cmake_minimum_required(VERSION 3.0.2)
project(h3_config_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  std_msgs
  h3_msgs
  h3_hardware_interface
)

catkin_package(INCLUDE_DIRS include 
CATKIN_DEPENDS controller_interface hardware_interface  roscpp std_msgs h3_hardware_interface h3_msgs)

include_directories( include ${catkin_INCLUDE_DIRS})

add_library(h3_config_controller src/h3_config_controller.cpp)

install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
)

install(FILES controller_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



