cmake_minimum_required(VERSION 3.0.2)
project(h3_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  controller_manager
  controller_manager_msgs
  roscpp
  std_msgs
  hardware_interface
)

# find PCAN libraries
set( PROJECT_LINK_LIBS libpcanbasic.so)
link_directories( /usr/lib)

catkin_package(
INCLUDE_DIRS include
  LIBRARIES h3_hardware_interface
  CATKIN_DEPENDS control_msgs control_toolbox controller_interface controller_manager controller_manager_msgs roscpp std_msgs hardware_interface
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_library(h3_hardware_interface src/h3_hardware_interface.cpp src/h3_handle.cpp src/h3_logger.cpp)
add_dependencies(h3_hardware_interface h3_hardware_interface)

add_executable(h3_hardware_interface_node src/h3_hardware_interface_node.cpp)
target_link_libraries(h3_hardware_interface_node h3_hardware_interface ${catkin_LIBRARIES} ${PROJECT_LINK_LIBS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
  launch/h3_hardware_interface.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
config/h3_hardware.yaml
DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/config"
)
