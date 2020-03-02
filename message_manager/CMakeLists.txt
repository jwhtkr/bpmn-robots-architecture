cmake_minimum_required(VERSION 2.8.3)
project(message_manager)

add_compile_options(-std=c++17 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    launcher_helpers
    bpmn_interface
    camunda_c_api
    architecture_msgs
)

find_package(cpprestsdk REQUIRED COMPONENTS cpprest)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    launcher_helpers
    bpmn_interface
    camunda_c_api
    architecture_msgs
  DEPENDS
    cpprest
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cpprestsdk_INCLUDE_DIRS}
)

## Message Manager Library
add_library(${PROJECT_NAME}
  src/message_manager.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Message Manager Node
add_executable(${PROJECT_NAME}_node
  src/message_manager_node.cpp
  src/message_manager.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
