cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager)

add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
  # Camunda API
    camunda_c_api
  # BPMN API
    bpmn_interface
  # Architecture Packages
    architecture_msgs
  # ROS Packages
    roscpp
    message_runtime
)

find_package(GTest REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    # Camunda API
    camunda_c_api
    # BPMN API
    bpmn_interface
    # Architecture Packages
    architecture_msgs
    # ROS Packages
    roscpp
    message_runtime
  DEPENDS
    GTest
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

## BehaviorManager Library
add_library(${PROJECT_NAME}
  src/resource_pool.cpp
  src/monitored_role.cpp
  src/monitored_resource.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Test for MonitoredResource
#add_executable(test_monitored_resource
#  test/test_monitored_resource.cpp
#  src/monitored_resource.cpp
#)
#set_target_properties(test_monitored_resource PROPERTIES OUTPUT_NAME test_monitored_resource PREFIX "")
#add_dependencies(test_monitored_resource ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_monitored_resource ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

## Test for MonitoredRole
#add_executable(test_monitored_role
#  test/test_monitored_role.cpp
#  src/monitored_resource.cpp
#  src/monitored_role.cpp
#)
#set_target_properties(test_monitored_role PROPERTIES OUTPUT_NAME test_monitored_role PREFIX "")
#add_dependencies(test_monitored_role ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_monitored_role ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

## Test for ResourcePool
#add_executable(test_resource_pool
#  test/test_resource_pool.cpp
#  src/resource_pool.cpp
#  src/monitored_resource.cpp
#  src/monitored_role.cpp
#)
#set_target_properties(test_resource_pool PROPERTIES OUTPUT_NAME test_resource_pool PREFIX "")
#add_dependencies(test_resource_pool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_resource_pool ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

