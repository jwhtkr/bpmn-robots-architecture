cmake_minimum_required(VERSION 2.8.3)
project(thread_pool)

add_compile_options(-std=c++17)
##add_compile_options(-g -Og)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    architecture_msgs
    node_server_msgs
    launcher_helpers
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
    architecture_msgs
    node_server_msgs
    launcher_helpers
  DEPENDS
    cpprestsdk:cpprest
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cpprestsdk_INCLUDE_DIRS}
)

## ThreadPool library
add_library(${PROJECT_NAME}
  src/thread_pool.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
)

## Thread Pool Node
add_executable(${PROJECT_NAME}_node
  src/thread_pool_node.cpp
  src/thread_pool.cpp
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

