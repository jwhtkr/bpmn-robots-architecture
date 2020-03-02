cmake_minimum_required(VERSION 2.8.3)
project(bpmn_interface)

add_compile_options(-std=c++11)
##add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    camunda_c_api
)

find_package(GTest REQUIRED)
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
    camunda_c_api
  DEPENDS
    cpprestsdk
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cpprestsdk_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

## BPMN Interface Library
add_library(${PROJECT_NAME}
  src/message_handler.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
)

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

#############
## Testing ##
#############

## Test for ProcessVariable Object
add_executable(test_process_variable test/test_process_variable.cpp)
set_target_properties(test_process_variable PROPERTIES OUTPUT_NAME test_process_variable PREFIX "")
add_dependencies(test_process_variable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_process_variable
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for TaskLock
add_executable(test_task_lock test/test_task_lock.cpp)
set_target_properties(test_task_lock PROPERTIES OUTPUT_NAME test_task_lock PREFIX "")
add_dependencies(test_task_lock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_task_lock
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## MessageHandler Test
add_executable(test_message_handler
  test/test_message_handler.cpp
  src/message_handler.cpp
)
set_target_properties(test_message_handler PROPERTIES OUTPUT_NAME test_message_handler PREFIX "")
add_dependencies(test_message_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_message_handler
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for ErrorHandler object
add_executable(test_error_handler test/test_error_handler.cpp)
set_target_properties(test_error_handler PROPERTIES OUTPUT_NAME test_error_handler PREFIX "")
add_dependencies(test_error_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_error_handler
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

