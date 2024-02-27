cmake_minimum_required(VERSION 2.8.3)
project(camunda_c_api)

add_compile_options(-std=c++11)
## add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cpprestsdk REQUIRED COMPONENTS cpprest)
find_package(GTest REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    cpprestsdk::cpprest
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cpprestsdk_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

## Camunda C API Library
add_library(${PROJECT_NAME}
  src/json.cpp
  src/lock_request.cpp
  src/lock_response.cpp
  src/variables.cpp
  src/complete_request.cpp
  src/topics.cpp
  src/extend_lock.cpp
  src/throw_signal.cpp
  src/json_print.cpp
  src/bpmn_error.cpp
  src/bpmn_exception.cpp
  src/camunda_error.cpp
  src/camunda_exception.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
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

## Test for printJson Functions
add_executable(test_print_json
  test/test_print_json.cpp
  src/json_print.cpp
)
set_target_properties(test_print_json PROPERTIES OUTPUT_NAME test_print_json PREFIX "")
add_dependencies(test_print_json ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_print_json
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for Json Object
add_executable(test_json
  test/test_json.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_json PROPERTIES OUTPUT_NAME test_json PREFIX "")
add_dependencies(test_json ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_json
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for LockRequest Object
add_executable(test_lock_request
  test/test_lock_request.cpp
  src/lock_request.cpp
  src/topics.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_lock_request PROPERTIES OUTPUT_NAME test_lock_request PREFIX "")
add_dependencies(test_lock_request ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lock_request
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for LockResponse Object
add_executable(test_lock_response
  test/test_lock_response.cpp
  src/lock_response.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_lock_response PROPERTIES OUTPUT_NAME test_lock_response PREFIX "")
add_dependencies(test_lock_response ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lock_response
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for Variables Object
add_executable(test_variables
  test/test_variables.cpp
  src/variables.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_variables PROPERTIES OUTPUT_NAME test_variables PREFIX "")
add_dependencies(test_variables ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_variables
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for Topics Object
add_executable(test_topics
  test/test_topics.cpp
  src/topics.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_topics PROPERTIES OUTPUT_NAME test_topics PREFIX "")
add_dependencies(test_topics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_topics
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for CompleteRequest Object
add_executable(test_complete_request
  test/test_complete_request.cpp
  src/complete_request.cpp
  src/variables.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_complete_request PROPERTIES OUTPUT_NAME test_complete_request PREFIX "")
add_dependencies(test_complete_request ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_complete_request
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for ExtendLock Object
add_executable(test_extend_lock
  test/test_extend_lock.cpp
  src/extend_lock.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_extend_lock PROPERTIES OUTPUT_NAME test_extend_lock PREFIX "")
add_dependencies(test_extend_lock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_extend_lock
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for ThrowSignal Object
add_executable(test_throw_signal
  test/test_throw_signal.cpp
  src/throw_signal.cpp
  src/variables.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_throw_signal PROPERTIES OUTPUT_NAME test_throw_signal PREFIX "")
add_dependencies(test_throw_signal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_throw_signal
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for CamundaException Object
add_executable(test_camunda_exception
  test/test_camunda_exception.cpp
  src/camunda_exception.cpp
)
set_target_properties(test_camunda_exception PROPERTIES OUTPUT_NAME test_camunda_exception PREFIX "")
add_dependencies(test_camunda_exception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_camunda_exception
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for BpmnException Object
add_executable(test_bpmn_exception
  test/test_bpmn_exception.cpp
  src/bpmn_exception.cpp
  src/variables.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_bpmn_exception PROPERTIES OUTPUT_NAME test_bpmn_exception PREFIX "")
add_dependencies(test_bpmn_exception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bpmn_exception
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for BpmnError Object
add_executable(test_bpmn_error
  test/test_bpmn_error.cpp
  src/bpmn_error.cpp
  src/bpmn_exception.cpp
  src/variables.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_bpmn_error PROPERTIES OUTPUT_NAME test_bpmn_error PREFIX "")
add_dependencies(test_bpmn_error ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bpmn_error
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

## Test for CamundaError Object
add_executable(test_camunda_error
  test/test_camunda_error.cpp
  src/camunda_error.cpp
  src/camunda_exception.cpp
  src/json.cpp
  src/json_print.cpp
)
set_target_properties(test_camunda_error PROPERTIES OUTPUT_NAME test_camunda_error PREFIX "")
add_dependencies(test_camunda_error ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_camunda_error
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

