cmake_minimum_required(VERSION 2.8.3)
project(behavior_listener)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  architecture_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES behavior_listener
  CATKIN_DEPENDS
    rospy
    std_msgs
    architecture_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  src/bpmn.py
  src/behavior_listener_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

