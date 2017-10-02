cmake_minimum_required(VERSION 2.8.3)
project(enif_iuc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  dji_sdk
  message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Signal.msg
  Waypoint.msg
  WaypointTask.msg
  AgentTakeoff.msg
  AgentState.msg
  AgentMPS.msg
  AgentWaypointTask.msg
  AgentGlobalPosition.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs mps_driver dji_sdk
  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS #other_catkin_pkg
  message_runtime  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


## Declare a cpp executable
add_executable(enif_iuc_quad
  src/enif_iuc_quad.cpp
)

add_executable(enif_iuc_ground
  src/enif_iuc_ground.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(enif_iuc_quad
  ${catkin_LIBRARIES}
)


target_link_libraries(enif_iuc_ground
  ${catkin_LIBRARIES}
)
