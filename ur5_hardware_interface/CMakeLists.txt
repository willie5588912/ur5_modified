cmake_minimum_required(VERSION 2.8.3)
project(ur5_hardware_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ur5_hardware_interface 
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ur5_hardware_interface
  src/ur5_hardware_interface.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(ur5_hardware_interface
  ${catkin_LIBRARIES}
)

