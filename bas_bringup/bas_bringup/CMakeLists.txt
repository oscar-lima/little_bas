cmake_minimum_required(VERSION 2.8.3)
project(bas_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

# include_directories(include)
#include_directories(bas_bringup
#	include/
#)

# include_directories(include)
#include_directories(bas_bringup_node
#	include/
#	${catkin_INCLUDE_DIRS}
#)

## Declare a cpp library
#add_library(bas_bringup
#	src/l6ac-kt_arm_driver.cpp
#)

## Declare a cpp executable
#add_executable(bas_bringup_node 
#	nodes/bas_bringup_node.cpp
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(bas_bringup
#	/usr/lib/libserial.so
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(bas_bringup_node
#	bas_bringup
#	${catkin_LIBRARIES}
#)
