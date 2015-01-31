cmake_minimum_required(VERSION 2.8.3)
project(bas_environments)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

# include_directories(include)
#include_directories(bas_environments
#	include/
#)

# include_directories(include)
#include_directories(bas_environments_node
#	include/
#	${catkin_INCLUDE_DIRS}
#)

## Declare a cpp library
#add_library(bas_environments
#	src/l6ac-kt_arm_driver.cpp
#)

## Declare a cpp executable
#add_executable(bas_environments_node 
#	nodes/bas_environments_node.cpp
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(bas_environments
#	/usr/lib/libserial.so
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(bas_environments_node
#	bas_environments
#	${catkin_LIBRARIES}
#)
