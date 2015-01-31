cmake_minimum_required(VERSION 2.8.3)
project(bas_arm_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  std_msgs
)

catkin_package(
)

# include_directories(include)
include_directories(bas_arm_driver
	include/
)

# include_directories(include)
include_directories(bas_arm_driver_node
	include/
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bas_arm_driver
	src/l6ac-kt_arm_driver.cpp
)

## Declare a cpp executable
add_executable(bas_arm_driver_node 
	nodes/bas_arm_driver_node.cpp
)

## Declare a cpp executable (sate publisher)
add_executable(bas_state_publisher
	nodes/state_publisher.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(bas_arm_driver
	/usr/lib/libserial.so
)

## Specify libraries to link a library or executable target against
target_link_libraries(bas_state_publisher
	${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(bas_arm_driver_node
	bas_arm_driver
	${catkin_LIBRARIES}
)
