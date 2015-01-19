cmake_minimum_required(VERSION 2.8.3)
project(bas_arm_driver_tester)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
cfg/arm_tester.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bas_arm_driver_tester
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bas_arm_driver_tester
#   src/${PROJECT_NAME}/bas_arm_driver_tester.cpp
# )

## Declare a cpp executable
add_executable(bas_arm_driver_tester src/bas_arm_driver_tester_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bas_arm_driver_tester_node bas_arm_driver_tester_generate_messages_cpp)

add_dependencies(bas_arm_driver_tester ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(bas_arm_driver_tester
	${catkin_LIBRARIES}
)
