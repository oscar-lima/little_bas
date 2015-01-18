cmake_minimum_required(VERSION 2.8.3)
project(bas_arm_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bas_arm_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bas_arm_driver
#   src/${PROJECT_NAME}/bas_arm_driver.cpp
# )

## Declare a cpp executable
# add_executable(bas_arm_driver_node src/bas_arm_driver_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(bas_arm_driver_node
#   ${catkin_LIBRARIES}
# )
