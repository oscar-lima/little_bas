cmake_minimum_required(VERSION 2.8.3)
project(bas_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bas_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bas_description
#   src/${PROJECT_NAME}/bas_description.cpp
# )

## Declare a cpp executable
add_executable(parser src/parser.cpp)

target_link_libraries(parser
   ${catkin_LIBRARIES}
)
