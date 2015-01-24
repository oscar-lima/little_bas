cmake_minimum_required(VERSION 2.8.3)
project(bas_motorcontroller)

find_package(catkin REQUIRED
  COMPONENTS
     geometry_msgs
     nav_msgs
     cob_msgs
     roscpp
     sensor_msgs
     tf
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    cob_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME} 
)

add_library(volksbot_motorcontroller 
  src/robot_factory.cpp
  src/robot_platform_volksbot_rt.cpp
  src/robot_platform_differential_drive.cpp
  src/timer.cpp
  src/com_adapter.cpp
  src/layer/communication_layer.cpp
  src/layer/translation_layer.cpp
  src/layer/vmc_api.cpp
  src/storage/motor.cpp
  src/storage/storage.cpp
  src/support/channel.cpp
  src/support/data.cpp
  src/support/error.cpp
  src/support/message.cpp
  src/support/multisend.cpp
  src/support/request.cpp
  src/support/send_two.cpp
  src/support/timestamp.cpp
)

add_executable(bas_motorcontroller_node 
  nodes/volksbot_motorcontroller_node.cpp
)
add_dependencies(bas_motorcontroller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(bas_motorcontroller_node
  volksbot_motorcontroller  
  ${catkin_LIBRARIES}
)

install(TARGETS volksbot_motorcontroller volksbot_motorcontroller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS bas_motorcontroller_node bas_motorcontroller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
