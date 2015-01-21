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
  common/include
  common/include/${PROJECT_NAME} 
)

add_library(volksbot_motorcontroller 
  common/src/robot_factory.cpp
  common/src/robot_platform_volksbot_rt.cpp
  common/src/robot_platform_differential_drive.cpp
  common/src/timer.cpp
  common/src/com_adapter.cpp
  common/src/layer/communication_layer.cpp
  common/src/layer/translation_layer.cpp
  common/src/layer/vmc_api.cpp
  common/src/storage/motor.cpp
  common/src/storage/storage.cpp
  common/src/support/channel.cpp
  common/src/support/data.cpp
  common/src/support/error.cpp
  common/src/support/message.cpp
  common/src/support/multisend.cpp
  common/src/support/request.cpp
  common/src/support/send_two.cpp
  common/src/support/timestamp.cpp
)

add_executable(bas_motorcontroller_node 
  ros/src/volksbot_motorcontroller_node.cpp
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

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
