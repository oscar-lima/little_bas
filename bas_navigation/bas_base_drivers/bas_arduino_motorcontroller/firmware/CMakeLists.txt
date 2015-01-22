cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(bas_arduino_motorcontroller
  SRCS bas_arduino_motorcontroller.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/arduino/arduino_one_9523335323135181E1B2
)
