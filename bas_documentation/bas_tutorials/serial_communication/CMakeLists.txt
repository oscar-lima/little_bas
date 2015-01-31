cmake_minimum_required(VERSION 2.8.7)
project(serial)

add_executable(serial src/serial.cpp)

target_link_libraries(serial
/usr/lib/libserial.so
)
