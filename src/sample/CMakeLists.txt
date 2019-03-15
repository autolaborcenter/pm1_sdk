cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 11)

add_executable(pm1_sdk_sample main.cpp adjust_rudder.h)
target_link_libraries(pm1_sdk_sample pm1_sdk)
